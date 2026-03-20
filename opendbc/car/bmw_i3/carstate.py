from collections import Counter, deque

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.interfaces import CarStateBase
from opendbc.car.bmw_i3.values import DBC
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter
WheelSpeeds = structs.CarState.WheelSpeeds
ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  @staticmethod
  def _decode_eps_angle(angle_raw: float) -> float:
    # Live routes show frame 51 behaving like a wrapped steering-wheel angle
    # rather than a clean signed +-1000 deg signal. Re-center it to the nearest
    # turn within a practical steering-wheel range until the final DBC formula
    # is closed mathematically.
    return ((float(angle_raw) + 512.0) % 1024.0) - 512.0

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.shifter_values = CANDefine(DBC[CP.carFingerprint][Bus.pt]).dv.get("DRIVE_STATE_EXPERIMENTAL", {})
    # These helpers originally came from historical route correlation work, but
    # 131/135/97 remain the current primary stock ACC/TJA state/button families.
    self.stock_acc_ctrl_state = 0
    self.stock_acc_ctrl_gate = 0
    self.stock_tja_active = False
    self.stock_acc_base_armed = False
    self.stock_assist_advanced = False
    self.stock_stalk_main_a = 0
    self.stock_stalk_main_b = 0
    self.stock_acc_button = False
    self.stock_tja_button = False
    self.stock_speed_adjust = False
    self.legacy_main_button = 0
    self.drive_state_kind_hist = deque(maxlen=3)
    self.drive_state_gear_est = GearShifter.unknown
    self.main_cruise_button = 0
    self.long_59_wb = 0
    self.long_59_wc = 0
    self.long_59_b3 = 0
    self.long_59_b5 = 0
    self.long_54_wb = 0
    self.long_54_wc = 0
    self.long_54_b4 = 0
    self.long_54_b6 = 0
    self.long_up_217_raw16 = 0
    self.long_up_217_word23 = 0
    self.long_up_217_value = 0
    self.long_up_217_i4_compat12 = 0
    self.long_up_796_raw16 = 0
    self.long_up_796_b1 = 0
    self.stock_long_upstream_mode = "unknown"
    self.stock_long_upstream_confidence = "none"
    self.driver_steer_torque = 0.0
    self.vehicle_speed_kph = 0.0
    self.stock_lat96_phase = 0
    self.stock_lat96_b1 = 0
    self.stock_lat96_b2 = 0
    self.stock_lat96_b3 = 0
    self.stock_lat112_b5 = 0
    self.stock_lat116_b5 = 0
    self.stock_lat_active_hint = False
    self.stock_lat_dir_hint = "unknown"
    self.stock_lat_dir_confidence = "none"
    self.stock_lat_mag_hint = 0.0
    self.stock_lat_mag_confidence = "none"

  @staticmethod
  def _stock_lat_dir_from_phase_b1(phase: int, b1: int) -> tuple[str, str]:
    # Best current route-derived local classifier for LAT96.byte1.
    # This is intentionally phase-local, not a fake global signed command.
    # Strongest phase: 60. Secondary supporting phases: 24 and 8.
    if phase == 60:
      return ("right", "high") if b1 > 112 else ("left", "high")
    if phase == 24:
      return ("right", "medium") if b1 > 81 else ("left", "medium")
    if phase == 8:
      return ("right", "medium") if b1 > 150 else ("left", "medium")
    return ("unknown", "none")

  @staticmethod
  def _stock_lat_mag_from_phase_b1(phase: int, b1: int) -> tuple[float, str]:
    # Current route-backed magnitude proxy is intentionally normalized, not a
    # fake degree-accurate command. Phase 60 is the only family with strong
    # enough signed correlation to call "high". Phases 24 and 8 remain weak.
    if phase == 60:
      thr = 112.083
      scale = 110.0
      return (min(1.0, abs(b1 - thr) / scale), "high")
    if phase == 24:
      thr = 80.833
      scale = 135.0
      return (min(1.0, abs(b1 - thr) / scale), "low")
    if phase == 8:
      thr = 149.5
      scale = 90.0
      return (min(1.0, abs(b1 - thr) / scale), "low")
    return (0.0, "none")

  @staticmethod
  def _stock_long_upstream_hint(acc217_raw16: int, brake796_b1: int) -> tuple[str, str]:
    # Best current route-backed upstream long interpretation:
    #   217.raw16 separates positive/coast
    #   796.byte1 separates negative/brake intent
    # Conservative buckets from aTarget fit:
    #   POS   -> 217.raw16 ~ 63334
    #   COAST -> 217.raw16 ~ 63364
    #   NEG   -> 796.byte1  ~ 13
    if brake796_b1 <= 0x0F:
      return ("negative", "medium")
    if acc217_raw16 <= 63349:
      return ("positive", "medium")
    if acc217_raw16 >= 63350:
      return ("coast", "medium")
    return ("unknown", "none")

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_state = can_parsers[Bus.pt]
    cp_flexray = can_parsers[Bus.cam]
    cp_can = can_parsers[Bus.party]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    ws = cp_flexray.vl.get("WHEEL_SPEED", {})
    ret.wheelSpeeds = WheelSpeeds(fl=float(ws.get("FL_SPEED_RAW", 0.0)) / 3.6,
                                  fr=float(ws.get("FR_SPEED_RAW", 0.0)) / 3.6,
                                  rl=float(ws.get("RL_SPEED_RAW", 0.0)) / 3.6,
                                  rr=float(ws.get("RR_SPEED_RAW", 0.0)) / 3.6)

    wheel_speeds = [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]
    wheel_speed_avg = float(sum(wheel_speeds) / 4.0)
    # Match the dynm/SP2018 BMW method semantically: consume vehicle speed only
    # from the valid m3 subframe of frame 55, and keep frame 46 wheel speeds as
    # fallback / consistency support.
    vehicle_speed = cp_flexray.vl.get("VEHICLE_SPEED_PROV", {})
    vehicle_speed_cycle = int(vehicle_speed.get("VEHICLE_SPEED_CYCLE_RAW", -1))
    if vehicle_speed_cycle == 3:
      self.vehicle_speed_kph = float(vehicle_speed.get("VEHICLE_SPEED_BMW", self.vehicle_speed_kph))
    ret.vEgoRaw = self.vehicle_speed_kph * CV.KPH_TO_MS if self.vehicle_speed_kph > 0.0 else wheel_speed_avg
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgoRaw
    ret.standstill = ret.vEgoRaw < 0.1

    eps_angle_raw = float(cp_flexray.vl.get("EPS_ANGLE", {}).get("STEERING_ANGLE_RAW", 0.0))
    ret.steeringAngleDeg = self._decode_eps_angle(eps_angle_raw)
    steer_torque = cp_flexray.vl.get("STEER_TORQUE", {})
    steer_torque_cycle = int(steer_torque.get("STEER_TORQUE_CYCLE_RAW", -1))
    if steer_torque_cycle == 0:
      self.driver_steer_torque = float(steer_torque.get("DRIVER_STEER_TORQUE_BMW", self.driver_steer_torque))
    ret.steeringTorque = self.driver_steer_torque
    ret.steeringPressed = abs(ret.steeringTorque) > 1.5

    ret.yawRate = float(cp_flexray.vl.get("DYNAMICS_YAW_PROV", {}).get("YAW_RATE_RAW_A", 0.0))
    # No physical brake-pressure value is closed yet. Keep brake at zero and use
    # brakePressed from PT-CAN 796 for the boolean path.
    ret.brake = 0.0

    # Best current stock longitudinal helper branches:
    #   59 -> powertrain-intent proxy
    #   54 -> brake-blend / regen-support proxy
    long_59 = cp_flexray.vl.get("LONG_TX_POWERTRAIN_CANDIDATE", {})
    self.long_59_wb = int(long_59.get("LONG_TX_POWERTRAIN_WORD_B", 0))
    self.long_59_wc = int(long_59.get("LONG_TX_POWERTRAIN_WORD_C", 0))
    self.long_59_b3 = int(long_59.get("LONG_TX_POWERTRAIN_BYTE_3", 0))
    self.long_59_b5 = int(long_59.get("LONG_TX_POWERTRAIN_BYTE_5", 0))

    long_54 = cp_flexray.vl.get("LONG_TX_BRAKE_BLEND_CANDIDATE", {})
    self.long_54_wb = int(long_54.get("LONG_TX_BRAKE_BLEND_WORD_B", 0))
    self.long_54_wc = int(long_54.get("LONG_TX_BRAKE_BLEND_WORD_C", 0))
    self.long_54_b4 = int(long_54.get("LONG_TX_BRAKE_BLEND_BYTE_4", 0))
    self.long_54_b6 = int(long_54.get("LONG_TX_BRAKE_BLEND_BYTE_6", 0))

    pt_accel = cp_can.vl.get("PTCAN_ACCELERATOR_CANDIDATE", {})
    self.long_up_217_raw16 = int(pt_accel.get("ACCEL_RAW_PT_CAN", 0))
    self.long_up_217_word23 = int(pt_accel.get("ACCELERATOR_WORD23_PT_CAN", 0))
    self.long_up_217_value = max(0, min(4000, int(pt_accel.get("ACCELERATOR_VALUE_PT_CAN", 0))))
    self.long_up_217_i4_compat12 = int(pt_accel.get("ACCELERATOR_I4_COMPAT_PT_CAN", 0))

    pt_brake = cp_can.vl.get("PTCAN_BRAKE_PRESSED_CANDIDATE", {})
    self.long_up_796_raw16 = int(pt_brake.get("BRAKE_PRESSED_RAW_PT_CAN", 0))
    self.long_up_796_b1 = int(pt_brake.get("BRAKE_PRESSED_BYTE_1_PT_CAN", 0xFF))
    self.stock_long_upstream_mode, self.stock_long_upstream_confidence = self._stock_long_upstream_hint(
      self.long_up_217_raw16, self.long_up_796_b1
    )
    brake_pedal_candidate = int(cp_can.vl.get("PTCAN_BRAKE_PEDAL_CANDIDATE", {}).get("PTCAN_BRAKE_PEDAL_BYTE_0", 0))

    lat96 = cp_flexray.vl.get("LAT_STOCK_TX_PAYLOAD_CANDIDATE", {})
    self.stock_lat96_phase = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_0", 0))
    self.stock_lat96_b1 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_1", 0))
    self.stock_lat96_b2 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_2", 0))
    self.stock_lat96_b3 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_3", 0))
    lat112 = cp_flexray.vl.get("ACC_STALK_TJA_CANDIDATE_B", {})
    lat116 = cp_flexray.vl.get("ACC_STALK_TJA_CANDIDATE_C", {})
    self.stock_lat112_b5 = int(lat112.get("LAT_STOCK_MAIN_BYTE_5", 0))
    self.stock_lat116_b5 = int(lat116.get("LAT_STOCK_SUPPORT_BYTE_5", 0))
    # Best current live discriminator from route work:
    #   112.byte5 bit5 set   -> manual/off tendency
    #   112.byte5 bit5 clear -> assisted/TJA tendency
    self.stock_lat_active_hint = (self.stock_lat112_b5 & 0x20) == 0

    self.stock_lat_dir_hint, self.stock_lat_dir_confidence = self._stock_lat_dir_from_phase_b1(self.stock_lat96_phase, self.stock_lat96_b1)
    self.stock_lat_mag_hint, self.stock_lat_mag_confidence = self._stock_lat_mag_from_phase_b1(self.stock_lat96_phase, self.stock_lat96_b1)
    if not self.stock_lat_active_hint:
      self.stock_lat_dir_hint = "unknown"
      self.stock_lat_dir_confidence = "none"
      self.stock_lat_mag_hint = 0.0
      self.stock_lat_mag_confidence = "none"

    ret.gas = self.long_up_217_value / 4000.0
    ret.gasPressed = self.long_up_217_value > 0

    # Replicated parked brake route 000001a0 isolates a cleaner brakePressed
    # source than the old 796 heuristic:
    #   538.byte0 = 0x00 released, 0x80 pressed
    ret.brakePressed = brake_pedal_candidate == 0x80

    drive_state = cp_flexray.vl.get("DRIVE_STATE_EXPERIMENTAL", {})
    drive_cycle = int(drive_state.get("DRIVE_STATE_CYCLE_COMPAT", 0))
    drive_kind_b11 = int(drive_state.get("DRIVE_STATE_KIND_BYTE_11", 0))
    drive_kind_b14 = int(drive_state.get("DRIVE_STATE_KIND_BYTE_14", 0))
    if drive_cycle == 3:
      drive_key = None
      if drive_kind_b11 == 0x24:
        drive_key = GearShifter.drive
      elif drive_kind_b11 == 0x25:
        drive_key = GearShifter.reverse
      elif drive_kind_b11 == 0x22:
        if drive_kind_b14 == 0xEE:
          drive_key = GearShifter.park
        elif drive_kind_b14 == 0xEC:
          drive_key = GearShifter.neutral
      if drive_key is not None:
        self.drive_state_kind_hist.append(drive_key)
    if self.drive_state_kind_hist:
      # FlexRay addr 40 is multiplexed. The cycle==3 subframe carries the
      # cleanest gear discriminator. Byte 11 splits D/R from the 0x22 branch,
      # then byte 14 splits the 0x22 branch into:
      #   0xEE -> P
      #   0xEC -> N
      # while:
      #   0x24 -> D
      #   0x25 -> R
      # A very short majority window suppresses subframe churn without
      # smearing long state transitions.
      self.drive_state_gear_est = Counter(self.drive_state_kind_hist).most_common(1)[0][0]
    ret.gearShifter = self.drive_state_gear_est

    stock_ctrl_state = int(cp_state.vl.get("ACC_TJA_OLD_ROUTE_HELPER_E", {}).get("ACC_TJA_OLD_CTRL_STATE", 0))
    stock_ctrl_gate = int(cp_state.vl.get("ACC_TJA_OLD_ROUTE_HELPER_D", {}).get("ACC_TJA_OLD_CTRL_GATE", 0))
    stock_stalk_main_a = int(cp_flexray.vl.get("ACC_TJA_OLD_ROUTE_HELPER_B", {}).get("ACC_TJA_OLD_STALK_MAIN_A", 0))
    stock_stalk_main_b = int(cp_flexray.vl.get("ACC_TJA_OLD_ROUTE_HELPER_B", {}).get("ACC_TJA_OLD_STALK_MAIN_B", 0))
    self.stock_acc_ctrl_state = stock_ctrl_state
    self.stock_acc_ctrl_gate = stock_ctrl_gate
    self.stock_acc_base_armed = stock_ctrl_state == 16610
    self.stock_assist_advanced = stock_ctrl_state == 24802
    self.stock_tja_active = self.stock_assist_advanced
    self.stock_stalk_main_a = stock_stalk_main_a
    self.stock_stalk_main_b = stock_stalk_main_b
    # Legacy route correlation:
    # 30716/65282 -> ACC button family
    # 18684/65283 -> TJA button family
    # 5884/65282  -> speed stalk +/- family
    self.stock_acc_button = stock_stalk_main_a == 30716 and stock_stalk_main_b == 65282
    self.stock_tja_button = stock_stalk_main_a == 18684 and stock_stalk_main_b == 65283
    self.stock_speed_adjust = stock_stalk_main_a == 5884 and stock_stalk_main_b == 65282

    # Old and modern ACC-only/TJA routes show the clearest stable states here:
    # 35041/643 -> off baseline
    # 16610/3584 -> ACC base armed/ready state
    # 24802/(640 or 656) -> advanced assist state / actual managed-control branch
    acc_enabled = stock_ctrl_state in (16610, 24802)
    acc_available = acc_enabled or stock_ctrl_gate in (640, 656, 3584)

    ret.cruiseState.available = acc_available
    ret.cruiseState.enabled = acc_enabled
    ret.cruiseState.standstill = ret.standstill

    blinker_byte6 = int(cp_can.vl.get("PTCAN_BLINKER_STATE_CANDIDATE", {}).get("BLINKER_STATE_BYTE_6", 0))
    turn_left_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_LEFT_TURN_CANDIDATE", 0))
    turn_right_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_RIGHT_TURN_CANDIDATE", 0))
    turn_active_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_TURNSIGNAL_ACTIVE_CANDIDATE", 0))
    turn_idle_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_TURNSIGNAL_IDLE_CANDIDATE", 0))
    main_cruise_word = int(cp_can.vl.get("PTCAN_CRUISE_BUTTONS_MAIN", {}).get("CRUISE_BTN_MAIN_PT_CAN", 0))
    driver_door_state = int(cp_can.vl.get("PTCAN_DRIVER_DOOR_CANDIDATE", {}).get("DRIVER_DOOR_STATE_BYTE_2", 0))
    seatbelt_a = int(cp_can.vl.get("PTCAN_SEATBELT_CANDIDATE_A", {}).get("PTCAN_SEATBELT_BYTE_4_A", 0))
    seatbelt_b = int(cp_can.vl.get("PTCAN_SEATBELT_CANDIDATE_B", {}).get("PTCAN_SEATBELT_BYTE_2_B", 0))
    # Prefer the dedicated PT-CAN turn-signal helper frame over the noisier byte-family
    # reverse from 274. The bit layout is already described in the custom DBC and gives
    # us explicit left/right/active/idle candidates.
    turn_asserted = bool(turn_active_candidate) and not bool(turn_idle_candidate)
    ret.leftBlinker = turn_asserted and bool(turn_left_candidate) and not bool(turn_right_candidate)
    ret.rightBlinker = turn_asserted and bool(turn_right_candidate) and not bool(turn_left_candidate)
    # Replicated parked belt routes 0000019c/0000019d isolate two agreeing PT-CAN
    # helpers for the buckle state:
    #   435.byte4 : 0x00 latched, 0x04 unlatched
    #   663.byte2 : 0xF1 latched, 0xF0 unlatched
    # Require agreement instead of trusting a single raw frame.
    seatbelt_unlatched = (seatbelt_a == 0x04 and seatbelt_b == 0xF0)
    seatbelt_latched = (seatbelt_a == 0x00 and seatbelt_b == 0xF1)
    ret.seatbeltUnlatched = seatbelt_unlatched and not seatbelt_latched
    ret.doorOpen = driver_door_state == 1
    ret.stockAeb = False
    ret.stockFcw = False
    ret.espDisabled = False

    prev_main_cruise_button = self.main_cruise_button
    prev_legacy_main_button = self.legacy_main_button
    # Broad button-route scans show only two 415-word values with enough purity to
    # be worth mapping today:
    #   0x8015 -> SET family
    #   0x8016 -> RES family
    # The rest (+/-/distance) still overlap too much for a safe public mapping.
    self.main_cruise_button = {
      0x8015: 1,
      0x8016: 2,
    }.get(main_cruise_word, 0)
    # Latest isolated ACC/TJA route shows a much cleaner split on FlexRay 97:
    #   30716 / 65282 -> ACC main button family
    #   18684 / 65283 -> TJA / lane-assist main button family
    self.legacy_main_button = 0
    if self.stock_acc_button:
      self.legacy_main_button = 1
    elif self.stock_tja_button:
      self.legacy_main_button = 2

    ret.buttonEvents = [
      *create_button_events(self.main_cruise_button, prev_main_cruise_button, {
        1: ButtonType.setCruise,
        2: ButtonType.resumeCruise,
      }),
      *create_button_events(self.legacy_main_button, prev_legacy_main_button, {
        1: ButtonType.mainCruise,
        2: ButtonType.lkas,
      }),
    ]
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    dbc = DBC[CP.carFingerprint][Bus.pt]
    pt_messages = [
      ("ACC_TJA_OLD_ROUTE_HELPER_D", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_E", float("nan")),
    ]
    cam_messages = [
      ("WHEEL_SPEED", float("nan")),
      ("STEER_TORQUE", float("nan")),
      ("EPS_ANGLE", float("nan")),
      ("VEHICLE_SPEED_PROV", float("nan")),
      ("DYNAMICS_YAW_PROV", float("nan")),
      ("PEDAL_OR_HOLD_STATE_CANDIDATE", float("nan")),
      ("BRAKE_BLEND_CANDIDATE_B", float("nan")),
      ("LAT_STOCK_TX_PAYLOAD_CANDIDATE", float("nan")),
      ("ACC_STALK_TJA_CANDIDATE_B", float("nan")),
      ("ACC_STALK_TJA_CANDIDATE_C", float("nan")),
      ("DRIVE_STATE_EXPERIMENTAL", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_B", float("nan")),
    ]
    party_messages = [
      ("PTCAN_ACCELERATOR_CANDIDATE", float("nan")),
      ("PTCAN_BRAKE_PRESSED_CANDIDATE", float("nan")),
      ("PTCAN_BRAKE_PEDAL_CANDIDATE", float("nan")),
      ("PTCAN_BLINKER_STATE_CANDIDATE", float("nan")),
      ("PTCAN_TURNSIGNALS_CANDIDATE", float("nan")),
      ("PTCAN_SEATBELT_CANDIDATE_A", float("nan")),
      ("PTCAN_SEATBELT_CANDIDATE_B", float("nan")),
      ("PTCAN_CRUISE_BUTTONS_MAIN", float("nan")),
      ("PTCAN_DRIVER_DOOR_CANDIDATE", float("nan")),
    ]

    cp_pt = CANParser(dbc, pt_messages, 0)
    cp_cam = CANParser(dbc, cam_messages, 1)
    cp_party = CANParser(dbc, party_messages, 2)

    # This custom FlexRay/CAN gateway exports many stock BMW frames without
    # stable checksum/counter semantics. Follow the dynm/BMW approach:
    # register the messages we actually consume, then relax checks so data can
    # flow while reverse work is still in progress.
    for cp, msg_names in (
      (cp_pt, [name for name, _ in pt_messages]),
      (cp_cam, [name for name, _ in cam_messages]),
      (cp_party, [name for name, _ in party_messages]),
    ):
      for msg in msg_names:
        cp.dbc.name_to_msg[msg].ignore_checksum = True
        cp.dbc.name_to_msg[msg].ignore_counter = True

    return {
      Bus.pt: cp_pt,
      Bus.cam: cp_cam,
      Bus.party: cp_party,
    }
