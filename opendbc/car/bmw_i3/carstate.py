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
    self.driver_steer_torque = 0.0
    self.vehicle_speed_kph = 0.0

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

    ret.steeringAngleDeg = float(cp_flexray.vl.get("EPS_ANGLE", {}).get("STEERING_ANGLE_RAW", 0.0))
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

    gas_raw = int(cp_can.vl.get("PTCAN_ACCELERATOR_CANDIDATE", {}).get("ACCELERATOR_I4_COMPAT_PT_CAN", 0))
    ret.gasPressed = gas_raw > 200

    brake_can_byte1 = int(cp_can.vl.get("PTCAN_BRAKE_PRESSED_CANDIDATE", {}).get("BRAKE_PRESSED_BYTE_1_PT_CAN", 0xFF))
    ret.brakePressed = brake_can_byte1 < 0x10

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
    blinker_byte7 = int(cp_can.vl.get("PTCAN_BLINKER_STATE_CANDIDATE", {}).get("BLINKER_STATE_BYTE_7", 0))
    main_cruise_word = int(cp_can.vl.get("PTCAN_CRUISE_BUTTONS_MAIN", {}).get("CRUISE_BTN_MAIN_PT_CAN", 0))
    driver_door_state = int(cp_can.vl.get("PTCAN_DRIVER_DOOR_CANDIDATE", {}).get("DRIVER_DOOR_STATE_BYTE_2", 0))
    # Latest isolated parked routes show the clearest side split here:
    #   0x45 -> right indicator family
    #   0x25 -> left indicator family
    ret.rightBlinker = blinker_byte6 == 0x45
    ret.leftBlinker = blinker_byte6 == 0x25
    # Latest isolated parked seatbelt route shows:
    #   0xA5 -> buckled
    #   0xB5 -> unbuckled
    ret.seatbeltUnlatched = blinker_byte7 == 0xB5
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
      ("DRIVE_STATE_EXPERIMENTAL", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_B", float("nan")),
    ]
    party_messages = [
      ("PTCAN_ACCELERATOR_CANDIDATE", float("nan")),
      ("PTCAN_BRAKE_PRESSED_CANDIDATE", float("nan")),
      ("PTCAN_BLINKER_STATE_CANDIDATE", float("nan")),
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
