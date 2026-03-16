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
    self.old_acc_ctrl_state = 0
    self.old_acc_ctrl_gate = 0
    self.old_tja_active = False
    self.old_acc_base_active = False
    self.old_assist_advanced = False
    self.old_stalk_main_a = 0
    self.old_stalk_main_b = 0
    self.old_acc_button = False
    self.old_tja_button = False
    self.old_speed_adjust = False
    self.legacy_main_button = 0
    self.drive_state_kind_hist = deque(maxlen=3)
    self.drive_state_gear_est = GearShifter.unknown
    self.main_cruise_button = 0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    ws = cp.vl.get("WHEEL_SPEED", {})
    ret.wheelSpeeds = WheelSpeeds(fl=float(ws.get("FL_SPEED_RAW", 0.0)) / 3.6,
                                  fr=float(ws.get("FR_SPEED_RAW", 0.0)) / 3.6,
                                  rl=float(ws.get("RL_SPEED_RAW", 0.0)) / 3.6,
                                  rr=float(ws.get("RR_SPEED_RAW", 0.0)) / 3.6)

    wheel_speeds = [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]
    ret.vEgoRaw = float(sum(wheel_speeds) / 4.0)
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = float(cp.vl.get("EPS_ANGLE", {}).get("STEERING_ANGLE_RAW", 0.0))
    ret.steeringTorque = float(cp.vl.get("STEER_TORQUE", {}).get("DRIVER_STEER_TORQUE_RAW", 0.0))
    ret.steeringPressed = abs(ret.steeringTorque) > 1.0

    ret.yawRate = float(cp.vl.get("DYNAMICS_YAW_PROV", {}).get("YAW_RATE_RAW_A", 0.0))
    ret.brake = float(cp.vl.get("PEDAL_OR_HOLD_STATE_CANDIDATE", {}).get("PEDAL_HOLD_STATE_RAW", 0.0))

    gas_raw = int(cp.vl.get("PTCAN_ACCELERATOR_CANDIDATE", {}).get("ACCELERATOR_I4_COMPAT_PT_CAN", 0))
    ret.gas = min(max(gas_raw / 4000.0, 0.0), 1.0)
    ret.gasPressed = gas_raw > 200

    brake_can_byte1 = int(cp.vl.get("PTCAN_BRAKE_PRESSED_CANDIDATE", {}).get("BRAKE_PRESSED_BYTE_1_PT_CAN", 0xFF))
    ret.brakePressed = brake_can_byte1 < 0x10

    drive_state = cp.vl.get("DRIVE_STATE_EXPERIMENTAL", {})
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

    old_ctrl_state = int(cp.vl.get("ACC_TJA_OLD_ROUTE_HELPER_E", {}).get("ACC_TJA_OLD_CTRL_STATE", 0))
    old_ctrl_gate = int(cp.vl.get("ACC_TJA_OLD_ROUTE_HELPER_D", {}).get("ACC_TJA_OLD_CTRL_GATE", 0))
    old_stalk_main_a = int(cp.vl.get("ACC_TJA_OLD_ROUTE_HELPER_B", {}).get("ACC_TJA_OLD_STALK_MAIN_A", 0))
    old_stalk_main_b = int(cp.vl.get("ACC_TJA_OLD_ROUTE_HELPER_B", {}).get("ACC_TJA_OLD_STALK_MAIN_B", 0))
    self.old_acc_ctrl_state = old_ctrl_state
    self.old_acc_ctrl_gate = old_ctrl_gate
    self.old_acc_base_active = old_ctrl_state == 16610
    self.old_assist_advanced = old_ctrl_state == 24802
    self.old_tja_active = self.old_assist_advanced
    self.old_stalk_main_a = old_stalk_main_a
    self.old_stalk_main_b = old_stalk_main_b
    # Legacy route correlation:
    # 30716/65282 -> ACC button family
    # 18684/65283 -> TJA button family
    # 5884/65282  -> speed stalk +/- family
    self.old_acc_button = old_stalk_main_a == 30716 and old_stalk_main_b == 65282
    self.old_tja_button = old_stalk_main_a == 18684 and old_stalk_main_b == 65283
    self.old_speed_adjust = old_stalk_main_a == 5884 and old_stalk_main_b == 65282

    # Old and modern ACC-only/TJA routes show the clearest stable states here:
    # 35041/643 -> off baseline
    # 16610/3584 -> ACC active base state
    # 24802/(640 or 656) -> advanced assist state / TJA-requested-or-gated branch
    acc_enabled = old_ctrl_state in (16610, 24802)
    acc_available = acc_enabled or old_ctrl_gate in (640, 656, 3584)

    ret.cruiseState.available = acc_available
    ret.cruiseState.enabled = acc_enabled
    ret.cruiseState.standstill = ret.standstill

    blinker_byte6 = int(cp.vl.get("PTCAN_BLINKER_STATE_CANDIDATE", {}).get("BLINKER_STATE_BYTE_6", 0))
    blinker_byte7 = int(cp.vl.get("PTCAN_BLINKER_STATE_CANDIDATE", {}).get("BLINKER_STATE_BYTE_7", 0))
    main_cruise_word = int(cp.vl.get("PTCAN_CRUISE_BUTTONS_MAIN", {}).get("CRUISE_BTN_MAIN_PT_CAN", 0))
    driver_door_state = int(cp.vl.get("PTCAN_DRIVER_DOOR_CANDIDATE", {}).get("DRIVER_DOOR_STATE_BYTE_2", 0))
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
    if self.old_acc_button:
      self.legacy_main_button = 1
    elif self.old_tja_button:
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
    messages = [
      ("WHEEL_SPEED", 100),
      ("STEER_TORQUE", 100),
      ("EPS_ANGLE", 100),
      ("VEHICLE_SPEED_PROV", 100),
      ("DYNAMICS_YAW_PROV", 100),
      ("DRIVE_STATE_EXPERIMENTAL", 50),
      ("PEDAL_OR_HOLD_STATE_CANDIDATE", 50),
      ("BRAKE_BLEND_CANDIDATE_B", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_A", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_B", 50),
      ("COLUMN_SWITCH_CANDIDATE", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_C", 50),
      ("ACC_STALK_TJA_CANDIDATE_B", 50),
      ("ACC_STALK_TJA_CANDIDATE_C", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_D", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_E", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_F", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_G", 50),
      ("ACC_STALK_TJA_CANDIDATE_F", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_H", 50),
      ("ACC_TJA_OLD_ROUTE_HELPER_I", 50),
      ("PTCAN_BLINKER_STATE_CANDIDATE", 50),
      ("PTCAN_DRIVER_DOOR_CANDIDATE", 50),
      ("PTCAN_BRAKE_PRESSED_CANDIDATE", 50),
    ]
    return {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 0)}
