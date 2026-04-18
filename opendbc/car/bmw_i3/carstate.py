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
  USE_PTCAN_INPUTS = False
  STEER_OVERRIDE_APPLY_NM = 2.2
  STEER_OVERRIDE_RELEASE_NM = 2.0
  _ASSIST_MODE_OFF = 0
  _ASSIST_MODE_ACC = 1
  _ASSIST_MODE_TJA = 2

  # Minimal route-backed lateral view of frame 96:
  #   selector  -> 96.byte0
  #   payload   -> low nibble of 96.byte2
  #   context   -> 96.byte3
  # Direction/magnitude remain phase-local but no longer depend on byte1.
  _LAT_B2_PHASE_MAP = {
    0x00: {"mode": "delta",  "gain": 0.2924724805838325, "center": 252.0, "confidence": "medium"},
    0x04: {"mode": "future", "gain": 2.5092792257102507, "center": 247.0, "confidence": "high"},
    0x08: {"mode": "delta",  "gain": -0.61435546875, "center": 249.0, "confidence": "medium"},
    0x0C: {"mode": "future", "gain": 2.478782854867928, "center": 248.5, "confidence": "medium"},
    0x10: {"mode": "future", "gain": 1.327924057710358, "center": 248.0, "confidence": "medium"},
    0x14: {"mode": "delta",  "gain": -0.8819670052290186, "center": 245.0, "confidence": "low"},
    0x18: {"mode": "future", "gain": 2.1200545980105185, "center": 250.0, "confidence": "high"},
    0x1C: {"mode": "future", "gain": 2.3656642313093204, "center": 244.0, "confidence": "high"},
    0x20: {"mode": "future", "gain": 1.66518026660722, "center": 246.0, "confidence": "medium"},
    0x24: {"mode": "future", "gain": 0.8935605457190904, "center": 252.0, "confidence": "medium"},
    0x28: {"mode": "future", "gain": 3.6022312608489697, "center": 248.0, "confidence": "medium"},
    0x2C: {"mode": "future", "gain": 0.9222890638394596, "center": 247.5, "confidence": "low"},
    0x30: {"mode": "delta",  "gain": -0.9152262882258118, "center": 249.5, "confidence": "medium"},
    0x34: {"mode": "future", "gain": 1.654658586103641, "center": 249.0, "confidence": "high"},
    0x38: {"mode": "future", "gain": 3.1955820742568717, "center": 242.5, "confidence": "high"},
    0x3C: {"mode": "future", "gain": 2.4277548422827278, "center": 251.0, "confidence": "high"},
  }

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.shifter_values = CANDefine(DBC[CP.carFingerprint][Bus.pt]).dv.get("DRIVE_STATE", {})
    # These helpers originally came from historical route correlation work, but
    # 131/135/97 remain the current primary stock ACC/TJA state/button families.
    self.stock_acc_ctrl_state = 0
    self.stock_acc_ctrl_gate = 0
    self.stock_tja_active = False
    self.stock_acc_base_armed = False
    self.stock_assist_advanced = False
    self.stock_tja_active_cnt = 0
    self.stock_long_target_u_83 = 0
    self.stock_long_target_speed_est_kph_83 = 0.0
    self.eps_angle_51_raw = 0
    self.stock_stalk_main_a = 0
    self.stock_stalk_main_b = 0
    self.stock_acc_button = False
    self.stock_tja_button = False
    self.stock_speed_adjust = False
    self.stock_acc_button_cnt = 0
    self.stock_tja_button_cnt = 0
    self.legacy_button_seen = False
    self.legacy_button_raw = False
    self.legacy_button_toggle_on = False
    self.legacy_button_cooldown = 0
    self.stock_assist_mode_hist = deque(maxlen=4)
    self.stock_assist_mode_stable = self._ASSIST_MODE_OFF
    self.drive_state_kind_hist = deque(maxlen=3)
    self.drive_state_gear_est = GearShifter.unknown
    self.main_cruise_button = 0
    self.long_up_217_raw16 = 0
    self.long_up_217_word23 = 0
    self.long_up_217_value = 0
    self.long_up_217_i4_compat12 = 0
    self.long_up_796_raw16 = 0
    self.long_up_796_b1 = 0
    self.brake_239_word23 = 32000
    self.brake_239_word56 = 32000
    self.stock_long_upstream_mode = "unknown"
    self.stock_long_upstream_confidence = "none"
    self.long_helper_49_wa = 0
    self.long_helper_49_wb = 0
    self.long_helper_49_wc = 0
    self.long_helper_49_wd = 0
    self.steering_angle_proxy_56_raw = 0
    self.eps_angle_proxy_44_raw = 0
    self.long_helper_55_wa = 0
    self.long_helper_55_wb = 0
    self.long_helper_55_wc = 0
    self.long_helper_55_wd = 0
    self.long_helper_56_wa = 0
    self.long_helper_56_wb = 0
    self.long_helper_56_wc = 0
    self.long_helper_56_wd = 0
    self.long_helper_63_wa = 0
    self.long_helper_63_wb = 0
    self.long_helper_63_wc = 0
    self.long_helper_63_wd = 0
    self.long_helper_93_wa = 0
    self.long_helper_93_wb = 0
    self.long_helper_93_wc = 0
    self.long_helper_93_wd = 0
    self.driver_steer_torque = 0.0
    self.driver_steer_pressed = False
    self.vehicle_speed_kph = 0.0
    self.stock_lat60_phase = 0
    self.stock_lat60_cmd_phase = 0
    self.stock_lat60_subframe = 0
    self.stock_lat72_cycle_count = 0
    self.stock_lat96_phase = 0
    self.stock_lat96_cycle_count = 0
    self.stock_lat96_b1 = 0
    self.stock_lat96_b2 = 0
    self.stock_lat96_b3 = 0
    self.stock_lat96_b4 = 0
    self.stock_lat96_b8 = 0
    self.stock_lat96_template = bytes([0xFF] * 9)
    self.stock_lat15_cycle_count = 0
    self.stock_lat15_raw = 0
    self.stock_lat15_deg_draft = 0.0
    self.stock_lat15_real_branch = False
    self.stock_lat112_b5 = 0
    self.stock_lat116_b5 = 0
    self.stock_lat_active_hint = False
    self.stock_lat_dir_hint = "unknown"
    self.stock_lat_dir_confidence = "none"
    self.stock_lat_mag_hint = 0.0
    self.stock_lat_mag_confidence = "none"
    self.ptcan_steering_raw = 0
    self.ptcan_steering_companion_raw = 0

  @staticmethod
  def _stock_lat_dir_from_phase_b2(phase: int, b2: int) -> tuple[str, str]:
    row = CarState._LAT_B2_PHASE_MAP.get(int(phase))
    if row is None:
      return ("unknown", "none")
    center = float(row["center"])
    b2f = float(b2)
    if abs(b2f - center) < 0.75:
      return ("center", str(row["confidence"]))
    left_is_high = float(row["gain"]) > 0.0
    if b2f > center:
      return (("left" if left_is_high else "right"), str(row["confidence"]))
    return (("right" if left_is_high else "left"), str(row["confidence"]))

  @staticmethod
  def _stock_lat_mag_from_phase_b2(phase: int, b2: int) -> tuple[float, str]:
    row = CarState._LAT_B2_PHASE_MAP.get(int(phase))
    if row is None:
      return (0.0, "none")
    center = float(row["center"])
    span = 7.0
    if span <= 1e-6:
      return (0.0, "none")
    return (min(1.0, abs(float(b2) - center) / span), str(row["confidence"]))

  @staticmethod
  def _stock_lat_support_from_b2(phase: int, b2: int) -> tuple[float, str]:
    row = CarState._LAT_B2_PHASE_MAP.get(int(phase))
    if row is None:
      return (0.0, "none")
    center = float(row["center"])
    span = 7.0
    if span <= 1e-6:
      return (0.0, "none")
    return (min(1.0, abs(float(b2) - center) / span), "low")

  @staticmethod
  def _lat_phase_entry(phase: int) -> dict | None:
    return CarState._LAT_B2_PHASE_MAP.get(int(phase))

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

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # FlexRay-only i3 uses stock ACC/TJA state transitions to synthesize a
        # single button-based enable event. We keep the standard OP behavior:
        # enable on a synthetic cruise-button release, disable on cancel.
        if b.type == ButtonType.decelCruise and not b.pressed:
          return True
    return False

  @classmethod
  def _stock_assist_mode(cls, acc_enabled: bool, tja_enabled: bool) -> int:
    if tja_enabled:
      return cls._ASSIST_MODE_TJA
    if acc_enabled:
      return cls._ASSIST_MODE_ACC
    return cls._ASSIST_MODE_OFF

  @classmethod
  def _stock_button_mode_candidate(cls, gate: int, state: int) -> int | None:
    # Route 2026-04-03 (`...bbbc`) gives the cleanest stable stock modes:
    #   OFF      -> gate 643 / state 35041
    #   ACC      -> gate 3584 / state 16610
    #   ACC+TJA  -> gate 640 / state 24802
    # Intermediate states like 26850, 18658, 225, or gate 656 / 16610 are
    # transitions/internal context and should not directly trigger OP buttons.
    if gate == 643 and state == 35041:
      return cls._ASSIST_MODE_OFF
    if gate == 3584 and state == 16610:
      return cls._ASSIST_MODE_ACC
    if gate == 640 and state == 24802:
      return cls._ASSIST_MODE_TJA
    return None

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_state = can_parsers[Bus.pt]   # src0 / SAS-side
    cp_aux = can_parsers[Bus.cam]    # src1 / vehicle-side companion
    cp_can = can_parsers[Bus.party]
    use_ptcan = self.USE_PTCAN_INPUTS
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Match the dynm/SP2018 BMW method semantically: consume vehicle speed only
    # from the valid m3 subframe of frame 55. Frame 46 was removed from the
    # runtime path because it does not track vehicle speed closely enough on the i3.
    vehicle_speed = cp_aux.vl.get("VEHICLE_SPEED_PROV", {})
    self.long_helper_55_wa = int(vehicle_speed.get("VEHICLE_SPEED_RAW_WORD_A", 0))
    self.long_helper_55_wb = int(vehicle_speed.get("VEHICLE_SPEED_RAW_WORD_B", 0))
    self.long_helper_55_wc = int(vehicle_speed.get("VEHICLE_SPEED_RAW_WORD_C", 0))
    self.long_helper_55_wd = int(vehicle_speed.get("VEHICLE_SPEED_RAW_WORD_D", 0))
    vehicle_speed_cycle = int(vehicle_speed.get("VEHICLE_SPEED_CYCLE_RAW", -1))
    if vehicle_speed_cycle == 3:
      self.vehicle_speed_kph = float(vehicle_speed.get("VEHICLE_SPEED_BMW", self.vehicle_speed_kph))
    ret.vEgoRaw = self.vehicle_speed_kph * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgoRaw
    ret.standstill = ret.vEgoRaw < 0.1

    if use_ptcan:
      # PT-CAN 770 is the best current live steering-wheel-angle source on the i3.
      pt_steering = cp_can.vl.get("PTCAN_STEERING_WHEEL_CANDIDATE", {})
      self.ptcan_steering_raw = int(pt_steering.get("PTCAN_STEERING_WHEEL_RAW", 0))
      pt_steering_deg = float(pt_steering.get("PTCAN_STEERING_WHEEL_ANGLE_I3_CAL", 0.0))
      pt_steering_companion = cp_can.vl.get("PTCAN_STEERING_WHEEL_COMPANION_CANDIDATE", {})
      self.ptcan_steering_companion_raw = int(pt_steering_companion.get("PTCAN_STEERING_WHEEL_COMPANION_RAW", 0))
      ret.steeringAngleDeg = pt_steering_deg
    else:
      self.ptcan_steering_raw = 0
      self.ptcan_steering_companion_raw = 0
      eps_angle = cp_aux.vl.get("EPS_ANGLE", {})
      eps_angle_cycle = int(eps_angle.get("EPS_ANGLE_CYCLE_RAW", -1))
      if eps_angle_cycle == 0:
        self.eps_angle_51_raw = int(eps_angle.get("EPS_ANGLE_RAW_WORD_B", self.eps_angle_51_raw))
        ret.steeringAngleDeg = float(eps_angle.get("EPS_STEERING_ANGLE_BMW", self.out.steeringAngleDeg))
      else:
        ret.steeringAngleDeg = self.out.steeringAngleDeg
    steer_torque = cp_aux.vl.get("STEER_TORQUE", {})
    self.long_helper_49_wa = int(steer_torque.get("STEER_TORQUE_RAW_WORD_A", 0))
    self.long_helper_49_wb = int(steer_torque.get("STEER_TORQUE_RAW_WORD_B", 0))
    self.long_helper_49_wc = int(steer_torque.get("STEER_TORQUE_RAW_WORD_C", 0))
    self.long_helper_49_wd = int(steer_torque.get("STEER_TORQUE_RAW_WORD_D", 0))
    steer_torque_cycle = int(steer_torque.get("STEER_TORQUE_CYCLE_RAW", -1))
    if steer_torque_cycle == 0:
      self.driver_steer_torque = float(steer_torque.get("DRIVER_STEER_TORQUE_BMW", self.driver_steer_torque))
    ret.steeringTorque = self.driver_steer_torque
    torque_abs = abs(ret.steeringTorque)
    if self.driver_steer_pressed:
      self.driver_steer_pressed = torque_abs > self.STEER_OVERRIDE_RELEASE_NM
    else:
      self.driver_steer_pressed = torque_abs > self.STEER_OVERRIDE_APPLY_NM
    ret.steeringPressed = self.driver_steer_pressed

    dynamics_yaw = cp_aux.vl.get("DYNAMICS_YAW_PROV", {})
    self.long_helper_56_wa = int(dynamics_yaw.get("DYNAMICS_YAW_RAW_WORD_A", 0))
    self.long_helper_56_wb = int(dynamics_yaw.get("DYNAMICS_YAW_RAW_WORD_B", 0))
    self.long_helper_56_wc = int(dynamics_yaw.get("DYNAMICS_YAW_RAW_WORD_C", 0))
    self.long_helper_56_wd = int(dynamics_yaw.get("DYNAMICS_YAW_RAW_WORD_D", 0))

    # No physical brake-pressure value is closed yet.
    ret.brake = 0.0

    if use_ptcan:
      pt_accel = cp_can.vl.get("PTCAN_ACCELERATOR_CANDIDATE", {})
      self.long_up_217_raw16 = int(pt_accel.get("ACCEL_RAW_PT_CAN", 0))
      self.long_up_217_word23 = int(pt_accel.get("ACCELERATOR_WORD23_PT_CAN", 0))
      self.long_up_217_value = max(0, min(4000, int(pt_accel.get("ACCELERATOR_VALUE_PT_CAN", 0))))
      self.long_up_217_i4_compat12 = int(pt_accel.get("ACCELERATOR_I4_COMPAT_PT_CAN", 0))

      pt_brake_aux = cp_can.vl.get("PTCAN_CRUISE_BUTTONS_AUX", {})
      self.brake_239_word23 = int(pt_brake_aux.get("BRAKE_PEDAL_WORD23_PT_CAN", 32000))
      self.brake_239_word56 = int(pt_brake_aux.get("BRAKE_PEDAL_WORD56_PT_CAN", 32000))

      pt_brake = cp_can.vl.get("PTCAN_BRAKE_PRESSED_CANDIDATE", {})
      self.long_up_796_raw16 = int(pt_brake.get("BRAKE_PRESSED_RAW_PT_CAN", 0))
      self.long_up_796_b1 = int(pt_brake.get("BRAKE_PRESSED_BYTE_1_PT_CAN", 0xFF))
    else:
      self.long_up_217_raw16 = 0
      self.long_up_217_word23 = 0
      self.long_up_217_value = 0
      self.long_up_217_i4_compat12 = 0
      self.brake_239_word23 = 32000
      self.brake_239_word56 = 32000
      self.long_up_796_raw16 = 0
      self.long_up_796_b1 = 0xFF
    self.stock_long_upstream_mode, self.stock_long_upstream_confidence = self._stock_long_upstream_hint(
      self.long_up_217_raw16, self.long_up_796_b1
    )
    long63 = cp_aux.vl.get("LONG_STATE_HELPER_D", {})
    self.long_helper_63_wa = int(long63.get("LONG_STATE_HELPER_D_WORD_A", 0))
    self.long_helper_63_wb = int(long63.get("LONG_STATE_HELPER_D_WORD_B", 0))
    self.long_helper_63_wc = int(long63.get("LONG_STATE_HELPER_D_WORD_C", 0))
    self.long_helper_63_wd = int(long63.get("LONG_STATE_HELPER_D_WORD_D", 0))

    long93 = cp_aux.vl.get("ACC_TJA_OLD_ROUTE_HELPER_A", {})
    self.long_helper_93_wa = int(long93.get("LONG_STATE_HELPER_A_WORD_A", 0))
    self.long_helper_93_wb = int(long93.get("LONG_STATE_HELPER_A_WORD_B", 0))
    self.long_helper_93_wc = int(long93.get("LONG_STATE_HELPER_A_WORD_C", 0))
    self.long_helper_93_wd = int(long93.get("LONG_STATE_HELPER_A_WORD_D", 0))

    lat60 = cp_state.vl.get("LAT_STOCK_TX_TRIGGER_CANDIDATE", {})
    self.stock_lat60_phase = int(lat60.get("LAT_STOCK_TRIGGER_PHASE_BYTE_0", 0))
    self.stock_lat60_cmd_phase = int(lat60.get("LAT_STOCK_TRIGGER_CMD_PHASE", 0))
    self.stock_lat60_subframe = int(lat60.get("LAT_STOCK_TRIGGER_SUBFRAME_LSB", 0))

    lat72 = cp_state.vl.get("LAT_STOCK_FRAME_72_CANDIDATE", {})
    self.stock_lat72_cycle_count = int(lat72.get("LAT_STOCK_72_CYCLE_COUNT", 0))

    lat131 = cp_state.vl.get("LAT_TJA_COMMAND_CANDIDATE", {})
    self.stock_long_target_u_83 = int(lat131.get("LONG_TARGET_U_83", 0))
    self.stock_long_target_speed_est_kph_83 = float(lat131.get("LONG_TARGET_SPEED_EST_83", 0.0))

    lat96 = cp_state.vl.get("LAT_STOCK_TX_PAYLOAD_CANDIDATE", {})
    self.stock_lat96_cycle_count = int(lat96.get("LAT_STOCK_TX_CYCLE_COUNT", lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_0", 0)))
    self.stock_lat96_phase = self.stock_lat96_cycle_count
    self.stock_lat96_b1 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_1", 0))
    self.stock_lat96_b2 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_2", 0))
    self.stock_lat96_b3 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_3", 0))
    self.stock_lat96_b4 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_4", 0xFF))
    self.stock_lat96_b8 = int(lat96.get("LAT_STOCK_TX_PAYLOAD_BYTE_8", 0xFF))
    self.stock_lat96_template = bytes((
      self.stock_lat96_phase & 0xFF,
      self.stock_lat96_b1 & 0xFF,
      self.stock_lat96_b2 & 0xFF,
      self.stock_lat96_b3 & 0xFF,
      self.stock_lat96_b4 & 0xFF,
      0xFF, 0xFF, 0xFF,
      self.stock_lat96_b8 & 0xFF,
    ))
    lat44 = cp_aux.vl.get("LAT_STOCK_TORQUE_CONTROL_44", {})
    self.stock_lat44_torque_raw = int(lat44.get("LAT_STOCK_TORQUE_44_RAW", 0))
    self.stock_lat44_torque_ican_hack_nm = float(lat44.get("LAT_STOCK_TORQUE_44_ICAN_HACK_NM", 0.0))

    lat15 = cp_aux.vl.get("LAT_STOCK_ANGLE_REQUEST_15", {})
    self.stock_lat15_cycle_count = int(lat15.get("LAT_STOCK_ANGLE_15_CYCLE_RAW", 0))
    lat15_branch_marker = int(lat15.get("LAT_STOCK_ANGLE_15_BRANCH_MARKER", 0))
    self.stock_lat15_real_branch = (self.stock_lat15_cycle_count & 0x3) in (0, 2) and lat15_branch_marker == 0xFFFFFFFFFFFFFFFF
    if self.stock_lat15_real_branch:
      self.stock_lat15_raw = int(lat15.get("LAT_STOCK_ANGLE_15_RAW", self.stock_lat15_raw))
      self.stock_lat15_deg_draft = float(lat15.get("LAT_STOCK_ANGLE_15_DEG_DRAFT", self.stock_lat15_deg_draft))

    lat112 = cp_aux.vl.get("ACC_STALK_TJA_CANDIDATE_B", {})
    lat116 = cp_aux.vl.get("ACC_STALK_TJA_CANDIDATE_C", {})
    self.stock_lat112_b5 = int(lat112.get("LAT_STOCK_MAIN_BYTE_5", 0))
    self.stock_lat116_b5 = int(lat116.get("LAT_STOCK_SUPPORT_BYTE_5", 0))
    # Best current live discriminator from route work:
    #   112.byte5 bit5 set   -> manual/off tendency
    #   112.byte5 bit5 clear -> assisted/TJA tendency
    self.stock_lat_active_hint = (self.stock_lat112_b5 & 0x20) == 0

    phase_for_decode = self.stock_lat96_phase
    self.stock_lat_dir_hint, self.stock_lat_dir_confidence = self._stock_lat_dir_from_phase_b2(phase_for_decode, self.stock_lat96_b2)
    self.stock_lat_mag_hint, self.stock_lat_mag_confidence = self._stock_lat_mag_from_phase_b2(phase_for_decode, self.stock_lat96_b2)
    if not self.stock_lat_active_hint:
      self.stock_lat_dir_hint = "unknown"
      self.stock_lat_dir_confidence = "none"
      self.stock_lat_mag_hint = 0.0
      self.stock_lat_mag_confidence = "none"

    ret.gasPressed = self.long_up_217_value > 100

    brake_delta = max(0.0, 32000.0 - float(self.brake_239_word56))
    ret.brake = min(1.0, brake_delta / 2060.0)
    ret.brakePressed = brake_delta > 10.0

    drive_state = cp_aux.vl.get("DRIVE_STATE", {})
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
    stock_stalk_main_a = int(cp_aux.vl.get("ACC_TJA_OLD_ROUTE_HELPER_B", {}).get("ACC_TJA_OLD_STALK_MAIN_A", 0))
    stock_stalk_main_b = int(cp_aux.vl.get("ACC_TJA_OLD_ROUTE_HELPER_B", {}).get("ACC_TJA_OLD_STALK_MAIN_B", 0))
    self.stock_acc_ctrl_state = stock_ctrl_state
    self.stock_acc_ctrl_gate = stock_ctrl_gate
    self.stock_acc_base_armed = stock_ctrl_state == 16610
    self.stock_assist_advanced = stock_ctrl_state == 24802
    # Keep TJA detection external to the DBC and conservative:
    # only accept the explicit managed/helper branch observed in route-backed
    # reverse work. Frame-96 heuristics are too permissive because the same
    # family remains alive outside true TJA-managed windows.
    stock_tja_candidate = stock_ctrl_state == 24802 and stock_ctrl_gate in (640, 656)
    if stock_tja_candidate:
      self.stock_tja_active_cnt = min(self.stock_tja_active_cnt + 1, 4)
    else:
      self.stock_tja_active_cnt = max(self.stock_tja_active_cnt - 1, 0)
    self.stock_tja_active = self.stock_tja_active_cnt >= 2
    self.stock_stalk_main_a = stock_stalk_main_a
    self.stock_stalk_main_b = stock_stalk_main_b
    # Legacy route correlation:
    # 30716/65282 -> ACC button family
    # 18684/65283 -> TJA button family
    # 5884/65282  -> speed stalk +/- family
    raw_stock_acc_button = stock_stalk_main_a == 30716 and stock_stalk_main_b == 65282
    raw_stock_tja_button = stock_stalk_main_a == 18684 and stock_stalk_main_b == 65283
    raw_stock_legacy_button = stock_stalk_main_b in (65282, 65283)
    # The stalk helper families are observed as short periodic pulses on FlexRay.
    # Stretch them slightly so UI/engagement logic sees one stable press/release
    # instead of repeated chattering edges.
    self.stock_acc_button_cnt = 4 if raw_stock_acc_button else max(self.stock_acc_button_cnt - 1, 0)
    self.stock_tja_button_cnt = 4 if raw_stock_tja_button else max(self.stock_tja_button_cnt - 1, 0)
    self.stock_acc_button = self.stock_acc_button_cnt > 0
    self.stock_tja_button = self.stock_tja_button_cnt > 0
    self.stock_speed_adjust = stock_stalk_main_a == 5884 and stock_stalk_main_b == 65282

    # Old and modern ACC-only/TJA routes show the clearest stable states here:
    # 35041/643 -> off baseline
    # 16610/3584 -> ACC base armed/ready state
    # 24802/(640 or 656) -> advanced assist state / actual managed-control branch
    acc_enabled = stock_ctrl_state in (16610, 24802)
    # Keep availability conservative on the flexray-only port:
    # only show engageable when the car is actually in drive and the stock
    # ACC/TJA stack is armed, or while a real stock assist button edge is seen.
    acc_available = (
      ret.gearShifter == structs.CarState.GearShifter.drive and
      (acc_enabled or self.stock_acc_button or self.stock_tja_button)
    )

    ret.cruiseState.available = acc_available
    # This flexray-only i3 port uses button-based engagement (pcmCruise=False).
    # If we mirror stock ACC-active into cruiseState.enabled, selfdrived will
    # raise cruiseMismatch forever because it expects enabled to track OP state
    # only on pcmCruise cars. Keep stock ACC/TJA state in the internal helpers
    # above and expose only availability here.
    ret.cruiseState.enabled = False
    ret.cruiseState.standstill = ret.standstill

    if use_ptcan:
      blinker_byte6 = int(cp_can.vl.get("PTCAN_BLINKER_STATE_CANDIDATE", {}).get("BLINKER_STATE_BYTE_6", 0))
      turn_left_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_LEFT_TURN_CANDIDATE", 0))
      turn_right_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_RIGHT_TURN_CANDIDATE", 0))
      turn_active_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_TURNSIGNAL_ACTIVE_CANDIDATE", 0))
      turn_idle_candidate = int(cp_can.vl.get("PTCAN_TURNSIGNALS_CANDIDATE", {}).get("PTCAN_TURNSIGNAL_IDLE_CANDIDATE", 0))
      main_cruise_word = int(cp_can.vl.get("PTCAN_CRUISE_BUTTONS_MAIN", {}).get("CRUISE_BTN_MAIN_PT_CAN", 0))
      driver_door_state = int(cp_can.vl.get("PTCAN_DRIVER_DOOR_CANDIDATE", {}).get("DRIVER_DOOR_STATE_BYTE_2", 0))
      seatbelt_a = int(cp_can.vl.get("PTCAN_SEATBELT_CANDIDATE_A", {}).get("PTCAN_SEATBELT_BYTE_4_A", 0))
      seatbelt_b = int(cp_can.vl.get("PTCAN_SEATBELT_CANDIDATE_B", {}).get("PTCAN_SEATBELT_BYTE_2_B", 0))
    else:
      blinker_byte6 = 0
      turn_left_candidate = 0
      turn_right_candidate = 0
      turn_active_candidate = 0
      turn_idle_candidate = 0
      main_cruise_word = 0
      driver_door_state = 0
      seatbelt_a = 0
      seatbelt_b = 0
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
    # Broad button-route scans show only two 415-word values with enough purity to
    # be worth mapping today:
    #   0x8015 -> SET family
    #   0x8016 -> RES family
    # The rest (+/-/distance) still overlap too much for a safe public mapping.
    self.main_cruise_button = {
      0x8015: 1,
      0x8016: 2,
    }.get(main_cruise_word, 0)
    # Latest live reverse work shows the wheel-command path is not a clean
    # single-frame pulse. Treat it as an assist-state machine instead:
    #   OFF < ACC < TJA
    # and emit OP button events only when the stock state crosses one of those
    # stable boundaries. This removes the false toggles caused by the old
    # 97/112/116 pulse decoder while still tracking the driver's real intent.
    legacy_events: list[structs.CarState.ButtonEvent] = []
    if raw_stock_legacy_button:
      self.legacy_button_seen = True
    elif self.legacy_button_seen:
      legacy_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.decelCruise))
      self.legacy_button_seen = False
    else:
      button_mode_candidate = self._stock_button_mode_candidate(stock_ctrl_gate, stock_ctrl_state)
      if button_mode_candidate is not None:
        self.stock_assist_mode_hist.append(button_mode_candidate)
      if len(self.stock_assist_mode_hist) == self.stock_assist_mode_hist.maxlen:
        mode_counts = Counter(self.stock_assist_mode_hist)
        stable_mode, stable_votes = mode_counts.most_common(1)[0]
        if stable_votes == self.stock_assist_mode_hist.maxlen and stable_mode != self.stock_assist_mode_stable:
          # Do not synthesize enable events from stock ACC/TJA state transitions:
          # that can auto-enable OP as soon as the OEM system arms. Keep only the
          # disable-side cancel when the stock system drops out.
          if stable_mode == self._ASSIST_MODE_OFF and self.stock_assist_mode_stable in (self._ASSIST_MODE_ACC, self._ASSIST_MODE_TJA):
            legacy_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
          elif self.stock_assist_mode_stable == self._ASSIST_MODE_TJA and stable_mode == self._ASSIST_MODE_ACC:
            legacy_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
          self.stock_assist_mode_stable = stable_mode

    ret.buttonEvents = [
      *create_button_events(self.main_cruise_button, prev_main_cruise_button, {
        1: ButtonType.setCruise,
        2: ButtonType.resumeCruise,
      }),
      *legacy_events,
    ]
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    dbc = DBC[CP.carFingerprint][Bus.pt]
    pt_messages = [
      ("LAT_TJA_COMMAND_CANDIDATE", float("nan")),
      ("LAT_STOCK_TX_TRIGGER_CANDIDATE", float("nan")),
      ("LAT_STOCK_FRAME_72_CANDIDATE", float("nan")),
      ("LAT_STOCK_TX_PAYLOAD_CANDIDATE", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_D", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_E", float("nan")),
    ]
    cam_messages = [
      ("VEHICLE_SPEED_PROV", float("nan")),
      ("STEER_TORQUE", float("nan")),
      ("EPS_ANGLE", float("nan")),
      ("DYNAMICS_YAW_PROV", float("nan")),
      ("LONG_STATE_HELPER_D", float("nan")),
      ("ACC_STALK_TJA_CANDIDATE_B", float("nan")),
      ("ACC_STALK_TJA_CANDIDATE_C", float("nan")),
      ("DRIVE_STATE", float("nan")),
      ("LAT_STOCK_TORQUE_CONTROL_44", float("nan")),
      ("LAT_STOCK_ANGLE_REQUEST_15", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_A", float("nan")),
      ("ACC_TJA_OLD_ROUTE_HELPER_B", float("nan")),
    ]
    party_messages = [] if not CarState.USE_PTCAN_INPUTS else [
      ("PTCAN_ACCELERATOR_CANDIDATE", float("nan")),
      ("PTCAN_STEERING_WHEEL_COMPANION_CANDIDATE", float("nan")),
      ("PTCAN_STEERING_WHEEL_CANDIDATE", float("nan")),
      ("PTCAN_CRUISE_BUTTONS_AUX", float("nan")),
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
