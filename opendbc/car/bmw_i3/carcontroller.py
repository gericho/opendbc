from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.crc import CRC8J1850
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import CarControllerParams


class CarController(CarControllerBase):
  ENABLE_LONG_TX_BUILDER = True
  ENABLE_LATERAL_TX_BUILDER = True
  LONG_59_ACTIVE_PARITY = 0
  LONG_54_ACTIVE_PARITY = 1
  LONG_59_CENTER_WB = 32777
  LONG_59_CENTER_WC = 32767
  LONG_54_CENTER_WB = 65025
  LONG_54_CENTER_WC = 7
  LATERAL_FORCE_WEAKEN_BP = [22.0, 31.0]
  LATERAL_FORCE_WEAKEN_V = [250.0, 250.0]

  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    # Shadow-only builder based on the existing BMW SP2018 ACC/72 method.
    # We do not transmit anything yet; this is only to converge on the
    # logical payload shape before touching real output.
    self.shadow_packer = CANPacker("bmw_sp2018")
    self.VM = VehicleModel(CP)
    self.apply_angle_last = 0.0
    self.shadow_cnt = 0
    self.shadow_cycle = 0
    self.last_shadow_acc_values = None
    self.last_shadow_acc_bytes = b""
    self.last_shadow_long_debug = None
    self.enable_long_tx_builder = bool(self.ENABLE_LONG_TX_BUILDER and CP.openpilotLongitudinalControl)
    self.enable_lateral_tx_builder = bool(self.ENABLE_LATERAL_TX_BUILDER)

  def _next_shadow_cnt(self) -> int:
    self.shadow_cnt = (self.shadow_cnt + 1) % 16
    return self.shadow_cnt

  def _next_shadow_cycle(self) -> int:
    cycle = self.shadow_cycle
    self.shadow_cycle = (self.shadow_cycle + 1) % 64
    return cycle

  def _crc8_j1850(self, data: bytes, init_value: int = 0xF1) -> int:
    crc = init_value & 0xFF
    for byte in data:
      crc ^= byte & 0xFF
      crc = CRC8J1850[crc]
    return crc

  def _shadow_steer_torque_req(self, desired_angle: float, current_angle: float) -> float:
    # Shadow-only heuristic: BMW logs show steer_torque_req as a real field, but
    # we do not have a closed i3-specific mapping yet. Use a conservative
    # proportional term on angle error only for method debugging.
    angle_error = desired_angle - current_angle
    return float(max(-3.0, min(3.0, angle_error * 0.12)))

  def _shadow_torque_reserve(self, driver_torque: float) -> int:
    # Local BMW analysis suggests this reserve drops as driver torque magnitude
    # rises. Keep a narrow, conservative range around the dynm default 0xA0.
    reserve = 0xA0 - int(min(0x30, abs(driver_torque) * 4.0))
    return max(0x70, min(0xA0, reserve))

  def _shadow_long_helper_state(self, CS) -> str:
    # Raw FlexRay helpers 55/56/63/93 are now the best offline discriminants
    # for the longitudinal frame families. Keep the live classifier explicit and
    # conservative until the final TX mapping is enabled.
    if int(getattr(CS, "stock_acc_ctrl_state", 0)) == 24802:
      if int(getattr(CS, "long_54_wb", 0)) or int(getattr(CS, "long_54_wc", 0)):
        return "MANAGED_BRAKE_BLEND"
      if int(getattr(CS, "long_59_wb", 0)) or int(getattr(CS, "long_59_wc", 0)):
        return "MANAGED_POWERTRAIN"
    if int(getattr(CS, "stock_acc_ctrl_state", 0)) == 16610:
      return "ACC_ARMED"
    if int(getattr(CS, "stock_acc_ctrl_gate", 0)) in (640, 656, 3584):
      return "ACC_GATE_ONLY"
    return "OFF"

  @staticmethod
  def _patch_long54_template(template: bytes, phase: int, word_b: int, word_c: int) -> bytes:
    payload = bytearray(template if len(template) == 17 else bytes([0xFF] * 17))
    payload[0] = phase & 0xFF
    payload[3] = word_b & 0xFF
    payload[5] = word_c & 0xFF
    return bytes(payload)

  @staticmethod
  def _patch_long59_template(template: bytes, phase: int, word_b: int, word_c: int) -> bytes:
    payload = bytearray(template if len(template) == 17 else bytes([0xFF] * 17))
    payload[0] = phase & 0xFF
    payload[4] = (word_b >> 8) & 0xFF
    payload[6] = (word_c >> 8) & 0xFF
    return bytes(payload)

  def _build_shadow_long_tx(self, CS, long_tx_hint: dict[str, int | str]) -> dict[str, int | str]:
    tx54_phase = int(getattr(CS, "long_54_phase", 0))
    tx59_phase = int(getattr(CS, "long_59_phase", 0))
    tx54_wb = int(long_tx_hint["tx_target_wb"]) if int(long_tx_hint["tx_branch"]) == 54 else int(getattr(CS, "long_54_wb", 0))
    tx54_wc = int(long_tx_hint["tx_target_wc"]) if int(long_tx_hint["tx_branch"]) == 54 else int(getattr(CS, "long_54_wc", 0))
    tx59_wb = int(long_tx_hint["tx_target_wb"]) if int(long_tx_hint["tx_branch"]) == 59 else int(getattr(CS, "long_59_wb", 0))
    tx59_wc = int(long_tx_hint["tx_target_wc"]) if int(long_tx_hint["tx_branch"]) == 59 else int(getattr(CS, "long_59_wc", 0))
    tx54_template = bytes(getattr(CS, "long_54_stock_template", bytes([0xFF] * 17)))
    tx59_template = bytes(getattr(CS, "long_59_stock_template", bytes([0xFF] * 17)))
    tx54 = self._patch_long54_template(tx54_template, tx54_phase, tx54_wb, tx54_wc)
    tx59 = self._patch_long59_template(tx59_template, tx59_phase, tx59_wb, tx59_wc)
    return {
      "tx54_phase": tx54_phase,
      "tx54_template_hex": tx54_template.hex(),
      "tx54_core_hex": tx54.hex(),
      "tx59_phase": tx59_phase,
      "tx59_template_hex": tx59_template.hex(),
      "tx59_core_hex": tx59.hex(),
    }

  def _build_long_can_msgs(self, CS, long_tx_hint: dict[str, int | str], long_tx_core: dict[str, int | str]) -> list[tuple[int, bytes, int]]:
    if not self.enable_long_tx_builder:
      return []
    branch = int(long_tx_hint["tx_branch"])
    if branch == 54:
      payload = bytes.fromhex(str(long_tx_core["tx54_core_hex"]))
      base = int(long_tx_core["tx54_phase"]) & 0xFF
    else:
      payload = bytes.fromhex(str(long_tx_core["tx59_core_hex"]))
      base = int(long_tx_core["tx59_phase"]) & 0xFF
    # Firmware injector expects:
    #   dat[0] = base/phase
    #   dat[1:1+replace_offset] = padding
    #   dat[1+replace_offset:] = replacement slice
    # For BMW i3 mimic long:
    #   replace_offset = 3
    #   replace_len = 4
    override = bytes([base, 0x00, 0x00, 0x00]) + payload[3:7]
    return [(branch, override, 1)]

  def _shadow_long_tx_hint(self, desired_accel: float, CS) -> dict[str, int | str]:
    # Prefer live stock words when they exist: the raw helper work showed that
    # exact family selection is upstream-state driven, not a simple desired-accel
    # heuristic. Keep desired_accel only as a weak fallback sign hint.
    helper_state = self._shadow_long_helper_state(CS)
    neg_hint = desired_accel < -0.05 or bool(CS.out.brakePressed) or getattr(CS, "stock_long_upstream_mode", "unknown") == "negative"
    live_54_wb = int(getattr(CS, "long_54_wb", 0))
    live_54_wc = int(getattr(CS, "long_54_wc", 0))
    live_59_wb = int(getattr(CS, "long_59_wb", 0))
    live_59_wc = int(getattr(CS, "long_59_wc", 0))

    if (helper_state == "MANAGED_BRAKE_BLEND" or neg_hint) and (live_54_wb or live_54_wc):
      return {
        "tx_mode": "negative",
        "tx_branch": 54,
        "tx_parity": self.LONG_54_ACTIVE_PARITY,
        "tx_target_wb": live_54_wb,
        "tx_target_wc": live_54_wc,
        "tx_source": "stock_live_54",
        "tx_helper_state": helper_state,
      }
    if live_59_wb or live_59_wc:
      return {
        "tx_mode": "positive_or_coast",
        "tx_branch": 59,
        "tx_parity": self.LONG_59_ACTIVE_PARITY,
        "tx_target_wb": live_59_wb,
        "tx_target_wc": live_59_wc,
        "tx_source": "stock_live_59",
        "tx_helper_state": helper_state,
      }
    return {
      "tx_mode": "positive_or_coast",
      "tx_branch": 59,
      "tx_parity": self.LONG_59_ACTIVE_PARITY,
      "tx_target_wb": self.LONG_59_CENTER_WB,
      "tx_target_wc": self.LONG_59_CENTER_WC,
      "tx_source": "center_fallback",
      "tx_helper_state": helper_state,
    }

  @staticmethod
  def _build_lateral_frame72(phase: int, cnt: int, byte8: int = 0xE0) -> bytes:
    payload = bytearray([0] * 17)
    payload[0] = phase & 0xFF
    payload[2] = (0xF << 4) | (cnt & 0xF)
    payload[8] = byte8 & 0xFF
    return bytes(payload)

  @staticmethod
  def _build_lateral_frame96(phase: int, b1: int, b2: int, b3: int, b4: int = 0, b8: int = 0) -> bytes:
    payload = bytearray([0] * 9)
    payload[0] = phase & 0xFF
    payload[1] = b1 & 0xFF
    payload[2] = b2 & 0xFF
    payload[3] = b3 & 0xFF
    payload[4] = b4 & 0xFF
    payload[8] = b8 & 0xFF
    return bytes(payload)

  def _build_shadow_lateral_tx(self, CC, CS):
    phase = int(getattr(CS, "stock_lat96_phase", 0))
    cnt = self.shadow_cnt & 0xF
    dir_hint = str(getattr(CS, "stock_lat_dir_hint", "unknown"))
    mag = float(getattr(CS, "stock_lat_mag_hint", 0.0))
    b1 = int(getattr(CS, "stock_lat96_b1", 0))
    b2 = int(getattr(CS, "stock_lat96_b2", 0))
    b3 = int(getattr(CS, "stock_lat96_b3", 0))
    if CC.latActive and dir_hint != "unknown":
      delta = int(round(24.0 * mag))
      b1 = min(255, b1 + delta) if dir_hint == "right" else max(0, b1 - delta)
    tx72 = self._build_lateral_frame72(phase, cnt)
    tx96 = self._build_lateral_frame96(phase, b1, b2, b3)
    return {
      "lat_phase": phase,
      "lat_dir_hint": dir_hint,
      "lat_mag_hint": mag,
      "lat_tx_enabled": self.enable_lateral_tx_builder,
      "lat_tx_msg_count": 2 if self.enable_lateral_tx_builder and CC.latActive else 0,
      "tx72_hex": tx72.hex(),
      "tx96_hex": tx96.hex(),
    }

  def _build_lateral_can_msgs(self, CC, lateral_tx):
    lat_allowed = bool(CC.latActive or getattr(self, "_stock_acc_lateral_gate", False))
    if not (self.enable_lateral_tx_builder and lat_allowed):
      return []
    tx72 = bytes.fromhex(str(lateral_tx["tx72_hex"]))
    tx96 = bytes.fromhex(str(lateral_tx["tx96_hex"]))
    base72 = int(lateral_tx["lat_phase"]) & 0xFF
    base96 = int(lateral_tx["lat_phase"]) & 0xFF
    # Firmware injector expects the override dat payload to contain only:
    #   dat[0] = base/phase
    #   dat[1:] = replacement bytes (replace_offset = 0)
    return [
      (72, bytes([base72]) + tx72[:9], 0),
      (96, bytes([base96]) + tx96[:9], 0),
    ]

  def _shadow_force_weaken(self, v_ego: float) -> int:
    # Mirror the smnogar BMW lateral method explicitly. Their current i4 fit
    # keeps the weaken field at a stock-like 250 over the validated speed band;
    # keep the interpolation form so future i3-specific tuning can change the
    # breakpoints/values without changing the payload logic again.
    if v_ego <= self.LATERAL_FORCE_WEAKEN_BP[0]:
      return int(self.LATERAL_FORCE_WEAKEN_V[0])
    if v_ego >= self.LATERAL_FORCE_WEAKEN_BP[-1]:
      return int(self.LATERAL_FORCE_WEAKEN_V[-1])
    a = (v_ego - self.LATERAL_FORCE_WEAKEN_BP[0]) / (self.LATERAL_FORCE_WEAKEN_BP[-1] - self.LATERAL_FORCE_WEAKEN_BP[0])
    return int(round(self.LATERAL_FORCE_WEAKEN_V[0] + a * (self.LATERAL_FORCE_WEAKEN_V[-1] - self.LATERAL_FORCE_WEAKEN_V[0])))

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS, now_nanos):
    actuators = CC.actuators
    self._stock_acc_lateral_gate = bool(getattr(CS, "stock_acc_base_armed", False))
    lat_allowed = bool(CC.latActive or self._stock_acc_lateral_gate)

    desired_angle = self.apply_angle_last
    if lat_allowed:
      desired_angle = float(actuators.steeringAngleDeg)
      desired_angle = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                  True, CarControllerParams, self.VM)
      self.apply_angle_last = desired_angle

      cycle_count = self._next_shadow_cycle()
      if cycle_count % 4 == 1:
        cnt1 = self._next_shadow_cnt()
        angle_error = abs(desired_angle - CS.out.steeringAngleDeg)
        driver_override = bool(CS.out.steeringPressed)
        tja_ready = int(CS.out.vEgoRaw > 0.1 and not driver_override and lat_allowed)
        lat_triggered = int(tja_ready and angle_error > 0.5)
        steering_engaged = 2 if tja_ready else 1
        steer_torque_req = self._shadow_steer_torque_req(desired_angle, CS.out.steeringAngleDeg)
        torque_reserve = self._shadow_torque_reserve(CS.out.steeringTorque)
        force_weaken = self._shadow_force_weaken(CS.out.vEgoRaw)
        values = {
          "cycle_count": cycle_count,
          "crc1": 0,
          "cnt1": cnt1,
          "always_0x9": 9,
          "steering_angle_req": desired_angle,
          "steer_torque_req": steer_torque_req,
          "TJA_ready": tja_ready,
          # Match the dynm/smnogar/BMW SP2018 method defaults unless route
          # evidence proves otherwise.
          "assist_mode": 0,
          "wayback_en1_lane_keeping_trigger": lat_triggered,
          "lane_keeping_triggered": lat_triggered,
          "like_assist_torque_reserve": torque_reserve,
          "constants": 0x03ff17fe,
          "wayback_en_2": lat_triggered,
          "steering_engaged": steering_engaged,
          "maybe_assist_force_enhance": 0xA2,
          "maybe_assist_force_weaken": force_weaken,
        }
        msg = self.shadow_packer.make_can_msg("ACC", 4, values)
        payload = bytearray(msg[1])
        payload[1] = self._crc8_j1850(bytes(payload[2:]))
        values["crc1"] = payload[1]
        self.last_shadow_acc_values = values
        self.last_shadow_acc_bytes = bytes(payload)

    lateral_tx = self._build_shadow_lateral_tx(CC, CS)
    lateral_can_msgs = self._build_lateral_can_msgs(CC, lateral_tx)
    desired_accel = float(actuators.accel)
    long_tx_hint = self._shadow_long_tx_hint(desired_accel, CS)
    long_tx_core = self._build_shadow_long_tx(CS, long_tx_hint)
    long_can_msgs = self._build_long_can_msgs(CS, long_tx_hint, long_tx_core)
    self.last_shadow_long_debug = {
      "desired_accel": desired_accel,
      "long_active": bool(CC.longActive),
      "gate": int(getattr(CS, "stock_acc_ctrl_gate", 0)),
      "state": int(getattr(CS, "stock_acc_ctrl_state", 0)),
      "acc_base_armed": bool(getattr(CS, "stock_acc_base_armed", False)),
      "assist_advanced": bool(getattr(CS, "stock_assist_advanced", False)),
      "tja_active": bool(getattr(CS, "stock_tja_active", False)),
      "v_ego": float(CS.out.vEgoRaw),
      "gas_pressed": bool(CS.out.gasPressed),
      "brake_pressed": bool(CS.out.brakePressed),
      "standstill": bool(CS.out.standstill),
      # 59 = best current stock powertrain-intent proxy
      "long_59_wb": int(getattr(CS, "long_59_wb", 0)),
      "long_59_wc": int(getattr(CS, "long_59_wc", 0)),
      "long_59_phase": int(getattr(CS, "long_59_phase", 0)),
      "long_59_b3": int(getattr(CS, "long_59_b3", 0)),
      "long_59_b5": int(getattr(CS, "long_59_b5", 0)),
      "long_59_stock_template": bytes(getattr(CS, "long_59_stock_template", b"")).hex(),
      # 54 = best current stock brake-blend / regen proxy
      "long_54_phase": int(getattr(CS, "long_54_phase", 0)),
      "long_54_wb": int(getattr(CS, "long_54_wb", 0)),
      "long_54_wc": int(getattr(CS, "long_54_wc", 0)),
      "long_54_b4": int(getattr(CS, "long_54_b4", 0)),
      "long_54_b6": int(getattr(CS, "long_54_b6", 0)),
      "long_54_stock_template": bytes(getattr(CS, "long_54_stock_template", b"")).hex(),
      # Upstream PT-CAN long intent candidates:
      "long_up_217_raw16": int(getattr(CS, "long_up_217_raw16", 0)),
      "long_up_217_i4_compat12": int(getattr(CS, "long_up_217_i4_compat12", 0)),
      "long_up_796_raw16": int(getattr(CS, "long_up_796_raw16", 0)),
      "long_up_796_b1": int(getattr(CS, "long_up_796_b1", 0)),
      "stock_long_upstream_mode": str(getattr(CS, "stock_long_upstream_mode", "unknown")),
      "stock_long_upstream_confidence": str(getattr(CS, "stock_long_upstream_confidence", "none")),
      "stock_long_helper_state": self._shadow_long_helper_state(CS),
      "long_helper_46_wa": int(getattr(CS, "long_helper_46_wa", 0)),
      "long_helper_46_wb": int(getattr(CS, "long_helper_46_wb", 0)),
      "long_helper_46_wc": int(getattr(CS, "long_helper_46_wc", 0)),
      "long_helper_46_wd": int(getattr(CS, "long_helper_46_wd", 0)),
      "long_helper_49_wa": int(getattr(CS, "long_helper_49_wa", 0)),
      "long_helper_49_wb": int(getattr(CS, "long_helper_49_wb", 0)),
      "long_helper_49_wc": int(getattr(CS, "long_helper_49_wc", 0)),
      "long_helper_49_wd": int(getattr(CS, "long_helper_49_wd", 0)),
      "long_helper_55_wa": int(getattr(CS, "long_helper_55_wa", 0)),
      "long_helper_55_wb": int(getattr(CS, "long_helper_55_wb", 0)),
      "long_helper_55_wc": int(getattr(CS, "long_helper_55_wc", 0)),
      "long_helper_55_wd": int(getattr(CS, "long_helper_55_wd", 0)),
      "long_helper_56_wa": int(getattr(CS, "long_helper_56_wa", 0)),
      "long_helper_56_wb": int(getattr(CS, "long_helper_56_wb", 0)),
      "long_helper_56_wc": int(getattr(CS, "long_helper_56_wc", 0)),
      "long_helper_56_wd": int(getattr(CS, "long_helper_56_wd", 0)),
      "long_helper_63_wa": int(getattr(CS, "long_helper_63_wa", 0)),
      "long_helper_63_wb": int(getattr(CS, "long_helper_63_wb", 0)),
      "long_helper_63_wc": int(getattr(CS, "long_helper_63_wc", 0)),
      "long_helper_63_wd": int(getattr(CS, "long_helper_63_wd", 0)),
      "long_helper_93_wa": int(getattr(CS, "long_helper_93_wa", 0)),
      "long_helper_93_wb": int(getattr(CS, "long_helper_93_wb", 0)),
      "long_helper_93_wc": int(getattr(CS, "long_helper_93_wc", 0)),
      "long_helper_93_wd": int(getattr(CS, "long_helper_93_wd", 0)),
      "long_tx_builder_enabled": self.enable_long_tx_builder,
      "long_tx_builder_msg_count": len(long_can_msgs),
      "long_tx_override_hex": long_can_msgs[0][1].hex() if long_can_msgs else "",
      **long_tx_core,
      **long_tx_hint,
      **lateral_tx,
      "lat72_override_hex": lateral_can_msgs[0][1].hex() if lateral_can_msgs else "",
      "lat96_override_hex": lateral_can_msgs[1][1].hex() if len(lateral_can_msgs) > 1 else "",
      "desired_angle": float(desired_angle),
      "lat_allowed": lat_allowed,
      "stock_acc_lateral_gate": bool(self._stock_acc_lateral_gate),
      "stock_lat_active_hint": bool(getattr(CS, "stock_lat_active_hint", False)),
      "stock_lat96_b1": int(getattr(CS, "stock_lat96_b1", 0)),
      "stock_lat96_b2": int(getattr(CS, "stock_lat96_b2", 0)),
      "stock_lat96_b3": int(getattr(CS, "stock_lat96_b3", 0)),
      "stock_lat112_b5": int(getattr(CS, "stock_lat112_b5", 0)),
      "stock_lat116_b5": int(getattr(CS, "stock_lat116_b5", 0)),
    }
    self.frame += 1
    return actuators.as_builder(), long_can_msgs + lateral_can_msgs
