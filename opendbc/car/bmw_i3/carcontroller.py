from opendbc.car import structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import CarControllerParams


class CarController(CarControllerBase):
  ENABLE_LONG_TX_BUILDER = True
  ENABLE_LATERAL_TX_BUILDER = True
  LAT72_ANGLE_FULL_SCALE_DEG = 30.0
  LAT72_MATCH_DELTA_BP = [0.0, 3.0, 8.0, 15.0]
  LAT72_MATCH_DELTA_V = [3.0, 4.0, 6.0, 7.0]
  LAT72_SAFE_ERROR_DEG = 12.0
  LAT72_TARGET_DEADBAND_DEG = 1.5
  LAT72_TARGET_CMD_MIN_DEG = 2.5
  LAT72_OFFSET_SPAN = 4
  LAT72_POS_NIBBLE_BY_PHASE = {
    0: 4, 1: 5, 2: 6, 3: 7, 4: 8, 5: 9, 6: 10, 7: 11,
    8: 12, 9: 13, 10: 14, 11: 0, 12: 1, 13: 2, 14: 3, 15: 4,
    16: 3, 17: 4, 18: 7, 19: 6, 20: 7, 21: 8, 22: 9, 23: 10,
    24: 11, 25: 12, 26: 13, 27: 14, 28: 0, 29: 1, 30: 2, 31: 3,
  }
  LAT72_NEG_NIBBLE_BY_PHASE = {
    0: 5, 1: 6, 2: 7, 3: 8, 4: 9, 5: 10, 6: 11, 7: 12,
    8: 13, 9: 14, 10: 0, 11: 14, 12: 0, 13: 1, 14: 2, 15: 3,
    16: 4, 17: 5, 18: 6, 19: 7, 20: 8, 21: 9, 22: 10, 23: 11,
    24: 12, 25: 13, 26: 14, 27: 0, 28: 1, 29: 2, 30: 3, 31: 4,
  }
  LONG_59_ACTIVE_PARITY = 0
  LONG_54_ACTIVE_PARITY = 1
  LONG_59_CENTER_WB = 32777
  LONG_59_CENTER_WC = 32767
  LONG_54_CENTER_WB = 65025
  LONG_54_CENTER_WC = 7
  LATERAL_FORCE_WEAKEN_BP = [22.0, 31.0]
  LATERAL_FORCE_WEAKEN_V = [250.0, 250.0]
  LONG_PAIR_TABLE = {
    ("ACC_ARMED_POSITIVE_PULL", "positive"): {
      "families": [
        (bytes.fromhex("3a00000000000000000000000000000000"), bytes.fromhex("3a96f99a7fd37e2fffffffffffffffffff")),
        (bytes.fromhex("0c00000000000000000000000000000000"), bytes.fromhex("0cf6f10280ff7f2fffffffffffffffffff")),
      ],
    },
    ("ACC_ARMED_POSITIVE_LOW", "positive"): {
      "families": [
        (bytes.fromhex("37fff416ee6eef8ef346f52222844cc8ff"), bytes.fromhex("3700000000000000000000000000000000")),
        (bytes.fromhex("37fff7c3c95cd12f0a5e152222522665ff"), bytes.fromhex("3700000000000000000000000000000000")),
        (bytes.fromhex("0600000000000000000000000000000000"), bytes.fromhex("0682fe057f62802fffffffffffffffffff")),
      ],
    },
    ("ACC_ARMED_NEUTRAL", "neutral"): {
      "families": [
        (bytes.fromhex("3e00000000000000000000000000000000"), bytes.fromhex("3e07fe8c89ff7f2fffffffffffffffffff")),
        (bytes.fromhex("2e00000000000000000000000000000000"), bytes.fromhex("2efbf27a80ff7f2fffffffffffffffffff")),
      ],
    },
    ("MANAGED_NEUTRAL_IDLE", "neutral"): {
      "families": [
        (bytes.fromhex("1000000000000000000000000000000000"), bytes.fromhex("1100000000000000000000000000000000")),
        (bytes.fromhex("2000000000000000000000000000000000"), bytes.fromhex("2100000000000000000000000000000000")),
        (bytes.fromhex("0400000000000000000000000000000000"), bytes.fromhex("0500000000000000000000000000000000")),
        (bytes.fromhex("0a00000000000000000000000000000000"), bytes.fromhex("0a2bf6ff7ffe7f2fffffffffffffffffff")),
        (bytes.fromhex("3800000000000000000000000000000000"), bytes.fromhex("3900000000000000000000000000000000")),
      ],
    },
    ("ACC_ARMED_NEGATIVE", "negative"): {
      "families": [
        (bytes.fromhex("0000000000000000000000000000000000"), bytes.fromhex("00d9f95981ff7f2fffffffffffffffffff")),
        (bytes.fromhex("1600000000000000000000000000000000"), bytes.fromhex("1629f10780ff7f2fffffffffffffffffff")),
      ],
    },
    ("UNKNOWN_NEGATIVE", "negative"): {
      "families": [
        (bytes.fromhex("3c00000000000000000000000000000000"), bytes.fromhex("3c95f3837fff7f2fffffffffffffffffff")),
        (bytes.fromhex("2bfff10000000000000000000000000000"), bytes.fromhex("2b00000000000000000000000000000000")),
      ],
    },
    ("ACC_ARMED_BLENDED", "blended"): {
      "families": [
        (bytes.fromhex("3600000000000000000000000000000000"), bytes.fromhex("366bf13485ff7f2fffffffffffffffffff")),
        (bytes.fromhex("1200000000000000000000000000000000"), bytes.fromhex("12e9fba980a07e2fffffffffffffffffff")),
        (bytes.fromhex("3dfc470aff07007efffffffffffffff899"), bytes.fromhex("3d00000000000000000000000000000000")),
        (bytes.fromhex("3600000000000000000000000000000000"), bytes.fromhex("3678f7f27cf2812fffffffffffffffffff")),
      ],
    },
    ("MANAGED_BLENDED", "blended"): {
      "families": [
        (bytes.fromhex("1bfffcca46b84def834e8e2222211552ff"), bytes.fromhex("1b00000000000000000000000000000000")),
        (bytes.fromhex("0dff2701ff07007ffffffffffffffff8e4"), bytes.fromhex("0d00000000000000000000000000000000")),
        (bytes.fromhex("3200000000000000000000000000000000"), bytes.fromhex("32def64e7f62802fffffffffffffffffff")),
        (bytes.fromhex("0400000000000000000000000000000000"), bytes.fromhex("0500000000000000000000000000000000")),
      ],
    },
  }
  LONG_PAIR_FALLBACK_BY_INTENT = {
    "positive": ("ACC_ARMED_POSITIVE_PULL", "positive"),
    "neutral": ("ACC_ARMED_NEUTRAL", "neutral"),
    "negative": ("ACC_ARMED_NEGATIVE", "negative"),
    "blended": ("ACC_ARMED_BLENDED", "blended"),
  }
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.VM = VehicleModel(CP)
    self.apply_angle_last = 0.0
    self.last_shadow_long_debug = None
    self.enable_long_tx_builder = bool(self.ENABLE_LONG_TX_BUILDER and CP.openpilotLongitudinalControl)
    self.enable_lateral_tx_builder = bool(self.ENABLE_LATERAL_TX_BUILDER)

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

  def _select_long_pair_family(self, CS) -> tuple[bytes, bytes, str]:
    stock_state = str(getattr(CS, "stock_long_state_fine", "unknown"))
    stock_intent = str(getattr(CS, "stock_long_intent", "unknown"))
    key = (stock_state, stock_intent)
    pair_entry = self.LONG_PAIR_TABLE.get(key)
    pair_source = f"{stock_state}|{stock_intent}|primary"
    if pair_entry is None:
      fallback_key = self.LONG_PAIR_FALLBACK_BY_INTENT.get(stock_intent, ("ACC_ARMED_NEUTRAL", "neutral"))
      pair_entry = self.LONG_PAIR_TABLE[fallback_key]
      pair_source = f"{fallback_key[0]}|{fallback_key[1]}|fallback"
    families = pair_entry["families"]
    live_54 = bytes(getattr(CS, "long_54_stock_template", b""))
    live_59 = bytes(getattr(CS, "long_59_stock_template", b""))
    live_phase54 = int(getattr(CS, "long_54_phase", 0)) & 0xFF
    live_phase59 = int(getattr(CS, "long_59_phase", 0)) & 0xFF

    def score_family(pair54: bytes, pair59: bytes) -> tuple[int, int, int]:
      # Prefer exact phase alignment first, then matching trailing bytes against
      # the rolling stock templates. This lets managed/blended states choose the
      # closest real family instead of a single hardcoded pair per state.
      phase_score = int(pair54[0] == live_phase54) + int(pair59[0] == live_phase59)
      suffix54 = sum(1 for a, b in zip(pair54[1:], live_54[1:]) if a == b) if len(live_54) == len(pair54) else 0
      suffix59 = sum(1 for a, b in zip(pair59[1:], live_59[1:]) if a == b) if len(live_59) == len(pair59) else 0
      return (phase_score, suffix54 + suffix59, suffix59)

    best_idx, best_pair, best_score = 0, families[0], score_family(*families[0])
    for idx, pair in enumerate(families[1:], start=1):
      score = score_family(*pair)
      if score > best_score:
        best_idx, best_pair, best_score = idx, pair, score
    return best_pair[0], best_pair[1], f"{pair_source}|fam{best_idx}"

  def _build_shadow_long_tx(self, CS, long_tx_hint: dict[str, int | str]) -> dict[str, int | str]:
    tx54_phase = int(getattr(CS, "long_54_phase", 0))
    tx59_phase = int(getattr(CS, "long_59_phase", 0))
    pair54, pair59, pair_source = self._select_long_pair_family(CS)
    tx54 = bytearray(pair54)
    tx59 = bytearray(pair59)
    tx54[0] = tx54_phase & 0xFF
    tx59[0] = tx59_phase & 0xFF
    return {
      "tx54_phase": tx54_phase,
      "tx54_template_hex": pair54.hex(),
      "tx54_core_hex": bytes(tx54).hex(),
      "tx59_phase": tx59_phase,
      "tx59_template_hex": pair59.hex(),
      "tx59_core_hex": bytes(tx59).hex(),
      "tx_pair_source": pair_source,
    }

  def _build_long_can_msgs(self, CS, long_tx_hint: dict[str, int | str], long_tx_core: dict[str, int | str]) -> list[tuple[int, bytes, int]]:
    if not self.enable_long_tx_builder or not getattr(self, "_stock_long_tx_gate", False):
      return []
    # Firmware injector uses the separate USB header base as the rule cycle selector,
    # not the FlexRay payload phase byte. For the current dynm-style i3 rules that
    # selector is fixed to cycle_base=1.
    base = 0x01
    tx54 = bytes.fromhex(str(long_tx_core["tx54_core_hex"]))
    tx59 = bytes.fromhex(str(long_tx_core["tx59_core_hex"]))
    # BMW i3 long looks like a coordinated two-frame family. Send both 54 and 59
    # together so the ECU sees a stock-like pair instead of a single patched frame.
    return [
      (54, bytes([base]) + tx54[:9], 1),
      (59, bytes([base]) + tx59[:9], 1),
    ]

  def _shadow_long_tx_hint(self, desired_accel: float, CS) -> dict[str, int | str]:
    helper_state = self._shadow_long_helper_state(CS)
    stock_intent = str(getattr(CS, "stock_long_intent", "unknown"))
    stock_state = str(getattr(CS, "stock_long_state_fine", "unknown"))
    return {
      "tx_mode": stock_intent,
      "tx_branch": 54,
      "tx_parity": self.LONG_54_ACTIVE_PARITY,
      "tx_target_wb": int(getattr(CS, "long_54_wb", 0)),
      "tx_target_wc": int(getattr(CS, "long_54_wc", 0)),
      "tx_source": stock_state,
      "tx_helper_state": helper_state,
      "tx_desired_accel": desired_accel,
    }

  @staticmethod
  def _wrap15(value: int) -> int:
    return value % 15

  @staticmethod
  def _build_i3_like_visible_72(phase: int, target_nibble: int) -> bytes:
    payload = bytearray([0] * 16)
    payload[0] = phase & 0xFF
    if phase & 0x01:
      # Real i3 control branch layout is:
      #   byte0 = phase/subframe
      #   byte1 = 0xFF
      #   byte2 = 0xF? where low nibble carries the command-state orbit
      #   byte3..7 = 0xFF
      #   byte8 = 0xE0
      #   byte9..15 = 0xFF
      payload[1] = 0xFF
      payload[2] = 0xF0 | (CarController._wrap15(target_nibble) & 0x0F)
      payload[3:8] = b"\xFF" * 5
      payload[8] = 0xE0
      payload[9:16] = b"\xFF" * 7
    return bytes(payload)

  @classmethod
  def _match_desired_angle_to_current(cls, desired_angle: float, current_angle: float, v_ego: float) -> float:
    if v_ego <= cls.LAT72_MATCH_DELTA_BP[0]:
      max_delta = cls.LAT72_MATCH_DELTA_V[0]
    elif v_ego >= cls.LAT72_MATCH_DELTA_BP[-1]:
      max_delta = cls.LAT72_MATCH_DELTA_V[-1]
    else:
      max_delta = cls.LAT72_MATCH_DELTA_V[-1]
      for i in range(len(cls.LAT72_MATCH_DELTA_BP) - 1):
        x0 = cls.LAT72_MATCH_DELTA_BP[i]
        x1 = cls.LAT72_MATCH_DELTA_BP[i + 1]
        if x0 <= v_ego <= x1:
          y0 = cls.LAT72_MATCH_DELTA_V[i]
          y1 = cls.LAT72_MATCH_DELTA_V[i + 1]
          t = 0.0 if x1 == x0 else (v_ego - x0) / (x1 - x0)
          max_delta = y0 + (y1 - y0) * t
          break
    delta = max(-max_delta, min(max_delta, float(desired_angle - current_angle)))
    return float(current_angle + delta)

  def _lat72_target_nibble(self, desired_angle: float, current_angle: float, stock_nibble: int, cmd_phase: int) -> int:
    error = float(desired_angle - current_angle)
    if abs(error) < self.LAT72_TARGET_DEADBAND_DEG:
      return self._wrap15(stock_nibble)

    phase = int(cmd_phase) & 0x1F
    if error > self.LAT72_TARGET_CMD_MIN_DEG:
      return self._wrap15(self.LAT72_POS_NIBBLE_BY_PHASE.get(phase, stock_nibble))
    if error < -self.LAT72_TARGET_CMD_MIN_DEG:
      return self._wrap15(self.LAT72_NEG_NIBBLE_BY_PHASE.get(phase, stock_nibble))
    return self._wrap15(stock_nibble)

  def _lateral_tx_readiness(self, CC, CS) -> tuple[bool, str]:
    reasons = []
    if not self.enable_lateral_tx_builder:
      reasons.append("builder_off")
    if not bool(CC.enabled):
      reasons.append("op_disabled")
    if not bool(CC.latActive):
      reasons.append("lat_inactive")
    if int(getattr(CS, "stock_lat60_phase", -1)) != int(getattr(CS, "stock_lat72_phase", -2)):
      reasons.append("60_72_phase_mismatch")
    ready = len(reasons) == 0
    return ready, "ready" if ready else "|".join(reasons)

  def _build_shadow_lateral_tx(self, CC, CS, desired_angle: float):
    lat_allowed = bool(CC.enabled and CC.latActive)
    dir_hint = str(getattr(CS, "stock_lat_dir_hint", "unknown"))
    mag = float(getattr(CS, "stock_lat_mag_hint", 0.0))
    trigger_phase = int(getattr(CS, "stock_lat60_phase", 0)) & 0xFF
    phase = int(getattr(CS, "stock_lat72_phase", 0)) & 0xFF
    predicted_phase = (phase + 4) & 0x3F
    cmd_phase = (predicted_phase >> 1) & 0x1F
    stock_nibble = int(getattr(CS, "stock_lat72_cnt_nibble", 0)) & 0x0F
    target_nibble = stock_nibble
    if lat_allowed:
      target_nibble = self._lat72_target_nibble(desired_angle, CS.out.steeringAngleDeg, stock_nibble, cmd_phase)
    visible72 = self._build_i3_like_visible_72(predicted_phase, target_nibble)
    lat_tx_ready, lat_tx_reason = self._lateral_tx_readiness(CC, CS)
    return {
      "lat_phase": phase,
      "lat_match_phase": predicted_phase,
      "lat_trigger_phase": trigger_phase,
      "lat_dir_hint": dir_hint,
      "lat_mag_hint": mag,
      "lat_tx_ready": lat_tx_ready,
      "lat_tx_reason": lat_tx_reason,
      "lat_tx_enabled": self.enable_lateral_tx_builder,
      "lat_tx_msg_count": 1 if self.enable_lateral_tx_builder and lat_allowed and lat_tx_ready else 0,
      "lat72_stock_nibble": stock_nibble,
      "lat72_cmd_phase": cmd_phase,
      "lat72_target_nibble": target_nibble,
      "lat72_err3": int(getattr(CS, "stock_lat72_err3", 0)),
      "lat72_err4": int(getattr(CS, "stock_lat72_err4", 0)),
      "lat72_orbit_match": bool(getattr(CS, "stock_lat72_orbit_match", False)),
      "lat72_current_angle": float(CS.out.steeringAngleDeg),
      "lat72_desired_angle": float(desired_angle),
      "lat72_angle_error": float(desired_angle - CS.out.steeringAngleDeg),
      "tx72_hex": visible72.hex(),
      "tx96_hex": "",
    }

  def _build_lateral_can_msgs(self, CC, lateral_tx):
    lat_allowed = bool(CC.enabled and CC.latActive)
    if not (self.enable_lateral_tx_builder and lat_allowed and bool(lateral_tx.get("lat_tx_ready", False))):
      return []
    phase72 = int(lateral_tx.get("lat_phase", 0)) & 0xFF
    # Current Pico firmware arms the 60->72 injector on cycle_base=1 only.
    # Keep the host-side override aligned to that bucket instead of queuing a
    # phase-sensitive payload that may get consumed on a later, mismatched cycle.
    if (phase72 & 0x03) != 0x01:
      return []
    desired_angle = float(lateral_tx.get("lat72_desired_angle", 0.0))
    current_angle = float(lateral_tx.get("lat72_current_angle", 0.0))
    if abs(desired_angle - current_angle) > self.LAT72_SAFE_ERROR_DEG:
      return []
    tx72 = bytes.fromhex(str(lateral_tx["tx72_hex"]))
    base72 = 0x01
    # Dynm-style i3 firmware currently patches frame 72 via the visible 16-byte
    # body, keyed by a fixed cycle_base=1 on trigger 60 -> target 72.
    return [
      (72, bytes([base72]) + tx72[:16], 0),
    ]

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS, now_nanos):
    actuators = CC.actuators
    helper_state = self._shadow_long_helper_state(CS)
    # Keep the TX gate simple and observable while finishing the injector path.
    # The previous stock-context gate stayed too opaque in real runs and blocked
    # sendcan entirely, which prevented us from distinguishing gate issues from
    # payload issues. Long TX should follow controlsd's longActive directly.
    self._stock_long_tx_gate = bool(CC.longActive)
    lat_allowed = bool(CC.latActive)

    desired_angle = self.apply_angle_last
    if lat_allowed:
      desired_angle = float(actuators.steeringAngleDeg)
      desired_angle = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                  True, CarControllerParams, self.VM)
      desired_angle = self._match_desired_angle_to_current(desired_angle, CS.out.steeringAngleDeg, CS.out.vEgoRaw)
      self.apply_angle_last = desired_angle

    lateral_tx = self._build_shadow_lateral_tx(CC, CS, desired_angle)
    lateral_can_msgs = self._build_lateral_can_msgs(CC, lateral_tx)
    desired_accel = float(actuators.accel)
    long_tx_hint = self._shadow_long_tx_hint(desired_accel, CS)
    long_tx_core = self._build_shadow_long_tx(CS, long_tx_hint)
    long_can_msgs = self._build_long_can_msgs(CS, long_tx_hint, long_tx_core)
    self.last_shadow_long_debug = {
      "desired_accel": desired_accel,
      "long_active": bool(CC.longActive),
      "long_tx_allowed": bool(self._stock_long_tx_gate),
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
      "stock_long_helper_state": helper_state,
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
      "long_tx_override_base": long_can_msgs[0][1][0] if long_can_msgs else -1,
      "long_tx_override_54_hex": long_can_msgs[0][1].hex() if long_can_msgs else "",
      "long_tx_override_59_hex": long_can_msgs[1][1].hex() if len(long_can_msgs) > 1 else "",
      **long_tx_core,
      **long_tx_hint,
      **lateral_tx,
      "lat72_override_hex": lateral_can_msgs[0][1].hex() if lateral_can_msgs else "",
      "lat72_override_base": lateral_can_msgs[0][1][0] if lateral_can_msgs else -1,
      "lat96_override_hex": "",
      "lat96_override_base": -1,
      "desired_angle": float(desired_angle),
      "lat_allowed": lat_allowed,
      "stock_acc_lateral_gate": False,
      "stock_lat_active_hint": bool(getattr(CS, "stock_lat_active_hint", False)),
      "stock_lat96_b1": int(getattr(CS, "stock_lat96_b1", 0)),
      "stock_lat96_b2": int(getattr(CS, "stock_lat96_b2", 0)),
      "stock_lat96_b3": int(getattr(CS, "stock_lat96_b3", 0)),
      "stock_lat112_b5": int(getattr(CS, "stock_lat112_b5", 0)),
      "stock_lat116_b5": int(getattr(CS, "stock_lat116_b5", 0)),
    }
    self.frame += 1
    return actuators.as_builder(), long_can_msgs + lateral_can_msgs
