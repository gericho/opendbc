from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.crc import CRC8J1850
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import CarControllerParams


class CarController(CarControllerBase):
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

  def _shadow_long_tx_hint(self, desired_accel: float) -> dict[str, int | str]:
    # Best current offline fit from the existing ACC routes:
    #   59 = positive/coast branch, mainly active on even subcycles
    #   54 = negative/brake-blend branch, mainly active on odd subcycles
    # This is not a closed payload mapping yet; it is a conservative replay
    # hint so the next live comparison uses the right branch family.
    if desired_accel < -0.05:
      return {
        "tx_mode": "negative",
        "tx_branch": 54,
        "tx_parity": self.LONG_54_ACTIVE_PARITY,
        "tx_target_wb": self.LONG_54_CENTER_WB,
        "tx_target_wc": self.LONG_54_CENTER_WC,
      }
    return {
      "tx_mode": "positive_or_coast",
      "tx_branch": 59,
      "tx_parity": self.LONG_59_ACTIVE_PARITY,
      "tx_target_wb": self.LONG_59_CENTER_WB,
      "tx_target_wc": self.LONG_59_CENTER_WC,
    }

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

    if CC.latActive:
      desired_angle = float(actuators.steeringAngleDeg)
      desired_angle = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                  True, CarControllerParams, self.VM)
      self.apply_angle_last = desired_angle

      cycle_count = self._next_shadow_cycle()
      if cycle_count % 4 == 1:
        cnt1 = self._next_shadow_cnt()
        angle_error = abs(desired_angle - CS.out.steeringAngleDeg)
        driver_override = bool(CS.out.steeringPressed)
        tja_ready = int(CS.out.vEgoRaw > 0.1 and not driver_override)
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
        if self.frame % 50 == 0:
          carlog.warning({
            "event": "bmw_i3_shadow_acc",
            "angle_deg": round(desired_angle, 3),
            "cycle_count": cycle_count,
            "cnt1": cnt1,
            "crc1": values["crc1"],
            "assist_mode": values["assist_mode"],
            "tja_ready": tja_ready,
            "driver_override": driver_override,
            "lat_triggered": lat_triggered,
            "steering_engaged": steering_engaged,
            "stock_lat96_phase": int(getattr(CS, "stock_lat96_phase", 0)),
            "stock_lat96_b1": int(getattr(CS, "stock_lat96_b1", 0)),
            "stock_lat112_b5": int(getattr(CS, "stock_lat112_b5", 0)),
            "stock_lat116_b5": int(getattr(CS, "stock_lat116_b5", 0)),
            "stock_lat_active_hint": bool(getattr(CS, "stock_lat_active_hint", False)),
            "stock_lat_dir_hint": str(getattr(CS, "stock_lat_dir_hint", "unknown")),
            "stock_lat_dir_confidence": str(getattr(CS, "stock_lat_dir_confidence", "none")),
            "steer_torque_req": round(steer_torque_req, 3),
            "torque_reserve": torque_reserve,
            "force_weaken": force_weaken,
            "payload": self.last_shadow_acc_bytes.hex(),
          })

    desired_accel = float(actuators.accel)
    long_tx_hint = self._shadow_long_tx_hint(desired_accel)
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
      "long_59_b3": int(getattr(CS, "long_59_b3", 0)),
      "long_59_b5": int(getattr(CS, "long_59_b5", 0)),
      # 54 = best current stock brake-blend / regen proxy
      "long_54_wb": int(getattr(CS, "long_54_wb", 0)),
      "long_54_wc": int(getattr(CS, "long_54_wc", 0)),
      "long_54_b4": int(getattr(CS, "long_54_b4", 0)),
      "long_54_b6": int(getattr(CS, "long_54_b6", 0)),
      **long_tx_hint,
    }
    if self.frame % 50 == 0:
      carlog.warning({
        "event": "bmw_i3_shadow_long",
        **self.last_shadow_long_debug,
      })

    self.frame += 1
    # Read-only shadow mode: build the logical frame shape, but do not send.
    return actuators.as_builder(), []
