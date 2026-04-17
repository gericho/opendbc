from opendbc.car import structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.bmw.values import CarControllerParams


class CarController(CarControllerBase):
  ENABLE_LATERAL_TX_BUILDER = True
  LAT131_SYNTH_CYCLE = 0x00
  LAT131_SYNTH_BYTE_1 = 0x80
  LAT131_SYNTH_BYTE_2 = 0xF0
  LAT131_SYNTH_BYTE_3 = 0x80
  LAT131_SYNTH_BYTE_4 = 0x7D
  LAT131_SYNTH_GATE_5 = 0x80
  LAT131_SYNTH_GATE_6 = 0x02
  LONG83_TARGET_SPEED_GAIN = 0.047815
  LONG83_TARGET_SPEED_OFFSET = -1460.510
  LATERAL_FORCE_WEAKEN_BP = [22.0, 31.0]
  LATERAL_FORCE_WEAKEN_V = [250.0, 250.0]
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.VM = VehicleModel(CP)
    self.apply_angle_last = 0.0
    self.last_shadow_debug = None
    self.enable_lateral_tx_builder = bool(self.ENABLE_LATERAL_TX_BUILDER)

  def _lateral_tx_readiness(self, CC, CS) -> tuple[bool, str]:
    reasons = []
    if not self.enable_lateral_tx_builder:
      reasons.append("builder_off")
    if not bool(CC.enabled):
      reasons.append("op_disabled")
    if not bool(CC.latActive):
      reasons.append("lat_inactive")
    if not bool(getattr(CS, "stock_lat_active_hint", False)):
      reasons.append("stock_lat_hint_off")
    ready = len(reasons) == 0
    return ready, "ready" if ready else "|".join(reasons)

  @classmethod
  def _target_speed_kph_to_u83(cls, target_speed_kph: float) -> int:
    u = int(round((float(target_speed_kph) - cls.LONG83_TARGET_SPEED_OFFSET) / cls.LONG83_TARGET_SPEED_GAIN))
    return max(0, min(65535, u))

  @staticmethod
  def _u83_to_b3b4(u: int) -> tuple[int, int]:
    u = max(0, min(65535, int(u)))
    return u & 0xFF, (u >> 8) & 0xFF

  def _build_shadow_lateral_tx(self, CC, CS, desired_angle: float):
    lat_allowed = bool(CC.enabled and CC.latActive)
    trigger_phase = int(getattr(CS, "stock_lat_trigger_phase", 0)) & 0xFF
    target_speed_kph = float(CC.hudControl.setSpeed) * CV.MS_TO_KPH
    target_u83 = self._target_speed_kph_to_u83(target_speed_kph)
    target_b3, target_b4 = self._u83_to_b3b4(target_u83)
    # 0x83 is no longer treated here as a lateral-angle payload.
    # The only active host-built content is the long target on bytes 3:4 and
    # the gate on bytes 5:6. Firmware keeps bytes 0/1/2/7/8 live from OEM.
    payload = bytes([
      self.LAT131_SYNTH_CYCLE,
      self.LAT131_SYNTH_BYTE_1,
      self.LAT131_SYNTH_BYTE_2,
      target_b3,
      target_b4,
      self.LAT131_SYNTH_GATE_5,
      self.LAT131_SYNTH_GATE_6,
    ])
    lat_tx_ready, lat_tx_reason = self._lateral_tx_readiness(CC, CS)
    return {
      "lat_builder_mode": "131_draft",
      "lat_trigger_phase": trigger_phase,
      "lat_tx_ready": lat_tx_ready,
      "lat_tx_reason": lat_tx_reason,
      "lat_tx_enabled": self.enable_lateral_tx_builder,
      "lat_tx_msg_count": 1 if self.enable_lateral_tx_builder and lat_allowed and lat_tx_ready else 0,
      "lat131_current_angle": float(CS.out.steeringAngleDeg),
      "lat131_desired_angle": float(desired_angle),
      "lat131_angle_error": float(desired_angle - CS.out.steeringAngleDeg),
      "lat131_target_speed_kph": target_speed_kph,
      "lat131_target_u83": target_u83,
      "lat131_target_b3": target_b3,
      "lat131_target_b4": target_b4,
      "tx131_hex": payload.hex(),
    }

  def _build_lateral_can_msgs(self, CC, lateral_tx):
    lat_allowed = bool(CC.enabled and CC.latActive)
    if not (self.enable_lateral_tx_builder and lat_allowed and bool(lateral_tx.get("lat_tx_ready", False))):
      return []
    tx131 = bytes.fromhex(str(lateral_tx["tx131_hex"]))
    base131 = 0x00
    return [
      # Host sends [base][payload0..6]. Firmware keeps payload bytes 0/1/2/7/8
      # live from the cached OEM template and overlays only payload bytes 3..6.
      (131, bytes([base131]) + tx131[:7], 0),
    ]

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS, now_nanos):
    actuators = CC.actuators
    lat_allowed = bool(CC.latActive)

    desired_angle = self.apply_angle_last
    if lat_allowed:
      desired_angle = float(actuators.steeringAngleDeg)
      desired_angle = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                  True, CarControllerParams, self.VM)
      self.apply_angle_last = desired_angle

    lateral_tx = self._build_shadow_lateral_tx(CC, CS, desired_angle)
    lateral_can_msgs = self._build_lateral_can_msgs(CC, lateral_tx)
    self.last_shadow_debug = {
      **lateral_tx,
      "lat96_override_hex": lateral_can_msgs[0][1].hex() if lateral_can_msgs else "",
      "lat96_override_base": lateral_can_msgs[0][1][0] if lateral_can_msgs else -1,
      "desired_angle": float(desired_angle),
      "lat_allowed": lat_allowed,
      "stock_acc_lateral_gate": False,
      "stock_lat_active_hint": bool(getattr(CS, "stock_lat_active_hint", False)),
      "stock_lat96_phase": int(getattr(CS, "stock_lat96_phase", 0)),
      "stock_lat96_cycle_count": int(getattr(CS, "stock_lat96_cycle_count", getattr(CS, "stock_lat96_phase", 0))),
      "stock_lat96_b1": int(getattr(CS, "stock_lat96_b1", 0)),
      "stock_lat96_b2": int(getattr(CS, "stock_lat96_b2", 0)),
      "stock_lat96_b3": int(getattr(CS, "stock_lat96_b3", 0)),
      "stock_lat112_b5": int(getattr(CS, "stock_lat112_b5", 0)),
      "stock_lat116_b5": int(getattr(CS, "stock_lat116_b5", 0)),
    }
    self.frame += 1
    return actuators.as_builder(), lateral_can_msgs
