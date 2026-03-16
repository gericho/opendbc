from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.crc import CRC8J1850
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    # Shadow-only builder based on the existing BMW SP2018 ACC/72 method.
    # We do not transmit anything yet; this is only to converge on the
    # logical payload shape before touching real output.
    self.shadow_packer = CANPacker("bmw_sp2018")
    self.VM = VehicleModel(CP)
    self.apply_angle_last = 0.0
    self.shadow_cnt = 0
    self.last_shadow_acc_values = None
    self.last_shadow_acc_bytes = b""

  def _next_shadow_cnt(self) -> int:
    self.shadow_cnt = (self.shadow_cnt + 1) % 16
    return self.shadow_cnt

  def _crc8_j1850(self, data: bytes, init_value: int = 0xF1) -> int:
    crc = init_value & 0xFF
    for byte in data:
      crc ^= byte & 0xFF
      crc = CRC8J1850[crc]
    return crc

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS, now_nanos):
    actuators = CC.actuators

    if CC.latActive:
      desired_angle = float(actuators.steeringAngleDeg)
      desired_angle = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                  True, CarControllerParams, self.VM)
      self.apply_angle_last = desired_angle

      if self.frame % 2 == 0:
        values = {
          "cycle_count": 1,
          "crc1": 0,
          "cnt1": self._next_shadow_cnt(),
          "always_0x9": 9,
          "steering_angle_req": desired_angle,
          "steer_torque_req": 0.0,
          "TJA_ready": 0,
          "assist_mode": 1,
          "wayback_en1_lane_keeping_trigger": 0,
          "lane_keeping_triggered": 0,
          "like_assist_torque_reserve": 0xA0,
          "constants": 0x03ff17fe,
          "wayback_en_2": 0,
          "steering_engaged": 2,
          "maybe_assist_force_enhance": 0xA2,
          "maybe_assist_force_weaken": 0xFA,
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
            "cnt1": values["cnt1"],
            "crc1": values["crc1"],
            "payload": self.last_shadow_acc_bytes.hex(),
          })

    self.frame += 1
    # Read-only shadow mode: build the logical frame shape, but do not send.
    return actuators.as_builder(), []
