from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
WheelSpeeds = structs.CarState.WheelSpeeds


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.shifter_values = CANDefine(DBC[CP.carFingerprint][Bus.pt]).dv.get("GEAR_STATE_EXPERIMENTAL", {})
    self._last_pedal_or_hold = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()

    # Confirmed / strongest candidates from the i3 FlexRay logs
    ret.wheelSpeeds = WheelSpeeds(
      fl=float(cp.vl["WHEEL_SPEED"]["WHEEL_SPEED_FL"]),
      fr=float(cp.vl["WHEEL_SPEED"]["WHEEL_SPEED_FR"]),
      rl=float(cp.vl["WHEEL_SPEED"]["WHEEL_SPEED_RL"]),
      rr=float(cp.vl["WHEEL_SPEED"]["WHEEL_SPEED_RR"]),
    )

    # Use averaged wheel speeds as primary vEgo until VEHICLE_SPEED_PROV is validated.
    wheel_speeds = [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]
    ret.vEgoRaw = float(sum(wheel_speeds) / 4.0)
    ret.vEgo = ret.vEgoRaw
    ret.aEgo = self.update_speed_kf(ret.vEgoRaw)[1]
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = float(cp.vl["EPS_ANGLE"]["STEERING_ANGLE_DEG"])
    ret.steeringTorque = float(cp.vl["STEER_TORQUE"]["STEERING_TORQUE"])
    ret.steeringPressed = abs(ret.steeringTorque) > 1.0

    # Provisional dynamic signals
    ret.yawRate = float(cp.vl["YAW_OR_DYN_PROV"]["YAW_RATE_PROV"])
    ret.brake = float(cp.vl["PEDAL_OR_HOLD_STATE_CANDIDATE"]["PEDAL_OR_HOLD_LEVEL"])
    ret.gas = 0.0
    ret.gasPressed = False

    # Experimental heuristic: frame 59 changed more clearly between regen-only stop and pedal stop.
    pedal_or_hold_raw = int(cp.vl["PEDAL_OR_HOLD_STATE_CANDIDATE"]["PEDAL_OR_HOLD_STATE_RAW"])
    ret.brakePressed = pedal_or_hold_raw != 0
    self._last_pedal_or_hold = pedal_or_hold_raw

    # Gear is still experimental; keep unknown when mapping is not validated.
    gear_raw = int(cp.vl["GEAR_STATE_EXPERIMENTAL"]["GEAR_STATE_RAW"])
    ret.gearShifter = {
      0: GearShifter.unknown,
      1: GearShifter.park,
      2: GearShifter.reverse,
      3: GearShifter.neutral,
      4: GearShifter.drive,
      5: GearShifter.sport,
      6: GearShifter.low,
    }.get(gear_raw, GearShifter.unknown)

    ret.cruiseState.available = False
    ret.cruiseState.enabled = False
    ret.cruiseState.standstill = ret.standstill

    ret.leftBlinker = False
    ret.rightBlinker = False
    ret.seatbeltUnlatched = False
    ret.doorOpen = False
    ret.stockAeb = False
    ret.stockFcw = False
    ret.espDisabled = False

    # No buttons available yet on FlexRay-only path.
    ret.buttonEvents = create_button_events([], [])
    return ret

  @staticmethod
  def get_can_parsers(CP):
    messages = [
      ("WHEEL_SPEED", 100),
      ("STEER_TORQUE", 100),
      ("EPS_ANGLE", 100),
      ("VEHICLE_SPEED_PROV", 100),
      ("YAW_OR_DYN_PROV", 100),
      ("GEAR_STATE_EXPERIMENTAL", 50),
      ("PEDAL_OR_HOLD_STATE_CANDIDATE", 50),
      ("BRAKE_BLEND_CANDIDATE_B", 50),
    ]
    return {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 0)}


DBC = {
  "BMW_I3_EXPERIMENTAL": {
    Bus.pt: "bmw_i3_flexray_custom_v3",
  },
}
