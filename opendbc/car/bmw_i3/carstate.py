from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.bmw_i3.values import DBC

GearShifter = structs.CarState.GearShifter
WheelSpeeds = structs.CarState.WheelSpeeds


class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.shifter_values = CANDefine(DBC[CP.carFingerprint][Bus.pt]).dv.get("DRIVE_STATE_EXPERIMENTAL", {})

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
    ret.gasPressed = False

    brake_can_byte1 = int(cp.vl.get("PTCAN_BRAKE_PRESSED_CANDIDATE", {}).get("BRAKE_PRESSED_BYTE_1_PT_CAN", 0xFF))
    ret.brakePressed = brake_can_byte1 < 0x10

    gear_raw = int(cp.vl.get("DRIVE_STATE_EXPERIMENTAL", {}).get("DRIVE_STATE_RAW", 0))
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

    ret.buttonEvents = []
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
      ("PTCAN_BRAKE_PRESSED_CANDIDATE", 50),
    ]
    return {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 0)}
