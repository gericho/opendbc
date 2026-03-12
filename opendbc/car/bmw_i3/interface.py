from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "bmw_i3"
    ret.brand = "bmw"
    ret.dashcamOnly = True
    ret.notCar = False
    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = False
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.experimentalLongitudinalAvailable = False
    ret.minEnableSpeed = -1.0
    ret.minSteerSpeed = 0.0
    ret.autoResumeSng = False
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.8
    ret.steerRatio = 15.0
    ret.wheelbase = 2.57
    ret.centerToFront = ret.wheelbase * 0.45
    ret.mass = 1365.0
    ret.tireStiffnessFactor = 1.0
    ret.safetyConfigs = []
    return ret
