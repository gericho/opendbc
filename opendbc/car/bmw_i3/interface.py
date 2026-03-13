from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw_i3.carcontroller import CarController
from opendbc.car.bmw_i3.carstate import CarState

ButtonType = structs.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, is_release, docs):
    ret.brand = "bmw"
    ret.dashcamOnly = False
    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = False
    ret.steerControlType = structs.CarParams.SteerControlType.angle
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
    ret.safetyConfigs = [
      get_safety_config(structs.CarParams.SafetyModel.noOutput),   # internal panda
      get_safety_config(structs.CarParams.SafetyModel.allOutput),  # external panda
    ]
    return ret
