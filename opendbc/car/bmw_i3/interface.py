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
    ret.pcmCruise = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.autoResumeSng = False
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = True
    ret.safetyConfigs = [
      get_safety_config(structs.CarParams.SafetyModel.noOutput),   # internal panda
      get_safety_config(structs.CarParams.SafetyModel.allOutput),  # external panda
    ]
    return ret
