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
    # Match the current BMW generic angle-control baseline until i3-specific
    # closed-loop tuning data says otherwise.
    ret.lateralTuning.pid.kf = 0.00005
    ret.lateralTuning.pid.kpBP = [0., 10., 20., 35.]
    ret.lateralTuning.pid.kpV = [0.45, 0.40, 0.35, 0.30]
    ret.lateralTuning.pid.kiBP = [0., 10., 20., 35.]
    ret.lateralTuning.pid.kiV = [0.12, 0.10, 0.08, 0.06]
    ret.safetyConfigs = [
      get_safety_config(structs.CarParams.SafetyModel.noOutput),   # internal panda
      get_safety_config(structs.CarParams.SafetyModel.allOutput),  # external panda
    ]
    return ret
