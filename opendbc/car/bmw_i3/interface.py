from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw_i3.carcontroller import CarController
from opendbc.car.bmw_i3.carstate import CarState
from openpilot.system.hardware import PC

ButtonType = structs.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  ENABLE_LONG_CONTROL = False

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, is_release, docs):
    ret.brand = "bmw"
    ret.dashcamOnly = False
    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = bool(CarInterface.ENABLE_LONG_CONTROL)
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
    # PC bring-up uses a single pico-flexray panda. Advertising a dual-panda
    # safety layout causes selfdrived controlsMismatch, since panda 0 is the
    # active allOutput device rather than an internal noOutput panda.
    if PC:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]
    else:
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.noOutput),   # internal panda
        get_safety_config(structs.CarParams.SafetyModel.allOutput),  # external panda
      ]
    return ret
