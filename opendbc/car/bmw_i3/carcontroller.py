from opendbc.car import structs
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS, now_nanos):
    self.frame += 1
    # Experimental read-only stub: no actuator output yet.
    return CC.actuators.as_builder(), []
