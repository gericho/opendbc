from opendbc.car import structs


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.frame = 0

  def update(self, CC: structs.CarControl, CS, now_nanos):
    self.frame += 1
    # Experimental read-only stub: no actuator output yet.
    return [], []
