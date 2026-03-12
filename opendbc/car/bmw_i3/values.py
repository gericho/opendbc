from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms


class BMWI3PlatformConfig(PlatformConfig):
  dbc_dict: DbcDict


class CAR(Platforms):
  BMW_I3_EXPERIMENTAL = BMWI3PlatformConfig(
    ["BMW i3 Experimental FlexRay"],
    CarSpecs(
      mass=1365.0,
      wheelbase=2.57,
      steerRatio=15.0,
      centerToFrontRatio=0.45,
    ),
    DbcDict({Bus.pt: "bmw_i3_flexray_custom_v3"}),
  )


DBC = CAR.create_dbc_map()
