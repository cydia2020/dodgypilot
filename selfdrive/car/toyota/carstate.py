import math
from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import mean
from common.filter_simple import FirstOrderFilter
from common.params import Params
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, TSS2_CAR, EPS_SCALE, RADAR_ACC_CAR_TSS1
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    params = Params()
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)
    self.has_zss = CP.hasZss
    self.cruise_active_prev = False
    self.allow_distance_adjustment = False if params.get_bool('EndToEndLong') else True

    self.low_speed_lockout = False
    self.acc_type = 1

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = ret.gas > 805
    else:
      # TODO: find a new, common signal
      msg = "GAS_PEDAL_HYBRID" if (self.CP.flags & ToyotaFlags.HYBRID) else "GAS_PEDAL"
      ret.gas = cp.vl[msg]["GAS_PEDAL"]
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.001

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    zss_angle_deg = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"] if self.has_zss else 0.

    # On some cars, the angle measurement is non-zero while initializing. Use if non-zero or ZSS
    # Also only get offset when ZSS comes up in case it's slow to start sending messages
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]) or \
       (self.has_zss and abs(zss_angle_deg) > 1e-3):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      acc_angle_deg = zss_angle_deg if self.has_zss else torque_sensor_angle_deg
      # Offset seems to be invalid for large steering angles
      # Compute offset after re-enabling
      if (abs(ret.steeringAngleDeg) < 90 or (bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"]) and not self.cruise_active_prev)) \
         and cp.can_valid:
        self.angle_offset.update(acc_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = acc_angle_deg - self.angle_offset.x
      self.cruise_active_prev = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])

    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] not in (1, 5)

    # cydia2020's stuff
    ret.parkingLightON = cp.vl["LIGHT_STALK"]['PARKING_LIGHT'] == 1
    ret.headlightON = cp.vl["LIGHT_STALK"]['LOW_BEAM'] == 1
    ret.meterDimmed = cp.vl["BODY_CONTROL_STATE"]['METER_DIMMED'] == 1
    ret.meterLowBrightness = cp.vl["BODY_CONTROL_STATE_2"]["METER_SLIDER_LOW_BRIGHTNESS"] == 1
    ret.brakeLights = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0)
    self.pcm_neutral_force = cp.vl["PCM_CRUISE"]["NEUTRAL_FORCE"]

    if self.CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
    else:
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS

    if self.CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
      ret.pcmFollowDistance = 3
    elif not self.allow_distance_adjustment:
      ret.pcmFollowDistance = 3
    else:
      ret.pcmFollowDistance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    if self.CP.carFingerprint in TSS2_CAR:
      self.acc_type = cp_cam.vl["ACC_CONTROL"]["ACC_TYPE"]
      self.distance_btn = 2 if (cp_cam.vl["ACC_CONTROL"]["DISTANCE"] == 1 and not self.allow_distance_adjustment) else 1 if (cp_cam.vl["ACC_CONTROL"]["DISTANCE"] == 1 and self.allow_distance_adjustment) else 0
    elif self.CP.smartDsu:
      self.distance_btn = 2 if (cp.vl["SDSU"]["FD_BUTTON"] == 1 and not self.allow_distance_adjustment) else 1 if (cp.vl["SDSU"]["FD_BUTTON"] == 1 and self.allow_distance_adjustment) else 0
    elif self.CP.carFingerprint in RADAR_ACC_CAR_TSS1 and self.CP.radarInterceptor:
      self.distance_btn = 2 if (cp.vl["ACC_CONTROL_COPY"]["DISTANCE"] == 1 and not self.allow_distance_adjustment) else 1 if (cp.vl["ACC_CONTROL_COPY"]["DISTANCE"] == 1 and self.allow_distance_adjustment) else 0
    else:
      self.distance_btn = 0

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in (CAR.LEXUS_IS, CAR.LEXUS_RC)) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)
    ret.pcmStandstill = self.pcm_acc_status == 7

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]["LKA_STATE"]

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.flags & ToyotaFlags.CHR_BSM:
      ret.rightBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.leftBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    # Calculate pitch, roll, yaw
    ret.kinematicsPitch = math.atan2(-cp.vl["KINEMATICS"]["ACCEL_Y"], math.sqrt(cp.vl["KINEMATICS"]["ACCEL_X"]**2 + ACCELERATION_DUE_TO_GRAVITY**2))
    ret.kinematicsYaw = math.radians(cp.vl["KINEMATICS"]["YAW_RATE"]) * (ret.vEgoRaw / 3.6)
    ret.kinematicsRoll = math.atan2(cp.vl["KINEMATICS"]["ACCEL_X"], math.sqrt(cp.vl["KINEMATICS"]["ACCEL_Y"]**2 + ACCELERATION_DUE_TO_GRAVITY**2))

    # LKAS_HUD is on a different address on the Prius V, don't send to avoid problems
    if self.CP.carFingerprint != CAR.PRIUS_V:
      self.sws_toggle = (cp_cam.vl["LKAS_HUD"]["LANE_SWAY_TOGGLE"])
      self.sws_sensitivity = (cp_cam.vl["LKAS_HUD"]["LANE_SWAY_SENSITIVITY"])
      self.sws_buzzer = (cp_cam.vl["LKAS_HUD"]["LANE_SWAY_BUZZER"])
      self.sws_fld = (cp_cam.vl["LKAS_HUD"]["LANE_SWAY_FLD"])
      self.sws_warning = (cp_cam.vl["LKAS_HUD"]["LANE_SWAY_WARNING"])
      self.sws_beeps = (cp_cam.vl["LKAS_HUD"]["TWO_BEEPS"])
      self.lda_left_lane = (cp_cam.vl["LKAS_HUD"]["LEFT_LINE"] == 3)
      self.lda_right_lane = (cp_cam.vl["LKAS_HUD"]["RIGHT_LINE"] == 3)
      self.lda_sa_toggle = (cp_cam.vl["LKAS_HUD"]["LDA_SA_TOGGLE"])
      self.lkas_status = (cp_cam.vl["LKAS_HUD"]["LKAS_STATUS"])
      self.lda_speed_too_low = (cp_cam.vl["LKAS_HUD"]["LDA_SPEED_TOO_LOW"])
      self.lda_on_message = (cp_cam.vl["LKAS_HUD"]["LDA_ON_MESSAGE"])
      self.lda_sensitivity = (cp_cam.vl["LKAS_HUD"]["LDA_SENSITIVITY"])
      self.ldw_exist = (cp_cam.vl["LKAS_HUD"]["LDW_EXIST"])
      self.lda_take_control = (cp_cam.vl["LKAS_HUD"]["TAKE_CONTROL"])
      self.lda_adjusting_camera = (cp_cam.vl["LKAS_HUD"]["ADJUSTING_CAMERA"])
      self.lda_unavailable_quiet = (cp_cam.vl["LKAS_HUD"]["LDA_UNAVAILABLE_QUIET"])
      self.lda_unavailable = (cp_cam.vl["LKAS_HUD"]["LDA_UNAVAILABLE"])
      self.lda_malfunction = (cp_cam.vl["LKAS_HUD"]["LDA_MALFUNCTION"])
      self.lda_fcb = (cp_cam.vl["LKAS_HUD"]["LDA_FRONT_CAMERA_BLOCKED"])

    # if openpilot does not control long and we are running on a TSS-P car, we can assume that
    # 0x343 will be present on the ADAS Bus. We assume resume will be ready when
    # 1) the car is no longer sending standstill
    # 2) the car is still in standstill
    if not self.CP.openpilotLongitudinalControl and self.CP.carFingerprint not in (TSS2_CAR, CAR.LEXUS_IS, CAR.LEXUS_RC):
      self.stock_resume_ready = (cp.vl["ACC_CONTROL"]["RELEASE_STANDSTILL"] == 1 and self.pcm_acc_status == 7)
    else:
      self.stock_resume_ready = False

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
      ("GEAR", "GEAR_PACKET"),
      ("BRAKE_PRESSED", "BRAKE_MODULE"),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS"),
      ("DOOR_OPEN_FL", "BODY_CONTROL_STATE"),
      ("DOOR_OPEN_FR", "BODY_CONTROL_STATE"),
      ("DOOR_OPEN_RL", "BODY_CONTROL_STATE"),
      ("DOOR_OPEN_RR", "BODY_CONTROL_STATE"),
      ("SEATBELT_DRIVER_UNLATCHED", "BODY_CONTROL_STATE"),
      ("PARKING_BRAKE", "BODY_CONTROL_STATE"),
      ("TC_DISABLED", "ESP_CONTROL"),
      ("BRAKE_HOLD_ACTIVE", "ESP_CONTROL"),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE", "STEER_ANGLE_SENSOR"),
      ("CRUISE_ACTIVE", "PCM_CRUISE"),
      ("CRUISE_STATE", "PCM_CRUISE"),
      ("GAS_RELEASED", "PCM_CRUISE"),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE_INITIALIZING", "STEER_TORQUE_SENSOR"),
      ("TURN_SIGNALS", "BLINKERS_STATE"),
      ("LKA_STATE", "EPS_STATUS"),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK"),
      # cydia2020's stuff
      ("METER_DIMMED", "BODY_CONTROL_STATE"),
      ("METER_SLIDER_LOW_BRIGHTNESS", "BODY_CONTROL_STATE_2"),
      ("BRAKE_LIGHTS_ACC", "ESP_CONTROL"),
      ("PARKING_LIGHT", "LIGHT_STALK"),
      ("LOW_BEAM", "LIGHT_STALK"),
      ("ACCEL_Y", "KINEMATICS"),
      ("ACCEL_X", "KINEMATICS"),
      ("YAW_RATE", "KINEMATICS"),
      ("NEUTRAL_FORCE", "PCM_CRUISE"),
    ]

    checks = [
      ("GEAR_PACKET", 1),
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("BODY_CONTROL_STATE_2", 2),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("KINEMATICS", 80),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    if CP.flags & ToyotaFlags.HYBRID:
      signals.append(("GAS_PEDAL", "GAS_PEDAL_HYBRID"))
      checks.append(("GAS_PEDAL_HYBRID", 33))
    else:
      signals.append(("GAS_PEDAL", "GAS_PEDAL"))
      checks.append(("GAS_PEDAL", 33))

    if CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
      signals.append(("MAIN_ON", "DSU_CRUISE"))
      signals.append(("SET_SPEED", "DSU_CRUISE"))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2"))
      signals.append(("SET_SPEED", "PCM_CRUISE_2"))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2"))
      signals.append(("PCM_FOLLOW_DISTANCE", "PCM_CRUISE_2"))
      checks.append(("PCM_CRUISE_2", 33))

    if CP.hasZss:
      signals.append(("ZORRO_STEER", "SECONDARY_STEER_ANGLE", 0))
      checks.append(("SECONDARY_STEER_ANGLE", 0))

    # checks for TSS-P RADAR ACC cars
    if CP.carFingerprint in RADAR_ACC_CAR_TSS1 and CP.radarInterceptor:
      signals.append(("DISTANCE", "ACC_CONTROL_COPY"))
      checks.append(("ACC_CONTROL_COPY", 33))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR"))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR"))
      checks.append(("GAS_SENSOR", 50))

    # checks for stock ACC auto resume
    if not CP.openpilotLongitudinalControl and CP.carFingerprint not in (TSS2_CAR, CAR.LEXUS_IS, CAR.LEXUS_RC):
      signals.append(("RELEASE_STANDSTILL", "ACC_CONTROL"))
      checks.append(("ACC_CONTROL", 33))

    if CP.enableBsm or (CP.flags & ToyotaFlags.CHR_BSM):
      signals += [
        ("L_ADJACENT", "BSM"),
        ("L_APPROACHING", "BSM"),
        ("R_ADJACENT", "BSM"),
        ("R_APPROACHING", "BSM"),
      ]
      checks.append(("BSM", 1))

    # SDSU
    if CP.smartDsu:
      signals.append(("FD_BUTTON", "SDSU"))
      checks.append(("SDSU", 0))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("FORCE", "PRE_COLLISION"),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION"),
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42),
      ("PRE_COLLISION", 0), # TODO: figure out why freq is inconsistent
    ]

    if CP.carFingerprint != CAR.PRIUS_V:
      signals += [
        ("LANE_SWAY_TOGGLE", "LKAS_HUD"),
        ("LANE_SWAY_SENSITIVITY", "LKAS_HUD"),
        ("LANE_SWAY_BUZZER", "LKAS_HUD"),
        ("LANE_SWAY_FLD", "LKAS_HUD"),
        ("LANE_SWAY_WARNING", "LKAS_HUD"),
        ("RIGHT_LINE", "LKAS_HUD"),
        ("LEFT_LINE", "LKAS_HUD"),
        ("LKAS_STATUS", "LKAS_HUD"),
        ("LDA_ON_MESSAGE", "LKAS_HUD"),
        ("LDA_SPEED_TOO_LOW", "LKAS_HUD"),
        ("LDA_SA_TOGGLE", "LKAS_HUD"),
        ("LDA_SENSITIVITY", "LKAS_HUD"),
        ("LDW_EXIST", "LKAS_HUD"),
        ("TWO_BEEPS", "LKAS_HUD"),
        ("TAKE_CONTROL", "LKAS_HUD"),
        ("ADJUSTING_CAMERA", "LKAS_HUD"),
        ("LDA_UNAVAILABLE_QUIET", "LKAS_HUD"),
        ("LDA_UNAVAILABLE", "LKAS_HUD"),
        ("LDA_MALFUNCTION", "LKAS_HUD"),
        ("LDA_FRONT_CAMERA_BLOCKED", "LKAS_HUD"),
      ]
      checks += [
        ("LKAS_HUD", 1),
      ]

    if CP.carFingerprint in TSS2_CAR:
      signals.append(("ACC_TYPE", "ACC_CONTROL"))
      signals.append(("DISTANCE", "ACC_CONTROL"))
      checks.append(("ACC_CONTROL", 33))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
