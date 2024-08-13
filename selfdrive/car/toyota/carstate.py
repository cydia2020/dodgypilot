import copy

from cereal import car
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car import DT_CTRL
from openpilot.selfdrive.car.conversions import Conversions as CV
from openpilot.selfdrive.car.filter_simple import FirstOrderFilter
from openpilot.selfdrive.car.helpers import mean
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR

SteerControlType = car.CarParams.SteerControlType

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.prev_distance_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.low_speed_lockout = False
    self.acc_type = 1
    self.lkas_hud = {}

    # Secondary Steer Sensor
    self.cruise_active_prev = False

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
      # TODO: find a common gas pedal percentage signal
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    zss_angle_deg = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"] if self.CP.flags & ToyotaFlags.SECONDARY_STEER_ANGLE else 0.

    # On some cars, the angle measurement is non-zero while initializing. Use if non-zero or ZSS
    # Also only get offset when ZSS comes up in case it's slow to start sending messages
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]) or \
       (self.CP.flags & ToyotaFlags.SECONDARY_STEER_ANGLE and abs(zss_angle_deg) > 1e-3):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      acc_angle_deg = zss_angle_deg if self.CP.flags & ToyotaFlags.SECONDARY_STEER_ANGLE else torque_sensor_angle_deg
      # Offset seems to be invalid for large steering angles and high angle rates
      # Compute offset after re-enabling
      if (abs(ret.steeringAngleDeg) < 90 or (bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"]) and not self.cruise_active_prev)) \
         and cp.can_valid:
        self.angle_offset.update(acc_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = acc_angle_deg - self.angle_offset.x
      self.cruise_active_prev = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])

    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    if self.CP.carFingerprint != CAR.TOYOTA_MIRAI:
      ret.engineRpm = cp.vl["ENGINE_RPM"]["RPM"]

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # brake lights
    ret.brakeLights = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0)

    # combination meter dimmer states
    ret.meterBrightness = cp.vl["BODY_CONTROL_STATE_2"]['METER_SLIDER_BRIGHTNESS_PCT'] * 0.5 if \
                          cp.vl["BODY_CONTROL_STATE"]['METER_DIMMED'] == 1 else 1.0 if \
                          cp.vl["BODY_CONTROL_STATE_2"]["METER_SLIDER_LOW_BRIGHTNESS"] == 1 else 100.

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

      # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
      # the more accurate angle sensor signal is initialized
      ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS

    # assume camera sends longitudinal signal when
    # 1- TSS 2.0 cars, and not TSS 2.0 RADAR ACC cars
    # 2- TSS-P cars with Irene's DSU bypass harness installed
    cp_acc = cp_cam if (self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) or bool(self.CP.flags & ToyotaFlags.DSU_BYPASS.value)) else cp

    # if TSS 2.0 or bypassing DSU, longitudinal messages (ACC, PCS) will be available for panda to read before it's filtered
    if self.CP.flags & ToyotaFlags.DSU_BYPASS.value or (self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value) \
      and not self.CP.flags & ToyotaFlags.SMART_DSU.value:
    # do not pass through ACC_TYPE on TSS-P cars regardless of 0x343 interceptor
      if not self.CP.flags & ToyotaFlags.DSU_BYPASS.value: # do not passthrough acc type signal if tss-p
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
    # alert signal for stock FCW, available when PCS is sent before panda's filter
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      # ignore standstill state in certain vehicles, since pcm allows to restart with just an acceleration request
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = False  # allow lateral when non-adaptive
    self.pcm_neutral_force = cp.vl["PCM_CRUISE"]["NEUTRAL_FORCE"]

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])
      self.sws_beeps = (cp_cam.vl["LKAS_HUD"]["TWO_BEEPS"])
      self.lda_left_lane = (cp_cam.vl["LKAS_HUD"]["LEFT_LINE"] == 3)
      self.lda_right_lane = (cp_cam.vl["LKAS_HUD"]["RIGHT_LINE"] == 3)
      self.lda_sa_toggle = (cp_cam.vl["LKAS_HUD"]["LDA_SA_TOGGLE"])

    if self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR:
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    # gate distance button behind main on
    if ret.cruiseState.available and (self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) or (self.CP.flags & ToyotaFlags.SMART_DSU) \
       or (self.CP.flags & ToyotaFlags.DSU_BYPASS)):
      # distance button is wired to the ACC module (camera or radar)
      self.prev_distance_button = self.distance_button
      if not self.CP.flags & ToyotaFlags.SMART_DSU and \
          (self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) or self.CP.flags & ToyotaFlags.DSU_BYPASS):
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]
      else:
        self.distance_button = cp.vl["SDSU"]["FD_BUTTON"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
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
      ("PCM_CRUISE_SM", 1),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    if CP.carFingerprint != CAR.TOYOTA_MIRAI:
      messages.append(("ENGINE_RPM", 42))

    if CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      messages.append(("DSU_CRUISE", 5))
      messages.append(("PCM_CRUISE_ALT", 1))
    else:
      messages.append(("PCM_CRUISE_2", 33))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      messages.append(("GAS_SENSOR", 50))

    # add zss if detected
    if CP.flags & ToyotaFlags.SECONDARY_STEER_ANGLE:
      messages.append(("SECONDARY_STEER_ANGLE", 0))  # rate inconsistent

    if CP.enableBsm:
      messages.append(("BSM", 1))

    if CP.carFingerprint in RADAR_ACC_CAR and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not CP.flags & ToyotaFlags.SMART_DSU.value:
        messages += [
          ("ACC_CONTROL", 33),
        ]
      messages += [
        ("PCS_HUD", 1),
      ]

    if CP.flags & ToyotaFlags.SMART_DSU:
      messages += [
        ("SDSU", 100),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = []

    if CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      messages += [
        ("LKAS_HUD", 1),
      ]

    if CP.flags & ToyotaFlags.DSU_BYPASS.value or CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      messages += [
        ("ACC_CONTROL", 33),
        ("PCS_HUD", 1),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
