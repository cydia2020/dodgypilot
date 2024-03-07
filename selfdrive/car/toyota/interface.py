from cereal import car
from cereal import log
from panda import Panda
from panda.python import uds
from openpilot.selfdrive.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                        MIN_ACC_SPEED, EPS_SCALE, UNSUPPORTED_DSU_CAR, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR, STOP_AND_GO_CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.disable_ecu import disable_ecu
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params

EventName = car.CarEvent.EventName
SteerControlType = car.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.pcmFollowDistancePrev = 0

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "toyota"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    # BRAKE_MODULE is on a different address for these cars
    if DBC[candidate]["pt"] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_ALT_BRAKE

    if candidate in ANGLE_CONTROL_CAR:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_LTA

      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    ret.stoppingControl = False  # Toyota starts braking more when it thinks you want to stop


    if candidate == CAR.PRIUS:
      # Only give steer angle deadzone to for bad angle sensor prius
      for fw in car_fw:
        if fw.ecu == "eps" and not (fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00'):
          ret.steerActuatorDelay = 0.25
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, \
                                                 steering_angle_deadzone_deg=0.0 if 0x23 in fingerprint[0] else 0.2)

    elif candidate in (CAR.RAV4_TSS2, CAR.RAV4_TSS2_2022, CAR.RAV4_TSS2_2023):
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kpBP = [0.0]
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.1]
      ret.lateralTuning.pid.kf = 0.00007818594

      # 2019+ RAV4 TSS2 uses two different steering racks and specific tuning seems to be necessary.
      # See https://github.com/commaai/openpilot/pull/21429#issuecomment-873652891
      for fw in car_fw:
        if fw.ecu == "eps" and (fw.fwVersion.startswith(b'\x02') or fw.fwVersion in [b'8965B42181\x00\x00\x00\x00\x00\x00']):
          ret.lateralTuning.pid.kpV = [0.15]
          ret.lateralTuning.pid.kiV = [0.05]
          ret.lateralTuning.pid.kf = 0.00004
          break

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    ret.enableBsm = 0x3F6 in fingerprint[0] and candidate in TSS2_CAR

    # Detect smartDSU, which intercepts ACC_CMD from the DSU (or radar) allowing openpilot to send it
    # 0x2AA is sent by a similar device which intercepts the radar instead of DSU on NO_DSU_CARs
    if 0x2FF in fingerprint[0] or (0x2AA in fingerprint[0] and candidate in NO_DSU_CAR):
      ret.flags |= ToyotaFlags.SMART_DSU.value

    if 0x343 in fingerprint[2] and candidate not in TSS2_CAR:
      ret.flags |= ToyotaFlags.DSU_BYPASS.value

    if 0x23 in fingerprint[0]:
      ret.flags |= ToyotaFlags.SECONDARY_STEER_ANGLE.value
    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make an adas dbc file for dsu-less models
    ret.radarUnavailable = DBC[candidate]['radar'] is None or candidate in (NO_DSU_CAR - TSS2_CAR)

    # In TSS2 cars, the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]
    ret.enableDsu = len(found_ecus) > 0 and Ecu.dsu not in found_ecus and candidate not in (NO_DSU_CAR | UNSUPPORTED_DSU_CAR) \
                                        and not (ret.flags & ToyotaFlags.SMART_DSU)

    # if the smartDSU is detected, openpilot can send ACC_CONTROL and the smartDSU will block it from the DSU or radar.
    # since we don't yet parse radar on TSS2/TSS-P radar-based ACC cars, gate longitudinal behind experimental toggle
    use_sdsu = bool(ret.flags & ToyotaFlags.SMART_DSU)
    if candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      ret.experimentalLongitudinalAvailable = use_sdsu or candidate in RADAR_ACC_CAR

      if not use_sdsu:
        # Disabling radar is only supported on TSS2 radar-ACC cars
        if experimental_long and candidate in RADAR_ACC_CAR:
          ret.flags |= ToyotaFlags.DISABLE_RADAR.value
      else:
        use_sdsu = use_sdsu and experimental_long

    # openpilot longitudinal enabled by default:
    #  - non-(TSS2 radar ACC cars) w/ smartDSU installed
    #  - cars w/ DSU disconnected
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind experimental long toggle:
    #  - TSS2 radar ACC cars w/ smartDSU installed
    #  - TSS2 radar ACC cars w/o smartDSU installed (disables radar)
    #  - TSS-P DSU-less cars w/ CAN filter installed (no radar parser yet)
    ret.openpilotLongitudinalControl = use_sdsu or ret.enableDsu or candidate in (TSS2_CAR - RADAR_ACC_CAR) or \
                                       bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value) or bool(ret.flags & ToyotaFlags.DSU_BYPASS.value)
    ret.autoResumeSng = ret.openpilotLongitudinalControl and candidate in NO_STOP_TIMER_CAR
    ret.enableGasInterceptor = 0x201 in fingerprint[0] and ret.openpilotLongitudinalControl

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_STOCK_LONGITUDINAL

    if ret.enableGasInterceptor:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_GAS_INTERCEPTOR

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if (candidate in STOP_AND_GO_CAR or ret.enableGasInterceptor) else MIN_ACC_SPEED

    # on stock Toyota this is -2.5
    ret.stopAccel = -2.5

    tune = ret.longitudinalTuning
    tune.deadzoneBP = [0., 12., 20.]
    tune.deadzoneV = [0., .08, .08]
    ret.stoppingDecelRate = 0.15  # This is okay for TSS-P
    if candidate in TSS2_CAR:
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stoppingDecelRate = 0.1  # reach stopping target smoothly
    tune.kpBP = [0., 5.]
    tune.kpV = [0.8, 1.]
    tune.kiBP = [0., 5.]
    tune.kiV = [0.3, 1.]

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    # disable radar if alpha longitudinal toggled on radar-ACC car without CAN filter/smartDSU
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(logcan, sendcan, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # events
    events = self.create_common_events(ret)

    if self.CS.pcmFollowDistance == 1 and self.pcmFollowDistancePrev != 1:
      Params().put_nonblocking("LongitudinalPersonality", str(log.LongitudinalPersonality.relaxed))
      self.pcmFollowDistancePrev = 1
    if self.CS.pcmFollowDistance == 2 and self.pcmFollowDistancePrev != 2:
      Params().put_nonblocking("LongitudinalPersonality", str(log.LongitudinalPersonality.standard))
      self.pcmFollowDistancePrev = 2
    if self.CS.pcmFollowDistance == 3 and self.pcmFollowDistancePrev != 3:
      Params().put_nonblocking("LongitudinalPersonality", str(log.LongitudinalPersonality.aggressive))
      self.pcmFollowDistancePrev = 3

    # LDA faults if user does not put their hands on the steering wheel
    # disallow engagement if LDA Steering Assist is ON
    if self.CS.lda_sa_toggle == 1:
      events.add(car.CarEvent.EventName.invalidLkasSetting)

    # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
    # the more accurate angle sensor signal is initialized
    if self.CP.steerControlType == SteerControlType.angle and not self.CS.accurate_steer_angle_seen:
      events.add(EventName.vehicleSensorsInvalid)

    if self.CP.openpilotLongitudinalControl:
      if ret.cruiseState.standstill and not ret.brakePressed and not self.CP.enableGasInterceptor:
        events.add(EventName.resumeRequired)
      if self.CS.low_speed_lockout:
        events.add(EventName.lowSpeedLockout)
      if ret.vEgo < self.CP.minEnableSpeed:
        events.add(EventName.belowEngageSpeed)
        if c.actuators.accel > 0.3:
          # some margin on the actuator to not false trigger cancellation while stopping
          events.add(EventName.speedTooLow)
        if ret.vEgo < 0.001:
          # while in standstill, send a user alert
          events.add(EventName.manualRestart)

    ret.events = events.to_msg()

    return ret

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
