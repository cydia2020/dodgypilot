import math
from cereal import car
from common.numpy_fast import clip, interp
from common.params import Params
from common.realtime import DT_CTRL
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_interceptor_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command
from selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, FULL_SPEED_DRCC_CAR, RADAR_ACC_CAR_TSS1, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
# constants for fault workaround
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 19

params = Params()

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.torque_rate_limits = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_limited = False
    self.last_off_frame = 0
    self.last_gas_pressed_frame = 0
    self.e2e_long = params.get_bool("EndToEndLong")
    self.steer_rate_counter = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

  def update(self, CC, CS):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel or (not CC.enabled and CS.pcm_acc_status)

    _accel_max = CarControllerParams.ACCEL_MAX_CAMRY if self.CP.carFingerprint == CAR.CAMRY else CarControllerParams.ACCEL_MAX

    # function for interpolating force
    def perform_force_transition(start_force, end_force, force_transition_frames):
      force_increment = (end_force - start_force) / force_transition_frames
      for f_frames in range(force_transition_frames):
        final_interpolated_force = start_force + force_increment * f_frames

      return final_interpolated_force

    # gas and brake
    # Default interceptor logic
    if self.CP.enableGasInterceptor and CC.longActive and self.CP.openpilotLongitudinalControl and self.CP.carFingerprint not in FULL_SPEED_DRCC_CAR:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if self.CP.carFingerprint in (CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER, CAR.HIGHLANDERH):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint in (CAR.COROLLA,):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    # FSRDRCC openpilot SnG logic
    elif not CS.out.brakePressed and self.CP.enableGasInterceptor and ((self.CP.openpilotLongitudinalControl and self.CP.carFingerprint in FULL_SPEED_DRCC_CAR \
         and actuators.accel > 0. and CS.out.standstill and CS.pcm_acc_status == 7) or (not self.CP.openpilotLongitudinalControl and self.CP.carFingerprint \
         not in (TSS2_CAR, CAR.LEXUS_IS, CAR.LEXUS_RC) and CS.stock_resume_ready)):
      interceptor_gas_cmd = 0.14
    else:
      interceptor_gas_cmd = 0.

    # record frames
    if not CC.enabled:
      self.last_off_frame = self.frame
    if not CS.out.gasPressed:
      self.last_gas_pressed_frame = self.frame

    # smooth in a forced used for offset based on current drive force
    force_transition_time = 0.5 # seconds to go from start to end force
    force_transition_frames = int(force_transition_time / DT_CTRL)
    start_force = 0. # start with 0
    end_force = CS.pcm_neutral_force # end with what we want to go to

    # only use the interpolated force 0.5 seconds after gas press or enabling
    if (self.frame - self.last_gas_pressed_frame) < force_transition_frames or \
      (self.frame - self.last_off_frame) < force_transition_frames:
      final_interpolated_force = perform_force_transition(start_force, end_force, force_transition_frames)
    else:
      final_interpolated_force = CS.pcm_neutral_force

    # accel offset logic
    accel_offset = 0.
    if CC.longActive:
      accel_offset = final_interpolated_force / self.CP.mass

    # calculate and clip pcm_accel_cmd
    pcm_accel_cmd = clip(actuators.accel + accel_offset, CarControllerParams.ACCEL_MIN, _accel_max)
    if not CC.longActive:
      pcm_accel_cmd = 0.

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.torque_rate_limits)
    self.steer_rate_limited = new_steer != apply_steer

    # EPS_STATUS->LKA_STATE either goes to 21 or 25 on rising edge of a steering fault and
    # the value seems to describe how many frames the steering rate was above 100 deg/s, so
    # cut torque with some margin for the lower state
    if CC.latActive and abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE:
      self.steer_rate_counter += 1
    else:
      self.steer_rate_counter = 0

    apply_steer_req = 1
    if not CC.latActive:
      apply_steer = 0
      apply_steer_req = 0
    elif self.steer_rate_counter >= MAX_STEER_RATE_FRAMES:
      apply_steer_req = 0
      self.steer_rate_counter = 0

    # cydia2020 - LVSTP Logic, 0.5 m/s to give radar some room for error
    # Lead is never stopped and openpilot is ready to be resumed when using e2e long
    lead_vehicle_stopped = (hud_control.leadVelocity < 0.5 and hud_control.leadVisible) and not self.e2e_long

    # release_standstill always 0 on radar_acc_tss1 cars
    if self.CP.carFingerprint in RADAR_ACC_CAR_TSS1:
      if CS.pcm_acc_status != 8:
        # pcm entered standstill or it's disabled
        self.standstill_req = False
    else:
      # cydia2020 - mimic stock behaviour, send standstill if the lead vehicle is stopped, else release
      # Don't go into standstill when using e2e long
      if CS.out.standstill and lead_vehicle_stopped and self.CP.carFingerprint not in NO_STOP_TIMER_CAR:
        self.standstill_req = True
      else:
        self.standstill_req = False

    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    can_sends = []

    # *** control msgs ***
    # print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, self.frame))

    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      can_sends.append(create_lta_steer_command(self.packer, 0, 0, self.frame // 2))

    # LTA mode. Set ret.steerControlType = car.CarParams.SteerControlType.angle and whitelist 0x191 in the panda
    # if self.frame % 2 == 0:
    #   can_sends.append(create_steer_command(self.packer, 0, 0, self.frame // 2))
    #   can_sends.append(create_lta_steer_command(self.packer, actuators.steeringAngleDeg, apply_steer_req, self.frame // 2))

    # Handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    alert_prompt = hud_control.audibleAlert in (AudibleAlert.promptDistracted, AudibleAlert.prompt)
    alert_prompt_repeat = hud_control.audibleAlert in (AudibleAlert.promptRepeat, AudibleAlert.warningSoft)
    alert_immediate = hud_control.audibleAlert == AudibleAlert.warningImmediate

    # we can spam can to cancel the system even if we are using lat only control
    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12. # at low speed we always assume the lead is present so ACC can be engaged
      adjust_distance = CS.distance_btn == 1

      # Send ACC_CONTROL_SAFE if RADAR Interceptor is detected, else send 0x343
      acc_msg = 'ACC_CONTROL_SAFE' if self.CP.carFingerprint in RADAR_ACC_CAR_TSS1 else 'ACC_CONTROL'
      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and self.CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
        can_sends.append(create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, adjust_distance, fcw_alert, CC.longActive, lead_vehicle_stopped, acc_msg))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type, adjust_distance, False, False, False, acc_msg))

    if self.frame % 2 == 0 and self.CP.enableGasInterceptor:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd

    if self.CP.carFingerprint != CAR.PRIUS_V:
      if self.frame % 10 == 0:
        can_sends.append(create_ui_command(self.packer, alert_prompt, alert_prompt_repeat, alert_immediate, hud_control.leftLaneVisible,
                                           hud_control.rightLaneVisible, CS.sws_toggle, CS.sws_sensitivity, CS.sws_buzzer, CS.sws_fld, 
                                           CS.sws_warning, CS.lda_left_lane, CS.lda_right_lane, CS.lda_sa_toggle, CS.lkas_status,
                                           CS.lda_speed_too_low, CS.lda_on_message, CS.lda_sensitivity, CS.ldw_exist, CC.enabled, CS.sws_beeps,
                                           CS.lda_take_control, CS.lda_adjusting_camera, CS.lda_unavailable_quiet, CS.lda_unavailable, 
                                           CS.lda_malfunction, CS.lda_fcb))

      if self.frame % 10 == 0 and self.CP.enableDsu:
        can_sends.append(create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
