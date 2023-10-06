def create_ipas_steer_command(packer, steer, enabled, steer_interceptor_enabled):
  """Creates a CAN message for the Toyota Steer Command."""
  if steer < 0:
    direction = 3
  elif steer > 0:
    direction = 1
  else:
    direction = 2

  mode = 3 if enabled else 1

  values = {
    "STATE": mode,
    "DIRECTION_CMD": direction,
    "ANGLE": steer,
    "SET_ME_X10": 0x10,
    "SET_ME_X40": 0x40
  }
  if steer_interceptor_enabled:
    return packer.make_can_msg("STEERING_IPAS_COMMA", 0, values)
  else:
    return packer.make_can_msg("STEERING_IPAS", 0, values)

def create_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_lta_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota LTA Steer Command."""

  values = {
    "COUNTER": raw_cnt + 128,
    "SETME_X1": 1,
    "SETME_X3": 3,
    "PERCENTAGE": 100,
    "SETME_X64": 0x64,
    "ANGLE": 0,
    "STEER_ANGLE_CMD": steer,
    "STEER_REQUEST": steer_req,
    "STEER_REQUEST_2": steer_req,
    "BIT": 0,
  }
  return packer.make_can_msg("STEERING_LTA", 0, values)


def create_accel_command(packer, accel, pcm_cancel, standstill_req, lead, acc_type, distance_button, fcw_alert, permit_braking, lead_vehicle_stopped, msg='ACC_CONTROL'):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel,
    "ACC_TYPE": acc_type,
    "DISTANCE": distance_button,
    "MINI_CAR": lead,
    "PERMIT_BRAKING": permit_braking,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
    "ALLOW_LONG_PRESS": 1,
    "ACC_CUT_IN": fcw_alert,
    "ACCEL_CMD_ALT": accel,
    "LEAD_STANDSTILL": lead_vehicle_stopped,
  }
  return packer.make_can_msg(msg, 0, values)


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "STANDSTILL_ON": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "PCS_INDICATOR": 1,
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "PCS_OFF": 1,
    "PCS_SENSITIVITY": 0,
  }
  return packer.make_can_msg("ACC_HUD", 0, values)


def create_ui_command(packer, alert_prompt, alert_prompt_repeat, alert_immediate, left_line, right_line, sws_toggle, 
                      sws_sensitivity, sws_buzzer, sws_fld, sws_warning, lda_left_lane, lda_right_lane, lda_sa_toggle,
                      lkas_status, lda_speed_too_low, lda_on_message, lda_sensitivity, ldw_exist, enabled, sws_beeps,
                      lda_take_control, lda_adjusting_camera, lda_unavailable_quiet, lda_unavailable, lda_malfunction,
                      lda_fcb):
  values = {
    "TWO_BEEPS": 1 if alert_prompt or sws_beeps else 0,
    "LDA_ALERT": 3 if alert_immediate else 2 if alert_prompt_repeat else 1 if alert_prompt else 0,
    "RIGHT_LINE": 3 if lda_right_lane else 1 if right_line else 2,
    "LEFT_LINE": 3 if lda_left_lane else 1 if left_line else 2,
    "BARRIERS" : 1 if enabled else 0,
    "REPEATED_BEEPS": 1 if alert_prompt_repeat or lda_right_lane or lda_left_lane else 0,

    # signal pass through
    "LANE_SWAY_FLD": sws_fld,
    "LANE_SWAY_BUZZER": sws_buzzer,
    "LANE_SWAY_WARNING": sws_warning,
    "LANE_SWAY_SENSITIVITY": sws_sensitivity,
    "LANE_SWAY_TOGGLE": sws_toggle,
    "LKAS_STATUS": lkas_status,
    "LDA_ON_MESSAGE": lda_on_message,
    "LDA_SPEED_TOO_LOW": lda_speed_too_low,
    "LDA_SA_TOGGLE": lda_sa_toggle,
    "LDA_SENSITIVITY": lda_sensitivity,
    "LDW_EXIST": ldw_exist,
    "LDA_FRONT_CAMERA_BLOCKED": lda_fcb,
    "TAKE_CONTROL": lda_take_control,
    "LDA_UNAVAILABLE": lda_unavailable,
    "LDA_MALFUNCTION": lda_malfunction,
    "LDA_UNAVAILABLE_QUIET": lda_unavailable_quiet,
    "ADJUSTING_CAMERA": lda_adjusting_camera,

    # static signals
    "SET_ME_X02": 2,
    "SET_ME_X01": 1,
  }
  return packer.make_can_msg("LKAS_HUD", 0, values)
