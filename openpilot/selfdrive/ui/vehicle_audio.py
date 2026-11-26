from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.toyota.values import CAR as ToyotaCAR

Source = log.SelfdriveState.AudibleAlert
Target = car.CarControl.HUDControl.AudibleAlert
Visual = car.CarControl.HUDControl.VisualAlert
Size = log.SelfdriveState.AlertSize

# These disengagements are explicitly initiated by the driver. A PCM disable
# without a known driver action is not necessarily a routine cancellation.
ROUTINE_CANCEL = ("buttonCancel/userDisable", "pedalPressed/userDisable",
                  "steerDisengage/userDisable", "parkBrake/userDisable")


def supports_toyota_vehicle_audio(CP):
  return CP.brand == "toyota" and CP.carFingerprint != ToyotaCAR.TOYOTA_PRIUS_V


def prius_vehicle_alert(state):
  """Return the Toyota dash request for the private vehicle-audio policy."""
  alert = state.alertSound
  if state.alertHudVisual == Visual.fcw:
    return Target.warningSoft
  if alert == Source.disengage:
    return Target.none if state.alertType in ROUTINE_CANCEL else Target.warningImmediate
  if alert in (Source.warningImmediateSystem, Source.warningImmediateUser):
    return Target.warningImmediate
  if alert == Source.warningSoft:
    return Target.warningSoft
  if alert == Source.promptDistracted or state.alertType == "steerSaturated/warning":
    return Target.promptDistracted
  if alert == Source.promptRepeat:
    return Target.promptRepeat
  if alert == Source.prompt:
    if state.alertHudVisual == Visual.steerRequired or state.alertSize != Size.small:
      return Target.prompt
    return Target.none
  return Target.none


def may_use_vehicle_audio(state):
  # Keep the collision-specific device tone, not a generic steering warning.
  return state.alertHudVisual != Visual.fcw
