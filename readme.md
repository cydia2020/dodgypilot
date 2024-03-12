<img src="/dodgy_logo.png"  width=50% height=50%>

# Welcome to dodgypilot

## What is this?
Dodgypilot is a dodgy fork of comma.ai's [openpilot](https://openpilot.comma.ai). It is actively maintained by cydia2020.

## General information
This fork has a vehicle whitelist, and will only run on a Toyota/Lexus vehicle.

*Please use the precompiled branch "release3-staging", this branch has been tested to function in normal circumstances.*

No feature backports once comma has determined a version to be ready for release, what's on the version stays on that version.
Old versions are left for future references, and should not be used by the end-user.


## Installation
For comma two users:
Choose "Custom Software" when setting up your comma device, and type "https://installer.comma.ai/cydia2020/release2-staging" in the input box.

For comma three users:
Choose "Custom Software" when setting up your comma device, and type "https://installer.comma.ai/cydia2020/release2-staging" in the input box.

Alternatively, ssh into your comma device, run `cd /data/openpilot`, `git remote add cydia2020 https://github.com/cydia2020/dodgypilot`, and `git checkout cydia2020/release2-staging` or `git checkout cydia2020/release3-staging`

## Preparing Your Vehicle
‼️ READ THIS BEFORE INSTALLING ‼️

IMPORTANT: You must turn off the vehicle's stock Steering Assist feature for dodgypilot to engage. To turn Steering Assist off, go to your car's settings screen, highlight the steering assist toggle, then press `OK` once to toggle it off. Toyota has a [helpful guide](https://www.youtube.com/watch?v=qEvAua6oobA) on this.

If you have a TSS-P Vehicle, it is recommended that you put together or buy a SmartDSU from Taobao or @ErichMoraga. It will lessen your chance of crashing while you are using dodgypilot. A SmartDSU also enables features such as follow distance adjustment and switching between openpilot and stock longitudinal. Alternatively, you can use a DSU reroute harness to achieve the same functionality, more details can be found [here](https://github.com/cydia2020/toyota-dsu-reroute-harness).

## Warnings and ToS
WARNING: openpilot might not compile on your device if I'm doing something to a non-precompiled branch, always wait for the code to stabilise before installing this fork on your device.

By using this software, you agree that:
1. Maintainers of this software, and by definition - their entities, are not responsible for personal injuries, or damages done to your properties (these include your comma device(s) and vehicle(s)) as a result of using this software.
2. You have viewed and acknowledged comma.ai and all its entities' terms of service.

You do everything at your own risk.

## Features
This fork:
1. Allows the user openpilot sounds. (Car will chime differently based on the severity of the alert if dodgypilot wants attention. openpilot sounds can be re-enabled by a toggle in the comma device's settings)
2. Supports ZSS.
3. Keeps factory LDA and SWS on Toyota/Lexus.
4. Displays radarState readings on the onroad UI.
5. Improves screen brightness handling by linking it with your headlights. To use this feature, go to settings, and turn on `Use Linked Brightness`.
6. Allows openpilot to be engaged even if adaptive cruise control is disabled.
7. Includes various longitudinal control improvements for Toyota vehicles.
8. Allows the driver to change openpilot's follow distance on Toyota/Lexus with openpilot longitudinal control (and SmartDSU / DSU reroute harness if the user has a TSS-P vehicle, please use [this](https://github.com/wocsor/panda/commit/0c10024d5250c737d5ae6b00f8d7c3341896b71f) firmware for your SmartDSU).
![Distance Indicator](/follow_distance_indicator.png)
9. Allows navigate on openpilot even without comma prime. (Shamelessly stolen from FrogPilot)

## Support dodgypilot
Thank you for using dodgypilot, if you would like to buy me a coffee, please use the link below.

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/donate/?business=ZE32GX6TZNMCG&no_recurring=1&item_name=Buy+me+a+coffee+and+support+the+development+and+maintenance+of+dodgypilot.&currency_code=AUD)

## Credits
Icon partially thanks to FallOutGirl9001 on DeviantArt.
