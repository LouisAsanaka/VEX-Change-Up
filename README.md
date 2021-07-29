# VEX-Change-Up

[![Created Badge](https://badges.pufler.dev/created/louisasanaka/vex-change-up)](https://badges.pufler.dev)
[![Visits Badge](https://badges.pufler.dev/visits/louisasanaka/vex-change-up)](https://badges.pufler.dev)

Crazy messy codebase for our VEX Change Up season X-drive. Some code is derived from [OkapiLib](https://github.com/OkapiLib/OkapiLib) (odometry and drive templates), [WPILib](https://github.com/wpilibsuite/allwpilib) (PID, geometry classes in particular), & theol0403's [odomDebug](https://github.com/theol0403/odomDebug).

Features include:
- 3-wheel odometry for a holonomic drivetrain (x-drive was used)
- Experimental pure pursuit pathing (holonomic-only)
- Motion profiling (trapezoidal profile)
- Pose-to-pose strafing with a motion-profiled PID controller
- Speedy spline trajectory planner adapted from [FRC codebase](https://github.com/TASRobotics/RaidZero-FRC-2020/tree/master/pathgen/src/main/java/raidzero/pathgen)
- Simple unit testing with OkapiLib ([CI script here](https://github.com/LouisAsanaka/VEX-Change-Up/blob/master/.github/workflows/ci.yml))

Not many people are going to see this or even use this code, but hopefully if you do, please excuse the messiness and lack of comments 😅.

Notes:
- The OkapiLib `OdomState` uses the following conventions:
    - +x is forward
    - +y is to the right
    - +θ is **clockwise**
- The custom chassis controllers (eg. `XOdomController`) use the following conventions:
    - Cartesian coordinates (+x is to the right, +y is upwards)
    - +θ is ***counter-clockwise***
- Assuming a `Pose2d` is fed in, follow the chassis controller conventions.
