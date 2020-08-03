# VEX-Change-Up

Notes:
- The OkapiLib `OdomState` uses the following conventions:
    - +x is forward
    - +y is to the right
    - +θ is **clockwise**
- The custom chassis controllers (eg. `XOdomController`) use the following conventions:
    - Cartesian coordinates (+x is to the right, +y is upwards)
    - +θ is ***counter-clockwise***
- Assuming a `Pose2d` is fed in, follow the chassis controller conventions.