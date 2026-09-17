# Gazebo terrain-step test

The world `Tools/simulation/gz/worlds/terrain_step.sdf` contains a static
4 x 4 x 0.8 m box centered at Gazebo world `(5, 0, 0.4)`. Its top is at
`z=0.8`, and it spans `x=3..7`, `y=-2..2`. Gazebo uses ENU: positive X is east.
The vehicle spawns on the floor at the origin. Both the floor and box have a
repeatable contrast pattern for the optical-flow camera. The box has collision
and visual geometry; the downward GPU lidar measures the rendered surface.
The pattern overlays are 1–2 mm above the surfaces to avoid z-fighting.

The SDF and its `terrain_step_assets/` image assets are in the
`Tools/simulation/gz` submodule and must be retained together.

## Start

Close any previous PX4/Gazebo simulation first: PX4 otherwise attaches to the
world that is already running instead of launching this world.

From the PX4 repository root:

```sh
PX4_GZ_WORLD=terrain_step make px4_sitl gz_x500_flow
```

This builds the modified firmware and uses the x500 model with camera-based
optical flow and downward lidar. No Gazebo Classic installation is needed.

In the **PX4 shell** (not a Linux terminal), configure the indoor sensor case:

```sh
param set EKF2_GPS_CTRL 0
param set EKF2_BARO_CTRL 0
param set EKF2_OF_CTRL 1
param set EKF2_HGT_REF 2
param set EKF2_RNG_CTRL 2
param set EKF2_RNG_STEP 0.3
param set MPC_ALT_MODE 0
param save
```

Stop and relaunch using the same command so the estimator starts with these
settings. These settings persist in this SITL instance. The flow airframe also
defaults simulated GPS off; explicitly disabling GPS/barometer fusion above
prevents those height sources from masking the behavior being tested.

## Flight comparison

Use QGroundControl joystick/RC or your normal offboard flight controls. Do not
drag the vehicle with Gazebo's pose editor: teleporting bypasses the real IMU
motion and controller response that this test is intended to exercise.

1. Take off to approximately 2 m above the floor and stabilize.
2. Fly slowly east toward the box (roughly 0.3–0.5 m/s), with a constant altitude
   setpoint. Cross `x=3` and stop near `x=5, y=0`.
3. Hover over the box for at least 15 seconds. Command a small climb and descent
   there to check that real vertical motion is still observed.
4. Return to the floor, or continue east beyond `x=7`. Repeat several crossings.
5. Repeat from a fresh launch with `EKF2_RNG_STEP=0` to compare disabled behavior.
   Set the value in the PX4 shell, save, stop, and relaunch for each comparison.

With the feature enabled, actual height should remain near the selected room
height while downward distance changes from roughly 2 m to 1.2 m and back.
Allow for sensor mounting offsets and attitude. With the feature disabled,
range-reference height control can instead climb over the box and descend when
leaving it. Minimum clearance protection is still enabled.

For a raised-launch test, start a fresh simulation with the vehicle on the box:

```sh
PX4_GZ_WORLD=terrain_step PX4_GZ_MODEL_POSE='5,0,1.1,0,0,0' make px4_sitl gz_x500_flow
```

This starts slightly above the box. Let the landing gear settle on the top
before arming, take off above the box, then depart it at a fixed altitude setpoint.
The initial range datum is the box top, not the lower floor.

## Measurements

Check live data from the PX4 shell:

```sh
listener distance_sensor
listener vehicle_optical_flow
listener vehicle_local_position
listener vehicle_local_position_groundtruth
```

In the SITL ULog compare changes in:

- `vehicle_local_position_groundtruth.z`: actual simulated height (NED, positive
  down). This is the primary measure of whether the vehicle climbed or descended.
- `vehicle_local_position.z` and `vz`: EKF height and vertical velocity.
- `vehicle_local_position.dist_bottom`: estimated actual clearance.
- `distance_sensor.current_distance`: raw sensor distance.
- `vehicle_optical_flow.quality`: confirm usable image tracking on both surfaces.

Compare height changes rather than requiring ground truth and EKF absolute `z`
to share a zero. Also inspect the commanded altitude to distinguish estimator
behavior from deliberate controller terrain-following or clearance changes.

The world has passed `gz sdf -k` validation. That does not establish flight
performance. Camera/lidar footprints see different mixtures of floor and box
near an edge, so this is a more realistic test than the synthetic EKF unit tests.
