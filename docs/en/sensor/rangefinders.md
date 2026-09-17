# Distance Sensors (Rangefinders)

Distance sensors provide distance measurement that can be used for [terrain following](../flying/terrain_following_holding.md#terrain_following), [terrain holding](../flying/terrain_following_holding.md#terrain_hold) (i.e. precision hovering for photography), improved landing behaviour ([conditional range aid](../advanced_config/tuning_the_ecl_ekf.md#conditional-range-aiding)), warning of regulatory height limits, collision prevention, etc.

This section lists the distance sensors supported by PX4 (linked to more detailed documentation), and provides information about the [generic configuration](#configuration) required for all rangefinders, [testing](#testing), and simulation with [Gazebo](#gazebo-simulation) or [Gazebo-Classic](#gazebo-classic-simulation).
More detailed setup and configuration information is provided in the topics linked below (and sidebar).

<img src="../../assets/hardware/sensors/lidar_lite/lidar_lite_v3.jpg" alt="Lidar Lite V3" width="200px" /><img src="../../assets/hardware/sensors/lidar_lightware/sf11c_120_m.jpg" alt="LightWare SF11/C Lidar" width="200px" /><img src="../../assets/hardware/sensors/optical_flow/ark_flow_distance_sensor.jpg" alt="ARK Flow" width="200px">

## Supported Rangefinders

::: tip
This is a subset of the rangefinders that can be used with PX4.
[Modules: Distance Sensors](../modules/modules_driver_distance_sensor.md) lists PX4 drivers for other non-CAN rangefinders.
There may also be other DroneCAN rangefinders than those listed here.
:::

| Rangefinder                                 | Technology      | Range (min – max)        | Connections            | NDAA | Notes                               |
| ------------------------------------------- | --------------- | ------------------------ | ---------------------- | ---- | ----------------------------------- |
| [Ainstein US-D1 Standard Radar Altimeter]   | Microwave radar | ~50 m                    | UART                   | ✔️   |                                     |
| [ARK DIST SR]                               | ToF (850 nm IR) | 8 cm to ~30 m            | DroneCAN, UART         | ✔️   |                                     |
| [ARK DIST MR]                               | ToF (IR)        | 8 cm to ~50 m            | DroneCAN, UART         | ✔️   |                                     |
| [Benewake TFmini]                           | ToF (IR laser)  | ~12 m                    | UART                   | ~    |                                     |
| [Holybro ST VL53L1X Lidar]                  | ToF (IR)        | up to ~4 m               | I2C                    | ~    |                                     |
| [LeddarOne]                                 | ToF (IR)        | 1 cm – 40 m              | UART                   | ~    |                                     |
| [Lidar-Lite]                                | ToF (IR laser)  | 5 cm – 40 m              | I2C, PWM               | ~    |                                     |
| [LightWare SF11/C]                          | ToF (IR laser)  | up to ~120 m             | UART, I2C              | ~    |                                     |
| [LightWare LW20/C]                          | ToF (IR laser)  | up to ~100 m             | I2C                    | ~    | Waterproof (IP67) + servo           |
| [LightWare SF45/B]                          | ToF (IR laser)  | ~50 m                    | UART                   | ~    | Rotary lidar (collision prevention) |
| [MaxBotix I2CXL-MaxSonar-EZ]                | Ultrasonic      |                          | I2C                    | ~    |                                     |
| [RaccoonLab Cyphal & DroneCAN µRANGEFINDER] | ToF (IR)        | ~0.1 m – ~8 m            | DroneCAN, Cyphal       | ~    |                                     |
| [Sony AS-DT1]                               | ToF             | up to 40 m               | UART                   | ~    | Multipoint distance sensor          |
| [TeraRanger Evo 60 m]                       | ToF (IR)        | 0.5 m – 60 m             | I2C                    | ~    |                                     |
| [TeraRanger Evo 600Hz]                      | ToF (IR)        | 0.75 m – 8 m             | I2C                    | ~    | High update rate (600 Hz)           |
| [LightWare SF02] _(disc.)_                  | ToF (IR laser)  | ~50 m                    | UART                   | ~    | Discontinued                        |
| [LightWare SF10/A] _(disc.)_                | ToF (IR laser)  | ~25 m                    | UART, I2C              | ~    | Discontinued                        |
| [LightWare SF10/B] _(disc.)_                | ToF (IR laser)  | ~50 m                    | UART, I2C              | ~    | Discontinued                        |
| [LightWare SF10/C] _(disc.)_                | ToF (IR laser)  | ~100 m                   | UART, I2C              | ~    | Discontinued                        |
| [Lanbao PSK-CM8JL65-CC5] _(disc.)_          | ToF (IR)        | 0.17 m – 8 m             | UART                   | ✖️   | Discontinued                        |
| [TeraRanger One] _(disc.)_                  | ToF (IR)        | ~0.2 m – ~14 m (typical) | I2C (adapter required) | ~    | Discontinued                        |

[Ainstein US-D1 Standard Radar Altimeter]: ../sensor/ulanding_radar.md
[ARK DIST SR]: ../dronecan/ark_dist.md
[ARK DIST MR]: ../dronecan/ark_dist_mr.md
[Benewake TFmini]: ../sensor/tfmini.md
[Holybro ST VL53L1X Lidar]: #holybro-st-vl53l1x-lidar
[Lanbao PSK-CM8JL65-CC5]: ../sensor/cm8jl65_ir_distance_sensor.md
[LeddarOne]: ../sensor/leddar_one.md
[Lidar-Lite]: ../sensor/lidar_lite.md
[LightWare Lidar]: ../sensor/sfxx_lidar.md
[LightWare SF11/C]: ../sensor/sfxx_lidar.md
[LightWare LW20/C]: ../sensor/sfxx_lidar.md
[LightWare SF45/B]: ../sensor/sfxx_lidar.md
[LightWare SF02]: ../sensor/sfxx_lidar.md
[LightWare SF10/A]: ../sensor/sfxx_lidar.md
[LightWare SF10/B]: ../sensor/sfxx_lidar.md
[LightWare SF10/C]: ../sensor/sfxx_lidar.md
[MaxBotix I2CXL-MaxSonar-EZ]: #maxbotix-i2cxl-maxsonar-ez
[Sony AS-DT1]: ../sensor/sony_asdt1.md
[TeraRanger Evo 60 m]: ../sensor/teraranger.md
[TeraRanger Evo 600Hz]: ../sensor/teraranger.md
[TeraRanger One]: ../sensor/teraranger.md

These adaptors allows you to connect a non-CAN rangefinder via the CAN interface.
Note that the range depends on the connected rangefinder

| Adaptor                                                 | Connections      | NDAA |
| ------------------------------------------------------- | ---------------- | ---- |
| **Avionics Anonymous UAVCAN Laser Altimeter Interface** | DroneCAN         | ~    |
| [RaccoonLab Cyphal & DroneCAN Rangefinder Adapter]      | DroneCAN, Cyphal | ~    |

[RaccoonLab Cyphal & DroneCAN µRANGEFINDER]: #raccoonlab-cyphal-and-dronecan-μrangefinder
[RaccoonLab Cyphal & DroneCAN Rangefinder Adapter]: #raccoonlab-cyphal-and-dronecan-rangefinder-adapter

Note that some [Optical Flow](../sensor/optical_flow.md) sensors also include a rangefinder, such as [ARK Flow](../dronecan/ark_flow.md) and [ARK Flow MR](../dronecan/ark_flow_mr.md).

### ARK DIST SR & ARK DIST MR

[ARK DIST SR](../dronecan/ark_dist.md) and [ARK DIST MR](../dronecan/ark_dist_mr.md) are open-source Time-of-Flight (ToF) rangefinder modules, which are capable of measuring distances from 8cm to 30m and from 8cm to 50m, respectively.

The sensors support [DroneCAN](../dronecan/index.md), run [PX4 DroneCAN Firmware](../dronecan/px4_cannode_fw.md), and are packed into a tiny form factor.
They can be connected to a flight controller via its `CAN1` port, allowing additional sensors to connected through the `CAN2` port.

### Holybro ST VL53L1X Lidar

The [VL53L1X](https://holybro.com/products/st-vl53l1x-lidar) is a state-of-the-art, Time-of-Flight (ToF), laser-ranging sensor, enhancing the ST FlightSense™ product family.
It is the fastest miniature ToF sensor on the market with accurate ranging up to 4 m and fast ranging frequency up to 50 Hz.

It comes with a JST GHR 4 pin connector that is compatible with the I2C port on [Pixhawk 4](../flight_controller/pixhawk4.md), [Pixhawk 5X](../flight_controller/pixhawk5x.md), and other flight controllers that follow the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

### Lidar-Lite

[Lidar-Lite](../sensor/lidar_lite.md) is a compact, high-performance optical distant measurement rangefinder.
It has a sensor range from (5cm - 40m) and can be connected to either PWM or I2C ports.

### MaxBotix I2CXL-MaxSonar-EZ

The MaxBotix [I2CXL-MaxSonar-EZ](https://maxbotix.com/collections/i2cxl-maxsonar-ez-products) range has a number of relatively short-ranged sonar based rangefinders that are suitable for assisted takeoff/landing and collision avoidance.
These can be connected using an I2C port.

The rangefinders are enabled using the parameter [SENS_EN_MB12XX](../advanced_config/parameter_reference.md#SENS_EN_MB12XX).

### Lightware LIDARs

[Lightware SFxx Lidar](../sensor/sfxx_lidar.md) provide a broad range of lightweight “laser altimeters” that are suitable for many drone applications.

PX4 supports: SF11/c and SF/LW20.
PX4 can also be used with the following discontinued models: SF02, SF10/a, SF10/b, SF10/c.

Others may be supported via the [RaccoonLab Cyphal and DroneCAN Rangefinder Adapter](#raccoonlab-cyphal-and-dronecan-rangefinder-adapter) described below.

PX4 also supports the [LightWare LiDAR SF45 Rotating Lidar](../sensor/sf45_rotating_lidar.md) for [collision prevention](../computer_vision/collision_prevention.md) applications.

### Sony AS-DT1

[Sony AS-DT1](../sensor/sony_asdt1.md) is a multipoint distance sensor that connects to PX4 over a UART/serial port.
PX4 configures the sensor baud rate and measurement output from the driver.

### TeraRanger Rangefinders

[TeraRanger](../sensor/teraranger.md) provide a number of lightweight distance measurement sensors based on infrared Time-of-Flight (ToF) technology.
They are typically faster and have greater range than sonar, and smaller and lighter than laser-based systems.

PX4 supports the following models connected via the I2C bus: TeraRanger One, TeraRanger Evo 60m and TeraRanger Evo 600Hz.

### Ainstein US-D1 Standard Radar Altimeter

The _Ainstein_ [US-D1 Standard Radar Altimeter](../sensor/ulanding_radar.md) is compact microwave rangefinder that has been optimised for use on UAVs.
It has a sensing range of around 50m.
A particular advantages of this product are that it can operate effectively in all weather conditions and over all terrain types (including water).

### LeddarOne

[LeddarOne](../sensor/leddar_one.md) is small Lidar module with a narrow, yet diffuse beam that offers excellent overall detection range and performance, in a robust, reliable, cost-effective package.
It has a sensing range from 1cm to 40m and needs to be connected to a UART/serial bus.

### TFmini

The [Benewake TFmini Lidar](../sensor/tfmini.md) is a tiny, low cost, and low power LIDAR with 12m range.

### PSK-CM8JL65-CC5

<Badge type="info" text="Discontinued" />

The [Lanbao PSK-CM8JL65-CC5 ToF Infrared Distance Measuring Sensor](../sensor/cm8jl65_ir_distance_sensor.md) is a very small (38 mm x 18mm x 7mm, <10g) IR distance sensor with a 0.17m-8m range and millimeter resolution.
It must be connected to a UART/serial bus.

### Avionics Anonymous UAVCAN Laser Altimeter Interface

The [Avionics Anonymous UAVCAN Laser Altimeter Interface](../dronecan/avanon_laser_interface.md) allows several common rangefinders (e.g. [Lightware SF11/c, SF30/D](../sensor/sfxx_lidar.md), etc) to be connected to the [CAN](../can/index.md) bus via [DroneCAN](../dronecan/index.md), a more robust interface than I2C.

### RaccoonLab Cyphal and DroneCAN Rangefinder Adapter

The [RaccoonLab Cyphal and DroneCAN Rangefinder Adapter](https://raccoonlab.co/tproduct/360882105-910084093051-cyphal-and-dronecan-rangefinder-adapter) allows several common rangefinders to be connected to the CAN bus via Cyphal or DroneCAN, providing a more robust interface than I2C or UART.
This adapter efficiently reads measurements via I2C or UART and publishes range data in meters, making it a versatile solution for UAVs, robotics, and technical documentation applications.

Supported rangefinders include:

- LightWare LW20/C
- TF-Luna
- Garmin Lite V3
- VL53L1CB

### RaccoonLab Cyphal and DroneCAN µRANGEFINDER

[RaccoonLab µRANGEFINDER](https://docs.raccoonlab.co/guide/rangefinder/uRANGEFINDER.html) is designed to measure distance and publish it via Cyphal/DroneCAN protocols.
It can be used to estimate precision landing or object avoidance.

Features:

- [VL53L1CBV0FY-1](https://www.st.com/resource/en/datasheet/vl53l1.pdf) sensor
- Input voltage sensor
- CAN connectors: 2 [UCANPHY Micro (JST-GH 4)](https://docs.raccoonlab.co/guide/wires/).

## Configuration/Setup {#configuration}

Rangefinders are usually connected to either a serial (PWM) or I2C port (depending on the device driver), and are enabled on the port by setting a particular parameter.

The hardware and software setup that is _specific to each distance sensor_ is covered in their individual topics.

The generic configuration that is _common to all distance sensors_, covering both the physical setup and usage, is given below.

### Generic Configuration

The common rangefinder configuration is specified using [EKF2_RNG\_\*](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL) parameters.
These include (non exhaustively):

- [EKF2_RNG_POS_X](../advanced_config/parameter_reference.md#EKF2_RNG_POS_X), [EKF2_RNG_POS_Y](../advanced_config/parameter_reference.md#EKF2_RNG_POS_Y), [EKF2_RNG_POS_Z](../advanced_config/parameter_reference.md#EKF2_RNG_POS_Z) - offset of the rangefinder from the vehicle centre of gravity in X, Y, Z directions.
- [EKF2_RNG_PITCH](../advanced_config/parameter_reference.md#EKF2_RNG_PITCH) - A value of 0 degrees (default) corresponds to the range finder being exactly aligned with the vehicle vertical axis (i.e. straight down), while 90 degrees indicates that the range finder is pointing forward.
  Simple trigonometry is used to calculate the distance to ground if a non-zero pitch is used.
- [EKF2_RNG_DELAY](../advanced_config/parameter_reference.md#EKF2_RNG_DELAY) - approximate delay of data reaching the estimator from the sensor.
- [EKF2_RNG_SFE](../advanced_config/parameter_reference.md#EKF2_RNG_SFE) - Range finder range dependent noise scaler.
- [EKF2_RNG_NOISE](../advanced_config/parameter_reference.md#EKF2_RNG_NOISE) - Measurement noise for range finder fusion

## Indoor Terrain Steps with Range Height Reference

The experimental [EKF2_RNG_STEP](../advanced_config/parameter_reference.md#EKF2_RNG_STEP) option handles abrupt surface changes when a downward rangefinder is the height reference.
For example, flying from a floor over a table can change the terrain estimate while preserving vehicle altitude instead of commanding a climb.
The terrain offset persists when hovering above the table, and ordinary range height correction resumes after the transition.
Leaving the table can produce another terrain step.

To enable it, set:

- [EKF2_HGT_REF](../advanced_config/parameter_reference.md#EKF2_HGT_REF) to `2` (range).
- [EKF2_RNG_CTRL](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL) to `2` (enabled continuously).
- [EKF2_RNG_STEP](../advanced_config/parameter_reference.md#EKF2_RNG_STEP) to a positive minimum step size in metres, for example `0.3`, and reboot.
  The default `0` disables this feature and retains the existing estimator behavior.
- [MPC_ALT_MODE](../advanced_config/parameter_reference.md#MPC_ALT_MODE) to `0` for manual altitude control without controller terrain following or terrain hold.

The detector compares tilt-corrected range changes over a rolling window of up to 0.1 seconds with IMU-predicted vertical displacement.
Across input gaps of up to 0.3 seconds, a stable pre-gap baseline allows comparison with the last observation, with an additional gate for vertical-velocity uncertainty accumulated during the gap.
The normal rolling window stays at 0.1 seconds to avoid interpreting slow changes as abrupt steps.
This allows a sharp edge to span multiple sensor observations, including when leaving a raised surface.
All confirmed changes must exceed `EKF2_RNG_STEP`. There are two confirmation paths:

- A large change also exceeding three standard deviations of range difference noise can be confirmed after at least three settled observations and 0.15 seconds.
  The difference variance combines the previous and new surface range variances using [EKF2_RNG_NOISE](../advanced_config/parameter_reference.md#EKF2_RNG_NOISE) and [EKF2_RNG_SFE](../advanced_config/parameter_reference.md#EKF2_RNG_SFE).
- A smaller persistent change can be confirmed after at least five settled observations and 0.25 seconds, provided the previous surface was also stable.
  The previous surface uses at least three IMU-propagated observations spanning 0.1 seconds, selected from 0.04 to 0.3 seconds before detection (before the last pre-gap observation when bridging a gap).
  Their total spread must fit the settling tolerance. Both the mean new distance and latest distance must differ from the previous surface mean by more than `EKF2_RNG_STEP`.
  This path allows a table step to be detected when the configured distance-dependent uncertainty exceeds the step size.

The settling tolerance is the smaller of one configured range standard deviation and half `EKF2_RNG_STEP`, constrained to 0.02–0.1 m.
New observations must stay within this tolerance of the first observation in the settling interval.
This is a persistence heuristic, not a statistical guarantee: the detector does not divide uncertainty by the square root of sample count or assume independent noise.
A persistent sensor bias change can still look like terrain; very noisy returns may fail to settle.
Probation normally starts at the larger of half `EKF2_RNG_STEP` and twice the configured range standard deviation capped at 0.15 m (a maximum noise contribution of 0.3 m).
If the baseline spread is no more than the smaller of 0.03 m and one quarter of `EKF2_RNG_STEP`, the start gate instead uses the largest of half `EKF2_RNG_STEP`, 0.03 m, and three times that observed spread.
This lets a persistent 0.127 m transition between adjacent 20-inch and 25-inch boxes start detection with `EKF2_RNG_STEP=0.09144` (0.3 ft), without reducing the noise used for normal range fusion.
The surface tracker owns classification separately from EKF fusion. It has four states:

- **Tracking:** normal range height correction against the current terrain datum.
- **Transition:** retain the pre-edge reference and propagate it with IMU vertical motion while an endpoint settles.
- **Unresolved:** the classification budget or input continuity was lost; ordinary height correction remains disabled.
- **Reacquiring:** collect a stable endpoint while applying only limited range height correction.

Changing endpoints restarts the settling interval without discarding the original surface reference.
For one second after confirmation, a return toward the previous surface can also start a transition using the confirmed reference.
The transition budget ends at one second or 0.15 m of accumulated vertical-motion uncertainty, whichever comes first.
The uncertainty budget integrates vertical-velocity standard deviation as correlated error; it is a conservative bridging heuristic, not an independent altitude measurement.
An input gap longer than 0.3 seconds during a transition also enters unresolved recovery.
Weak candidates without a stable baseline must reach the large-change noise gate within 0.1 seconds or enter recovery early.

During a transition, range height and optical flow fusion are withheld.
Exhausting the budget does **not** authorize full height correction against the old surface, and there is no blind cooldown.
Recovery clips range innovation to ±0.05 m and uses observation variance of at least 0.25 m².
These limits bound the measurement supplied to the EKF; they are not a guarantee of maximum physical aircraft displacement.
Normal height reset paths cannot bypass this recovery policy.
Optical flow remains withheld while the surface distance is unresolved; existing estimator validity checks continue to apply if aiding is unavailable.

After at least five settled observations over 0.25 seconds, recovery can reanchor terrain if the original transition had credible surface-change evidence.
This preserves aircraft altitude and accumulated covariance, but does not recover room-relative height lost during ambiguity.
Reanchoring is reported separately from a normally confirmed step.
Without credible terrain evidence, limited correction continues until the range innovation is within 0.05 m before returning to tracking.
Noise recovery does not create a confirmed-surface reference.

A confirmed step changes only terrain, including covariance and terrain reset reporting; it does not reset vehicle altitude or vertical velocity.
Raw range is never offset, and optical flow uses the actual new surface distance after confirmation or reacquisition.
Minimum optical flow clearance protections remain active and can still command a climb.
Range observation noise normally uses configured sensor noise only: datum uncertainty is already represented by filter covariance.
The stored datum survives landing/takeoff and ordinary range loss within an estimator session, and resets on a full estimator reset or reboot.
If range is lost while tracking rather than during a detected transition, unseen surface changes remain unobservable on reacquisition.

The `estimator_range_step_status` topic records classification state, transitions, thresholds, sample intervals, bridge age/uncertainty, terrain changes and degraded recovery.
The default log profile records it and raw `distance_sensor` updates without rate limiting.
Diagnostic event timestamps use the delayed EKF horizon. The last event and terrain delta remain available between transitions.

This is a heuristic, not an independent room-relative height measurement.
A flap or sensor bias that stays stable long enough can still resemble terrain. Very noisy returns can prevent reacquisition, and actual vertical motion during long ambiguous transitions can accumulate error.
IMU, conventional optical flow and downward range alone cannot guarantee drift-free room-relative altitude.

Regression coverage includes the observation timing from log 241's gradual entry and chained exit, short range gaps, noise, real vertical motion, and prolonged ambiguity.
A simple closed-loop vertical point-mass test checks repeated crossings against simulated actual height and vertical commands; it does not replace full vehicle simulation or flight validation.

## Testing

The easiest way to test the rangefinder is to vary the range and compare to the values detected by PX4.
The sections below show some approaches to getting the measured range.

### QGroundControl MAVLink Inspector

The _QGroundControl MAVLink Inspector_ lets you view messages sent from the vehicle, including `DISTANCE_SENSOR` information from the rangefinder.
The main difference between the tools is that the _Analyze_ tool can plot values in a graph.

::: info
The messages that are sent depend on the vehicle configuration.
You will only get `DISTANCE_SENSOR` messages if the connected vehicle has a rangefinder installed and is publishing sensor values.
:::

To view the rangefinder output:

1. Open the menu **Q > Select Tool > Analyze Tools**:

   ![Menu for QGC Analyze Tool](../../assets/qgc/analyze/menu_analyze_tool.png)

1. Select the message `DISTANCE_SENSOR`, and then check the plot checkbox against `current_distance`.
   The tool will then plot the result:
   ![QGC Analyze DISTANCE_SENSOR value](../../assets/qgc/analyze/qgc_analyze_tool_distance_sensor.png)

### QGroundControl MAVLink Console

You can also use the _QGroundControl MAVLink Console_ to observe the `distance_sensor` uORB topic:

```sh
listener distance_sensor 5
```

::: info
The _QGroundControl MAVLink Console_ works when connected to Pixhawk or other NuttX targets, but not the Simulator.
On the Simulator you can run the commands directly in the terminal.
:::

For more information see: [Development > Debugging/Logging > Sensor/Topic Debugging using the Listener Command](../debug/sensor_uorb_topic_debugging.md).

## Simulation

### Gazebo Simulation

Lidar and sonar rangefinders can be used in the [Gazebo](../sim_gazebo_gz/index.md) simulator.
To do this you must start the simulator using a vehicle model that includes the rangefinder.

Downward facing sensors that write to the [DistanceSensor](../msg_docs/DistanceSensor.md) UORB topic can be used to test use cases such as [landing](../flight_modes_mc/land.md) and [terrain following](../flying/terrain_following_holding.md):

- [Quadrotor(x500) with 1D LIDAR (Down-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-down-facing)

  ```sh
  make px4_sitl gz_x500_lidar_down
  ```

Front-facing sensors that write to [ObstacleDistance](../msg_docs/ObstacleDistance.md) can be used to test [Collision Prevention](../computer_vision/collision_prevention.md#gazebo-simulation):

- [Quadrotor(x500) with 2D LIDAR](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-2d-lidar)

  ```sh
  make px4_sitl gz_x500_lidar_2d
  ```

- [Quadrotor(x500) with 1D LIDAR (Front-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-front-facing)

  ```sh
  make px4_sitl gz_x500_lidar_front
  ```

### Gazebo-Classic Simulation

Lidar and sonar rangefinders can be used in the [Gazebo Classic](../sim_gazebo_classic/index.md) simulator.
To do this you must start the simulator using a vehicle model that includes the rangefinder.

The iris optical flow model includes a Lidar rangefinder:

```sh
make px4_sitl gazebo-classic_iris_opt_flow
```

The typhoon_h480 includes a sonar rangefinder:

```sh
make px4_sitl gazebo-classic_typhoon_h480
```

If you need to use a different vehicle you can include the model in its configuration file.
You can see how in the respective Iris and Typhoon configuration files:

- [iris_opt_flow.sdf](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/models/iris_opt_flow/iris_opt_flow.sdf)

  ```xml
    <include>
      <uri>model://lidar</uri>
      <pose>-0.12 0 0 0 3.1415 0</pose>
    </include>
    <joint name="lidar_joint" type="revolute">
      <child>lidar::link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
  ```

- [typhoon_h480.sdf](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/models/typhoon_h480/typhoon_h480.sdf.jinja#L1131-L1145)

  ```xml
    <include>
      <uri>model://sonar</uri>
    </include>
    <joint name="sonar_joint" type="revolute">
      <child>sonar_model::link</child>
      <parent>typhoon_h480::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
  ```
