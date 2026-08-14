# Fly Catcher Controller

ROS 2 Humble controller for the Robotican Fly Catcher assignment.

## Control approach

The node subscribes to the measured fly position and drone odometry, then runs a closed-loop controller at 20 Hz.

1. Estimate the fly velocity from consecutive 10 Hz position measurements.
2. Low-pass filter and magnitude-limit the estimate to reduce measurement noise.
3. Solve a constant-velocity intercept equation to predict a future meeting point.
4. Convert the vector from the drone to that point into world-frame `yaw` and `pitch`.
5. Map a distance-scheduled desired speed to `thrust` in `[0, 100]`.
6. Fall back naturally to direct pursuit when no positive intercept solution exists.
7. Latch the caught state and command zero thrust when the separation is below `0.2 m`.

The controller stops safely when fly or drone measurements become stale. RViz2 markers show the fly, drone, predicted intercept point, and pursuit line.

## Build

Place this package next to the supplied packages inside the workspace `src` directory, then run:

```bash
cd ~/fly_catcher_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the full demo

```bash
ros2 launch fly_catcher_controller demo.launch.py
```

This starts the supplied simulator, the controller, and RViz2.

Run without RViz2:

```bash
ros2 launch fly_catcher_controller demo.launch.py use_rviz:=false
```

Run only the controller against an already running simulator:

```bash
ros2 launch fly_catcher_controller demo.launch.py use_simulator:=false use_rviz:=false
```

## Main topics

- Input: `/fly/position` (`geometry_msgs/msg/PointStamped`)
- Input: `/drone/state` (`nav_msgs/msg/Odometry`)
- Output: `/drone/cmd` (`fly_catcher_interfaces/msg/DroneCmd`)
- Visualization: `/fly_catcher/markers` (`visualization_msgs/msg/MarkerArray`)

## Tuning

Edit `config/controller.yaml`:

- `lead_factor`: amount of prediction. Lower it if the drone leads too aggressively; increase it if it trails the fly.
- `max_prediction_time_s`: limits distant or noisy predictions.
- `velocity_filter_alpha`: higher values react faster; lower values smooth more.
- `chase_speed_max_mps`: maximum commanded speed, capped by the simulator at 3 m/s.
- `chase_speed_min_mps` and `slow_radius_m`: control close-range speed scheduling.
- `fly_timeout_s` and `drone_timeout_s`: stale-data fail-safe thresholds.
- `control_rate_hz`: command publication rate; default is 20 Hz.

## Expected behavior

The default configuration typically catches the supplied fly in a few seconds. The controller logs distance, estimated fly speed, drone speed, and the final catch time.
