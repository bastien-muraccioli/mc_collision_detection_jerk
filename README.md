# CollisionDetectionJerk — mc_rtc Global Plugin

A lightweight mc_rtc global plugin for IMU-based collision detection using **jerk estimation**. It computes the jerk (rate of change of acceleration) directly from raw IMU measurements and flags a collision when the signal exceeds an adaptive threshold.

---

## How It Works

At each control tick the plugin:

1. **Reads IMU data** — linear acceleration and angular velocity from a named body sensor.
2. **Estimates jerk** — using the kinematic relation expressed in the **body frame**:
   ```
   jerk_B = s(ω) · a_B + ȧ_B
   ```

   where `s(ω)` is the skew-symmetric matrix of the gyroscope reading `ω`, `a_B` is the accelerometer measurement in the body frame, and `ȧ_B` is its finite-difference derivative. This is equivalent to `R^T ȧ`, i.e. the world-frame jerk rotated back into the body frame.
3. **Applies an adaptive threshold** — a low-pass filter (`LpfThreshold`) tracks the running signal and flags a collision when any axis exceeds `filtered_signal ± offset`.
4. **Sets a datastore flag** — writes `true` to the `"Obstacle detected"` datastore entry so other controllers can react.


---

## Configuration

Add the plugin to your mc_rtc configuration:

```yaml
Plugins:
  - CollisionDetectionJerk

collision_detection_jerk:
  threshold_filtering_base: 0.05          # LPF coefficient (0–1); higher = faster adaptation
  threshold_offset_base: [1.0, 1.0, 1.0] # Per-axis detection band (±) around the filtered signal

# Optional (top-level):
imuBodyName: Accelerometer  # Name of the body sensor (default: "Accelerometer")
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `threshold_filtering_base` | `double` | `0.05` | Low-pass filter coefficient for the adaptive threshold |
| `threshold_offset_base` | `[x, y, z]` | `[1.0, 1.0, 1.0]` | Fixed band added/subtracted from the filtered jerk signal |
| `imuBodyName` | `string` | `"Accelerometer"` | Name of the robot body sensor to read |

---

## Runtime GUI

The plugin exposes a panel under **Plugins → CollisionDetectionJerk**:

| Control | Description |
|---|---|
| `threshold_filtering_base` | Adjust LPF coefficient live |
| `threshold_offset_base` | Adjust per-axis offset live |
| `Axis shown` | Select axis (0=x, 1=y, 2=z) for the live plot |
| `Collision stop` | Enable/disable writing the detection result to the datastore |
| `Verbose` | Print per-axis collision messages to the console |
| `Add plot` | Open a live plot of jerk vs. adaptive thresholds |

---

## Logged Entries

| Key | Type | Description |
|---|---|---|
| `CollisionDetectionJerk_jerk_withoutModel` | `Vector3d` | Estimated jerk (x, y, z) |
| `CollisionDetectionJerk_jerk_withoutModel_norm` | `double` | Euclidean norm of the jerk |

---

## Datastore Interface

| Key | Type | Written by | Description |
|---|---|---|---|
| `"Obstacle detected"` | `bool` | Plugin (`before`) | `true` when a collision is detected on any axis; only written when *Collision stop* is enabled |

The entry is created automatically on `init` if it does not already exist.

---

## Dependencies

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
- [state-observation](https://github.com/jrl-umi3218/state-observation) (`skewSymmetric`)
