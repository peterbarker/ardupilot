# Blimp Named Value Floats — Tuning Reference

These are real-time values the blimp streams to your ground station (e.g. Mission Planner or QGroundControl) via MAVLink `NAMED_VALUE_FLOAT` messages. They are the main window into what the autopilot is doing while you tune it.

**To enable streaming:** Set the `STREAM_RATE` parameter to a non-zero value (default is 1 Hz; range 0.5–10 Hz). The fin and scaler values are sent at this rate. The Auto mode target values are sent every loop regardless.

---

## Fin demand: FINIR, FINIF, FINID, FINIY

**What they are:** The amount of thrust the controller is asking for in each direction, just before it gets sent to the fins/motors.

| Name    | Direction            |
|---------|----------------------|
| `FINIR` | Right/left           |
| `FINIF` | Forward/back         |
| `FINID` | Up/down              |
| `FINIY` | Yaw (rotation)       |

**Range:** −1.0 to +1.0 (or beyond — see below).

**How to read them:**
- A value of 0 means no demand in that direction.
- ±1.0 is full demand. If you consistently see values at or beyond ±1.0 the controller is asking for more than the fins can give — this is called **saturation**, and usually means gains are too aggressive or the blimp is being asked to move too fast.
- These are logged *before* the `FINS_THR_MAX` clamp, so values outside ±`FINS_THR_MAX` will appear here even though the fins won't actually be driven that hard. This is intentional — it lets you spot over-aggressive tuning.

**What to do if they saturate:** Reduce position or velocity PID gains, reduce `WP_SPEED`.

---

## Loiter output scalers: BSCX, BSCY, BSCZ, BSCYAW (and BSCXN, BSCYN, BSCZN, BSCYAWN)

**What they are:** The blimp's fins can only do so much at once. When the controller asks for simultaneous thrust in two directions that share the same fin (e.g. forward and up on a fishblimp), the combined demand can exceed 100%. The scaler reduces both demands proportionally so the fins are not overloaded.

| Name      | Axis scaled         |
|-----------|---------------------|
| `BSCX`    | X (forward) — smoothed  |
| `BSCY`    | Y (right) — smoothed    |
| `BSCZ`    | Z (up/down) — smoothed  |
| `BSCYAW`  | Yaw — smoothed          |
| `BSCXN`   | X — instantaneous (before smoothing) |
| `BSCYN`   | Y — instantaneous       |
| `BSCZN`   | Z — instantaneous       |
| `BSCYAWN` | Yaw — instantaneous     |

**Range:** 0.0 to 1.0.

**How to read them:**
- **1.0** means no scaling is needed — the fins are not saturated on that axis.
- **Less than 1.0** means the combined output on that axis pair exceeded the maximum and had to be pulled back. A value of 0.5 means the demand was halved.
- The `BSC*` (smoothed) values change slowly; the `BSCN*` (new/instantaneous) values show the raw situation each cycle. Both sets are useful: the instantaneous values show brief saturation events, the smoothed values show the average load.
- How quickly the smoothed values track the instantaneous ones is controlled by `LOIT_SCALER_SPD` (default 0.99 — very slow smoothing; lower values make it respond faster).

**What to do if scalers are consistently below 1.0:** The blimp is regularly saturating its fins. Reduce velocity or position gains, reduce maximum speed (`LOIT_VEL_XY_MAX`, `LOIT_VEL_Z_MAX`), or accept that the blimp cannot perform the demanded manoeuvres at current settings.

---

## Auto mode target position: TarX, TarY

**What they are:** The position (in metres, North/East from the EKF origin) that the autopilot is currently flying towards during an Auto mission.

| Name   | Direction                        |
|--------|----------------------------------|
| `TarX` | North from origin (positive = north) |
| `TarY` | East from origin (positive = east)   |

**How to read them:** Compare these against the blimp's actual position (visible in Mission Planner's HUD or via `LOCAL_POSITION_NED`). If the blimp is far from its target with low fin demand values, the position gains may be too low. If it overshoots badly, they may be too high.

These values are sent every loop (not rate-gated by `STREAM_RATE`) and are only active while the blimp is in **Auto** mode.
