# Copter Drag Estimation Flight

This script flies a Copter in a specific pattern to measure its bluff
body and momentum drag, then computes and saves the EK3_DRAG_BCOEF_X,
EK3_DRAG_BCOEF_Y and EK3_DRAG_MCOEF parameters used for EKF3 wind
estimation.  The flight pattern is the one recommended by Paul
Riseborough in his 2021 ArduPilot developers conference presentation
(https://youtu.be/xVVtvVuZGQE?t=1423), flown autonomously in GUIDED
mode.

WARNING: on successful completion this script writes the
EK3_DRAG_BCOEF_X, EK3_DRAG_BCOEF_Y and EK3_DRAG_MCOEF parameters,
changing EKF3 wind and airspeed estimation behaviour.  Review the
reported values before flying missions that rely on them.

## Parameters

- DRGE_ANGLE : lean angle (in degrees) used when driving the vehicle
  into the wind to build up airspeed
- DRGE_SPD_MIN : minimum body-axis relative airspeed (in m/s) for a
  sample to contribute to the drag fit
- DRGE_ACC_MAX : filtered horizontal acceleration (in m/s/s) below
  which the vehicle is considered to have stopped accelerating at the
  end of the drive into wind; the wind measurement at terminal drift
  velocity settles to one third of this
- DRGE_DIST_MAX : the test is aborted if the vehicle moves further
  than this (in meters) from the position at which it was started
- DRGE_WND_MIN : minimum wind speed (in m/s) for the test to proceed;
  the test aborts if the measured wind is ever below this

## How To Use

1. Set SCR_ENABLE to 1 and reboot
2. Install this script in the autopilot's SD card's APM/scripts directory
3. Assign RC option 300 (Scripting1) to a spare transmitter switch
   (e.g. RC10_OPTION = 300)
4. Choose a day with steady wind of at least a few metres per second.
   Consistent wind matters more than strong wind; early morning or
   late evening flights avoid thermal turbulence
5. Take off in LOITER and climb to clean air at least 20m above home,
   well away from all obstacles.  Ensure there is downwind space for
   the vehicle to drift to the speed of the wind, and upwind space
   for four drive-and-drift cycles (at least 50m in each direction;
   more in stronger wind)
6. Change to mode GUIDED and move the switch high
7. Be ready to retake control in LOITER if required.  Changing mode,
   lowering the switch, or descending below 20m aborts the test
8. Wait for the script to report the computed coefficients and
   "params saved", followed by "done" when it has returned to the
   start position

## How It Works

1. The vehicle holds a level attitude and drifts until it stops
   accelerating; at terminal drift velocity it is moving with the air
   mass, so the averaged groundspeed is the wind velocity
2. For each vehicle orientation relative to the wind (nose-in,
   right-side-in, tail-in, left-side-in) the vehicle repositions to
   the initial drift's end point, yaws to the test heading, drives
   into the wind at a fixed lean angle until it stops accelerating,
   then levels off and drifts back downwind
3. A multicopter's thrust is along the body Z axis, so the body-frame
   X and Y specific forces measured by the accelerometers are purely
   aerodynamic drag.  During each level drift the vehicle decelerates
   from its maximum airspeed through zero and back up to wind speed,
   and matched low-pass-filtered pairs of body-frame relative
   airspeed and specific force are recorded
4. The samples are fitted per axis, by least squares, to the drag
   model fused by EKF3:

       accel = -sign(V)*(0.5*rho/BCOEF)*V^2 - MCOEF*(rho/1.225)*V

   with an intercept absorbing accelerometer bias.  Nose-in and
   tail-in runs exercise the body X axis, side-in runs the body Y
   axis; the momentum drag coefficient is the average of the two
   axes' fits.  Air density comes from the barometer
5. The wind measurement is refreshed at the end of every drift, when
   the vehicle is again moving with the air mass
6. On a valid fit the coefficients are reported and saved, and the
   vehicle returns to the position at which the test was started
