# Trial plan: how reliable is the safety stop?

Status: **approved by the professor** (2026-11-13). Updated for the final hardware: the
distance sensor is the **HC-SR04 ultrasonic sensor on the Elegoo Mega**. The TFmini-S
LiDAR was dropped.

This turns the truck from an engineering build into a data science experiment. The
hardware already exists (see [PROGRESS.md](PROGRESS.md)). Trials run in the lab, and the
analysis can be done on any laptop from the data file.

## Question

How reliably do the truck's safety stops detect a person, and does using the camera and
the ultrasonic sensor together work better than either one alone?

## Test station

The sensors don't ride on the truck. They sit at a fixed **station** and the truck runs
in place:

- **Sensor station:** the HC-SR04 (on the Mega) and the ESP32-CAM mounted side by side,
  5-10 cm apart, at the same height (waist to chest), both pointing the same way and level.
  Both plug into the Mac by USB.
- **Truck:** on a stand with the wheels off the ground, driven from the dashboard
  (hold the up arrow), so a stop shows as the wheels stopping.
- **Clear zone:** nothing but the test target inside the sensor's view out to about
  6 ft. The HC-SR04 sees a cone about 30° wide and counts a desk edge, wall or the floor
  as "someone close".

## What changes between trials

| Factor | Settings |
|---|---|
| Distance from the sensor station | 1 ft, 3 ft, 5 ft |
| Lighting | Bright (lab lights on), dim (lights off / blinds closed) |
| Person | Standing still, walking across |

3 × 2 × 2 = **12 setups**. Each setup runs **5 times**, for **60 person trials**.

**Plus 20 "no person" trials**, with only an object in front of the station (chair, box,
bag, cup). These measure **false alarms**: the truck stopping when it shouldn't.

**Total: 80 trials**, about 2 class sessions.

Both sensors run on every trial, so each trial gives a camera result and an ultrasonic
result at the same time. "Both together" means the truck stops if either sensor holds.

Expected difference worth testing: the ultrasonic sensor uses sound, so lighting should
not affect it, while the camera may miss people in dim light. It may also stop for
objects (it can't tell a chair from a person), while the camera should not.

## Keep these the same

- The same truck, stand, station spot and camera angle.
- Tape marks on the floor at 1, 3 and 5 ft, measured from the front of the sensor station.
- The same dashboard settings for every trial: stop distance, `CAMERA_HOLD_S` (1 s), and
  both **Safety stop** and **Camera stop** ticked.
- **Stop distance: 183 cm (6 ft).** The default 152 cm is exactly 5 ft, so a person on
  the 5 ft mark would sit right on the edge and trigger at random. 6 ft puts all three
  distances inside the stop zone. Set it with the slider before trial 1 and don't change it.
- **Camera range calibrated at 6 ft** (dashboard: stand on the 6 ft mark facing the camera,
  click "Calibrate"). The camera can't measure distance, so before this it held the truck
  for anyone anywhere in view. Now it holds only for a person whose box in the picture is
  at least as tall as at 6 ft (95% of it). Farther people are still logged in
  `camera_seen_any_distance`. Recalibrate if the camera is moved.
- **Sensor hold 0.5 s** (`SENSOR_HOLD_S`): the HC-SR04 flickers between a reading and
  "no echo" on a person, so the stop keeps holding for 0.5 s after the last close reading.
- The physical remote stays off. Drive only from the dashboard.

## Randomize the order

Don't run all the bright trials first, then all the dim ones. Shuffle the 80 trials into a
random order before starting (a short Python script can make the list). That way
something that drifts during a session, like the battery level or people walking past,
doesn't line up with one setting.

## What to record for each trial

| Column | Example |
|---|---|
| trial | 17 |
| date | 2026-11-13 |
| distance_ft | 3 |
| lighting | dim |
| target | person_walking / person_standing / chair / box / bag / cup |
| camera_detected | yes / no |
| ultrasonic_detected | yes / no |
| ultrasonic_cm | 92 |
| truck_stopped | yes / no |
| time_to_stop_s | 0.4 |
| notes | person partly out of frame |

Save as `data/trials.csv`. Planned: have `server.py` write these rows automatically, so
nothing is copied by hand.

**Time to stop** has two parts:
- **Mac reaction** (from the dashboard log, `/api/events`): from the sensor or camera first
  seeing the target to the transmitter switching to idle.
- **Wheels stopped** (from slow-motion phone video of the wheels): from the target
  arriving to the wheels fully stopping. The wheels are off the ground with no load, so
  they coast longer than they would on the floor; report this as a lab-stand result.

## Analysis (at home)

- **Detection rate** for each sensor (camera, ultrasonic, either) at each distance and lighting.
- **Misses:** a person was there and nothing stopped. This is the dangerous error.
- **False alarms:** the truck stopped for an object.
- **Confusion matrix** for each sensor (person vs no person).
- **Chart:** detection rate vs distance, one line per sensor, split by lighting.
- **Time to stop:** average and spread for each sensor.
- **Distance accuracy:** ultrasonic reading vs the tape mark.
- Compare: does "either sensor" catch people that one sensor alone misses?

## Before the trials

1. ~~Wire the TFmini-S LiDAR~~ (dropped; using the HC-SR04 on the Mega, already working).
2. Build the station and put the truck on its stand (see "Test station").
3. Check the HC-SR04 against a tape measure at 1, 3 and 5 ft.
4. ~~Add trial logging to `server.py`~~ (done 2026-11-13: **Trials** card on the dashboard).
5. ~~Make the shuffled 80-trial list~~ (done: `data/trial_order.csv`, from `python3 make_trials.py`,
   fixed seed).
6. ~~Do 3 practice trials~~ (done 2026-11-13; rows kept in `data/practice_trials.csv`. They
   showed the camera holding for far-away people and the sensor flicker, fixed above).
7. Calibrate the camera range at 6 ft, then start the real trials from trial 1.

## Running a trial (dashboard "Trials" card)

1. The card shows the next unrecorded trial, e.g. "Trial 7: 3 ft · dim · person walking".
   Set the lighting and get the target ready, **outside** the stop zone.
2. Start the phone's slow-motion video on the wheels.
3. Hold **↑** so the truck is driving, then press **T** (or click Start). Recording runs 8 s.
4. Bring the target to its tape mark (walk across it for "person walking"). Keep holding ↑
   until the 8 s are up.
5. The row saves itself to `data/trials.csv`: camera / ultrasonic detected and when,
   ultrasonic cm, truck stopped, which sensor stopped it, and the Mac reaction time.
6. From the video, type **Time to stop** and any notes under "Last saved", and click Save.

Re-running a trial number replaces its row. Times in the row are seconds after Start.
The camera's time is when detection finished on a frame, so it includes the ~12 ms of
detection but not the camera's own frame delay.
