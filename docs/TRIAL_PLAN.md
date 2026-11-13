# Trial plan: how reliable is the safety stop?

Status: **draft, waiting for the professor's OK** (pitch planned for 2026-11-06).

This turns the truck from an engineering build into a data science experiment. The
hardware already exists (see [PROGRESS.md](PROGRESS.md)). Trials run in the lab, and the
analysis can be done on any laptop from the data file.

## Question

How reliably do the truck's safety stops detect a person, and does using the camera and
the LiDAR together work better than either one alone?

## What changes between trials

| Factor | Settings |
|---|---|
| Distance from truck | 1 ft, 3 ft, 5 ft |
| Lighting | Bright (lab lights on), dim (lights off / blinds closed) |
| Person | Standing still, walking across |

3 × 2 × 2 = **12 setups**. Each setup runs **5 times**, for **60 person trials**.

**Plus 20 "no person" trials**, with only an object in front of the truck (chair, box,
bag, cup). These measure **false alarms**: the truck stopping when it shouldn't.

**Total: 80 trials**, about 2 class sessions.

Both sensors run on every trial, so each trial gives a camera result and a LiDAR result
at the same time. "Both together" means the truck stops if either sensor holds.

## Keep these the same

- The same truck, the same spot in the lab, and the same camera angle.
- Tape marks on the floor at 1, 3 and 5 ft, measured from the front of the truck.
- The same dashboard settings (hold threshold, `CAMERA_HOLD_S`) for every trial.
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
| date | 2026-11-11 |
| distance_ft | 3 |
| lighting | dim |
| target | person_walking / person_standing / chair / box / bag / cup |
| camera_detected | yes / no |
| lidar_detected | yes / no |
| truck_stopped | yes / no |
| time_to_stop_s | 0.4 |
| notes | person partly out of frame |

Save as `data/trials.csv`. Planned: have `server.py` write these rows automatically, so
nothing is copied by hand.

## Analysis (at home)

- **Detection rate** for each sensor (camera, LiDAR, either) at each distance and lighting.
- **Misses:** a person was there and nothing stopped. This is the dangerous error.
- **False alarms:** the truck stopped for an object.
- **Confusion matrix** for each sensor (person vs no person).
- **Chart:** detection rate vs distance, one line per sensor, split by lighting.
- **Time to stop:** average and spread for each sensor.
- Compare: does "either sensor" catch people that one sensor alone misses?

## Before the trials

1. Wire the TFmini-S LiDAR to the ESP32-CAM (see [NEXT_CLASS.md](NEXT_CLASS.md), section 2).
2. Check the LiDAR against a tape measure at 1, 2 and 3 ft.
3. Add trial logging to `server.py`.
4. Do 3 practice trials to check that everything records, then start the real ones.
