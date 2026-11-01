# Galaxy XR: a lamp filter for acquisition

## Implemented: the rotor-phase veto (1 Nov 2026)

What went in is the second stage below, not the occupancy filter: `Solver::RotorVeto` / `RotorPhase` in
`src/driver/sync.cpp`, Galaxy XR only. Under each acquisition candidate, a station's sightings (within `INLIER`, all
bright rays) are tested for phase at the station's own rotor period, on the frames' sensor times: coherence (mean
resultant length) `R`, with `n R^2 >= 4.6` "follows", `n >= 12` and `R < 0.3` "doesn't", else "can't tell". A new
candidate counts only stations that follow; the current fit (`ThinnedScore`) loses only those that don't.

Measured on the 30-31 Oct recordings, with the truth from a lighthouse device on a headset mount:

- The stations' sightings: `R` 0.41-0.63 in every minute with 12 or more. The lamps three wrong fits rested on:
  0.04-0.22. ED72 (channel 2) peaks at 50.154 Hz against its 50.157.
- The lamps are periodic, but not at 100.000 Hz (coherence 0.00-0.15 there): half at ~100.24 Hz, half at 7x the
  frame rate (104.95 Hz, a sampling artefact of steady lights). Only `tau`, not `tau/2`, is tested: channel 1's
  `tau/2` is 100.10 Hz, next to the lamps (one reached 0.31 there).
- Blob size and occupancy separated these recordings too (dots 3-18 px, lamps 20-60; occupancy as in the tables
  below), but both depend on where the head is and looks, and with the veto in they changed no outcome: left out.
- Replays of all ten recordings: no wrong acquisition (five before, 3-8 m off); 30 Oct's 5.8 min run acquires as before
  (84 s) and ends within 0.06 m; 29 Oct 00:09 (82 s, the old pipeline) no longer acquires in its 80 s. 31 Oct 21:37
  stays unlocked: its minute holds 47 station sightings, nearly all of ED72, too few for any fit.

## The problem

In a lit room a Galaxy XR acquires a wrong alignment from a seated head, and with the room lights off it acquires the
right one. A recording (31 Oct 2026, a tracker on a rigid headset mount for the truth, *Force recalibrate* pressed):

- `force recalibrate` at 20:54:24, 20:55:42 and 20:58:54; each time `acquired: yaw -97.7 deg t [0.20 2.80 -1.09]`
  with `support 12`–`14` within 16–47 s. The truth was about yaw −140°, t [−1.99 2.87 −1.99].
- `qlhs_replay` lands on the same alignment with three different camera calibrations: it isn't the calibration.
- The wrong alignment puts base station ED72 1.6 m from the head and 48° up, where a ceiling lamp is; ~400 bright
  spots agree with it to 1°, at a median residual of 0.33°.

Why it gets in: a lamp is in nearly every short exposure, a base station only when one of its sweeps happens to cross
the 144 µs exposure. `Solver::Thin` collapses repeats to one ray per 5 cm head cell and 2° direction, so from a still
head both a lamp and a station come down to a handful of rays, and a wrong alignment that lines two stations up with
two lamps reaches `ACQ` sooner than the right one, whose rays come in slowly. After *Force recalibrate*
(`acq_forced_`) half of `ACQ` already wins (`Solver::Step`, `enough`).

## Measured: occupancy separates steady lights, not the lit room's lamps

Occupancy of a ray: of the camera's short frames in the ±5 s around it, the share that had a saturated spot within 1°
of the same **world** direction (a lamp stays put in the world as the head turns). On a 5.8 min mount recording
(30 Oct 2026, room lighting not noted, the truth from the tracker), with sightings counted as a station's within 1°
of the truth:

| | n | occupancy 5 / 25 / 50 / 75 / 95 % |
|---|---|---|
| station sightings | 407 | 0.7 / 1.3 / **2.0** / 2.7 / 4.7 % |
| everything else | 14 543 | 0.7 / 6.7 / **44** / 90 / 99 % |

| threshold | station sightings kept | other spots dropped |
|---|---|---|
| 5 % | 97 % | 78 % |
| 10 % | 100 % | 71 % |
| 20 % | 100 % | 63 % |
| 50 % | 100 % | 47 % |

Expected for a station: two sweep planes per rotor turn (~20 ms) cross a direction for a few µs each, so a 144 µs
exposure catches one in ~1.4 % of frames, in bursts as the rotor's phase drifts against the 15/s short frames per
camera.

**But in the lit room it isn't enough.** In the 31 Oct recording (lights on), the 416 spots the wrong alignment rests
on (within 1° of its stations) have a median occupancy of **10 %** (5–95 %: 3–30 %), not the steady lights' 44 %:

| threshold | spots of the lamp alignment dropped |
|---|---|
| 5 % | 86 % |
| 10 % | 54 % |
| 20 % | 14 % |

So these lamps are seen in only part of the short exposures; a guess, not checked: they flicker with the mains, and a
144 µs exposure catches their bright phase now and then. At 5 % (97 % of the station sightings kept) ~58 of the 416
would remain, and whether that keeps the wrong alignment under `ACQ` after `Thin` is untested. A run with the room
fully lit and the tracker on the mount, with enough head movement for a usable truth, is still missing.

## Design

1. **Occupancy, in `Sync`** (it sees every `F` line, those without blobs too, so the denominator is exact): per camera
   a ring of its short frames of the last ~10 s, each with its bright rays' world directions (head pose at the frame's
   time, as `Solver::Add` gets them). For a new ray: the share of that camera's short frames within ±5 s with a
   bright ray within 1° of its direction. The centre of the window lies 5 s back, so a ray is judged ~5 s late, or on
   the window so far (one-sided) while acquiring.
2. **Lamps out of acquisition only:** rays over a threshold (5 % from the numbers above; to be checked against a
   fully lit run) go to `Solver` flagged (like `B` for dim ones) and
   `Acquire` / `Hypotheses` leave them out. Tracking a fit (`Fit`, `Support` of the current alignment) can keep them;
   a lamp far from a station doesn't count there anyway.
3. **Count them:** a `lamps` number next to `bright spots, used` in the acquiring log line and the overlay's
   *Bright spots* row, so a lit room shows.
4. **Second stage, likely needed for flickering lamps:** check a candidate's per-station sightings against the
   station's rotor period with the existing `SeenScores` (station identity): a lamp assigned to a station shouldn't
   show periodicity at τ. Needs `kMinN` 60 in-view frames and `kMinSeen` 20 sightings per station, i.e. tens of
   seconds seated: as a veto on a candidate, not a gate on acquiring. A caution, unchecked: on a 50 Hz grid a lamp
   flickers at 100 Hz, and a channel 1 station turns at 50.05 Hz with two sweeps a turn, so both repeat every ~10 ms
   and part only by 0.1 %: telling them apart takes ~20 s of sightings, and the fake periods `SeenScores` compares
   against must not sit on the mains' harmonics.
5. **`acq_forced_`:** halving `ACQ` after *Force recalibrate* lets a lamp fit in at 6. Keep it for the Quest; for a
   Galaxy XR (exact times, few sightings) require the full `ACQ`, or apply it only once the occupancy filter is in.

## Testing

`qlhs_replay` with the round-trip clock (`--pings`) on recordings; the ray dump (`--rays`) and the scripts that
measured the table above (occupancy against the truth from a mounted tracker) are worth turning into a test:

- 31 Oct, lights on, *Force recalibrate*: must **not** acquire yaw −97.7 (or anything >20° off the truth); acquiring on
  until the stations are seen is fine.
- Still to record: the room fully lit, the tracker on the mount, *Force recalibrate*, a few minutes seated and then
  walking about (the walking gives the truth), to set the threshold and check the second stage.
- 30 Oct, 5.8 min with the mount and walking: must still acquire within ~1 min, ending within 0.5° / 5 cm of the truth.
- A seated recording with the lights off: acquisition time no worse than now.
