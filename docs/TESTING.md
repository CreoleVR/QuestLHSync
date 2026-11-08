# Testing QuestLHSync

There is no unit-test suite. The solver is checked by replaying session recordings offline, the headset link
against the real network, and the rest by hand in the headset. This page lists the tools, the procedures, and the
results to expect from the reference recordings.

The reference recordings and the camera calibration they need live outside the repository (34 MB), in
`D:\QuestLHSync\testdata\` on the machine they were made on. Its `README.md` describes the room. Copy that folder
to test on another machine.

## Tools

`build_replay.bat` builds them all into `out\`.

| Tool | What it does |
|---|---|
| `qlhs_replay` | Runs a recording through the driver's `Sync` as if live: logs, the fit each second (`--out`), every sighting ray (`--rays`) |
| `qlhs_nettest` | The driver's network link and solver against `src\tools\fake_lhsyncd.py` (a recording served over the headset protocol) |
| `qlhs_linktest` | The driver's `HeadsetLink` alone against the real network: which headset it takes and why, and Switch headset |
| `qlhs_stat` | The running driver's status (the dashboard's shared memory) as text, and the dashboard's buttons (`qlhs_stat cmd <n>`) |
| `src\tools\fitshare.py` | Share of a replay spent on the right fit, and on known wrong ones |
| `src\tools\lights.py` | Where the bright dots in a recording are and which station's rotor each follows: stations, reflections, lights |
| `src\tools\peers_test.c` | Headset side: what `lhsyncd` reports as `peers=` (built with the NDK, run over adb) |
| `src\tools\getcalib.py` | Saves a headset's camera calibration for replays |

`qlhs_stat cmd` numbers: 2 pause, 3 resume, 4 record on, 5 record off, 6 force recalibrate, 7 switch headset.
Recordings go to `%LOCALAPPDATA%\QuestLHSync\recordings\`. The camera calibration a replay needs is what the headset
sends after its hello on TCP 47280 (`C <len> <name>` then the bytes). Save it once per headset, while it's awake:
`python -I src\tools\getcalib.py <headset ip> calib.json`.

## 1. Solver regression: replay the reference recordings

```
set TD=D:\QuestLHSync\testdata
set CAL=%TD%\calib-questpro-230YC01DCM00CR.json
mkdir scratch\3st & copy %TD%\stations\3st-stations.json scratch\3st\stations.json
out\qlhs_replay.exe %TD%\recordings\3st-lightson-25min-lamp.txt --calib %CAL% --dir scratch\3st --pings --out scratch\3st.out > scratch\3st.log
python -I src\tools\fitshare.py scratch\3st.out -73 -59 +107
```

Use a fresh `--dir` each run: it holds `stations.json` and the state the replay writes. Expected results (build of
2026-11-07):

| Recording | Stations | `fitshare` arguments | Right fit | Notes |
|---|---|---|---|---|
| `3st-lightson-25min-lamp.txt` | `3st-stations.json` | `-73 -59 +107` | 94.9% (lamp fit 5.0%, mirror 0.1%) | Before the light checks: 64.2% right, 35.7% on the lamp |
| `4st-lightson-2min.txt` | `4st-stations.json` | `-71.4` | 100%, yaw spread 0.07 deg | |
| `4st-dark-2min.txt` | `4st-stations.json` | `-71.4` | 100%, yaw spread 0.16 deg | Room lights off |

In the three-station log, expect these lines, and no station ever taken for a light:

- 214 s: `channel check: where the fit puts LHB-0F5A66EB, at [-2.44 3.36 1.25], ... follow no base station's rotor`
  (the fit on the lamp, caught), then 223 s: `acquired: yaw -73.16`.
- 311 s and 866 s: `light at [...]` for two lamps.

A change to `sync.cpp` that lowers the right-fit share or marks a station as a light is a regression.

### Recenter

A recenter moves the headset's space under the head between two HMD poses. The driver measures the move from the
poses either side (turn, tilt and shift), keeps it as `Sync::space_`, takes later poses back through it and sends
the transform out through it: the solver's own fit doesn't move at all (2026-11-08; before, a break dropped every
sighting and acquired from scratch, which live landed on wrong fits). The Quest's horizon adjust, streamed by Steam
Link, makes the head's whole orientation the new level: the space tips by the head's nod (33.6 deg in the recording).

- **Real, tipped:** `otherpc-4st-steamlink-tipped-recenter.txt` (other PC, headset .126, `otherpc-4st-stations.json`,
  `calib-questpro-230YC01F0900C2.json`). Expect at 119 s
  `HMD pose break (0.01 s, 22 cm, 83.0 deg): the headset space turned -80.5 deg, tipped 33.6 deg and moved 68 cm (a recenter?): the alignment goes with it`,
  and `fitshare ... 131.4` at 100%. The build before went 70 s unlocked, then took a wrong fit.
- **Synthetic:** any recording becomes a recenter test by transforming its `P 0` lines (a 3x4 matrix: rotation rows
  with the position in the 4th column) after some time: R' = Ry(a) R, p' = Ry(a) p + t. The 25-minute recording
  turned +60 deg and moved (0.5, 0, -0.3) m at 400 s logs `turned +59.9 deg, tipped 0.1 deg and moved 58 cm` and
  replays exactly as without it (`fitshare ... -73`: 94.9%).

### Tipped from the start

When SteamVR starts with the headset's space already tipped (a horizon adjust before), no pose break shows it.
`Solver::FindTilt` measures it from the stations alone, every 15 s until it has once (then every 60 s): each station's
place where its sightings cross, named by its rotor; Horn's rotation from SteamVR's places onto them; the tip of the
vertical. Three named stations agreeing within 30 cm, tipped 5 deg or more, are taken back into `Sync::space_`.

Test: tip every `P 0` line of a recording from the start (R' = Tip R, p' = Tip p; 25 deg about x then 10 deg about z
is 26.8 deg). `4st-dark-2min.txt` tipped so logs at 110 s
`the headset's space is tipped 27.6 deg against the base stations (4 stations (...) within 6 cm): taken back, acquiring`,
then acquires with 139 sightings; `4st-lightson-2min.txt` at 120 s (26.1 deg). The untipped recordings measure 0.3-1.4
deg (SteamVR's own room) and never act; their right-fit shares stay as in the table above. Recordings carry each look
as `I tiltlook <found> <tilt> <places and rotor scores>`.

### The headset's accelerometer

On a Quest Pro, `lhsyncd` reads Meta's HMD IMU (Android sensor type 65537, `oculus.sensor.hmd_imu.1`: data[0]
temperature, data[1..3] accelerometer m/s^2, about 1000 Hz, timestamps in microseconds on the IMU's own clock, which
the module ties to CLOCK_MONOTONIC itself) and sends `G <t_us> <ax> <ay> <az> <n>` 20 times a second. The PC turns each
through the calibration's `ImuCalibration` (DeviceFromImu and the accelerometer's rectification: the device frame is
the head frame, as for the cameras) and the head's orientation into up in the alignment's space (`Sync::GravityStep`,
5 s windows; two running that agree within 1 deg on 5 deg or more tip the space back, as `FindTilt` does).

- `4st-tipped-start-imu.txt` (this PC, a horizon adjust before SteamVR started, `4st-stations-1108.json`): live, the
  base stations found the tip at about 70 s (41.9 deg). Offline, its G lines say 41.7 deg (30 s windows 41.5-42.1,
  insensitive to 0-100 ms of timing). Replayed now: `the headset's space is tipped 42.1 deg (the headset's
  accelerometer, 70 samples)` at 21 s, locked at 22 s on the same fit (yaw +153.4), then up within 0.7-1.4 deg of the
  vertical. The G lines in it have broken times (the first module read the IMU clock as ns); the driver falls back to
  their arrival times.
- Probe a headset's IMU by hand: `src\tools\imu_probe.c` (build and run lines in its header); `dumpsys sensorservice`
  lists the sensor.

### Level restarts

The cameras' level reading moves the level only when it agrees with what came before; a reading 0.6 deg or more
(`kLevelJump`) off says the level moved. Since 2026-11-08 such a move is taken only when the next reading agrees with
it (within 0.3 deg): single readings off by 1-1.6 deg (another set of stations in view) tipped the frame on their
own, about 5 cm at a worn tracker. `otherpc-4st-level-restarts.txt` (other PC) had three restarts in 51 s live;
replayed now, none, and the fit 100% right.

## 2. Lights and stations by hand

```
out\qlhs_replay.exe <recording> --calib %CAL% --dir scratch\x --pings --rays scratch\rays.txt
python -I src\tools\lights.py scratch\rays.txt <recording> [last_seconds]
```

Each place where sightings cross is listed with its rotor scores per channel. Stations score 0.01-0.25 on their own
channel; lights stay under 0.004 on all. The script scores by sighting times only, so it is a cross-check: the
driver's `SeenScores` (frames in view, seen or not) is what decides. A still head gives no depth: walk about.

In `4st-dark-2min.txt` it finds all four stations on their own channels, ch4 at [-2.67 3.40 0.31], which is
where SteamVR's map has it. With the room lights on, ch4's dots were too faint to stand out and a lamp-lit spot
30 cm away looked like it: test light handling with the lights on, and station placement with them off.

## 3. Light checks: what the thresholds rest on

`sync.h`: `kLightScore = 0.004`, `kRotorScore = 0.008`, `kLightVotes = 3`, `kLightRays = 200`, `kLightGuard = 0.5`.

- In the 25-minute recording, surveys with 200+ sightings scored lights 0.000-0.003 and stations 0.005-0.15 (ch4,
  mostly hidden, at the low end). Live, ch4 once scored under 0.004 three surveys running and was marked a light,
  and a lamp once scored 0.004. Hence the band 0.004-0.008 that changes nothing, the guard around fitted station
  places, and the rule that a place that followed a rotor (0.008 or more) is never a light.
- `CheckIdentity`'s check (a fitted station place that follows no rotor while another follows its own, three checks
  running) is what takes the fit off a lamp. The survey only finds lights away from fitted stations.
- When tuning, check both: the right-fit share on the reference recordings, and that no station place shows up in
  a `light at` line.

## 4. Headset choice (two headsets of a kind)

On the PC that streams a headset, with the streamer connected:

```
copy %LOCALAPPDATA%\QuestLHSync\headset.txt scratch\headset.txt
out\qlhs_linktest.exe scratch\state scratch\headset.txt oculus_virtualdesktop 60 30
```

Expect `connected (streams with this PC)` to the streamed headset within a second, even when `headset.txt`
remembers the other one, and `headsets=2`. At 30 s the tool presses Switch headset: it moves to the other headset,
logs `chosen with Switch headset`, and stays there (`why="chosen on this page"`).

What decides it:

- **This PC's TCP table:** a streamer's connection to the headset (Virtual Desktop holds TCP 38810-38840).
  `Get-NetTCPConnection -State Established` shows it.
- **The headset's peers:** run `peers_test` on each headset (see `src\tools\peers_test.c`). It should list only the
  PC that streams it. adb, ssh, KDE Connect, Chromecast and Wi-Fi Direct (controllers) must not appear.
- **Discovery without broadcast:** many Wi-Fi networks drop broadcasts to a headset. The PC asks remembered
  addresses, hosts and its streamer's peer straight. To check a headset answers:

  ```
  python -c "import socket;s=socket.socket(2,2);s.settimeout(2);s.sendto(b'QLHS?',('<ip>',47281));print(s.recv(512))"
  ```

  A module from 1.29 on answers with `... peers=<ip,..|->`.

In the live driver log: `Quest Pro at <ip>: connected (<why>)`, and when it corrects itself: `... isn't SteamVR's
headset: ... streams with this PC, switching to it` (seen live: from .126 to .14 within ten seconds).

## 5. Station models drawn where they are

The driver re-sends each base station's last pose when the alignment moves (`ResendStations`, `driver_main.cpp`).
Base stations send their poses rarely, so before this fix a model kept wherever the alignment was when its last
pose went out. Test in the headset: Force recalibrate (`qlhs_stat cmd 6`), let it lock, and check that every
station model and the controllers sit where the real ones are. This can't be replayed (it needs SteamVR).

## 6. Installer

Run `out\release-<ver>\QuestLHSync-steamvr-installer.exe` with its zip beside it. It must show "Package next to the
installer ... (downloads are off)" and open no network connection:

```
Get-NetTCPConnection -OwningProcess (Get-Process QuestLHSync-steamvr-installer).Id
```

That should list nothing. With Downloads switched on (kept in `%LOCALAPPDATA%\QuestLHSync\installer.json`), it
checks GitHub and offers a newer release. Without a zip beside it, it downloads as before.

`release.py` refuses with "is in use (is its installer open?)" while the installer from that folder runs, and leaves
the folder as it was.

## 7. Dashboard page

```
driver\questlhsync\bin\win64\QuestLHSync.exe --preview scratch\two.png two
driver\questlhsync\bin\win64\QuestLHSync.exe --preview scratch\unsure.png unsure
```

`two` shows the Switch headset button, "2 headsets answer" and "streams with this PC" by HEADSET. `unsure` shows
"can't tell which one is SteamVR's" in amber.

## Live checks

`out\qlhs_stat.exe 120 10` prints the state every 10 s for two minutes: the headset and why it was chosen, the fit,
and each station's support and when the cameras last saw it. To record a test: `qlhs_stat cmd 4`, walk about
turning so each station passes to your sides (the Quest Pro's side cameras see them; the front pair isn't scanned,
so a station dead ahead is barely seen), then `qlhs_stat cmd 5`.

## Open items

- **Small recenters:** a pose jump counts as a break only past 5 deg or 5 cm (plus what the head's own speed
  explains). A smaller recenter isn't carried over, and the alignment follows it only as fast as the corrections slew
  (5 mm/s with the head still).
- **A tipped space at start:** with the module's G lines (Quest Pro), the accelerometer finds it within about 20 s,
  with any number of base stations. Without them (Steam Frame, Galaxy XR, an older module) only `FindTilt` does, once
  three stations have been named by their rotors from several head positions: one to two minutes of looking about.
  The Quest 3/3S IMU sensor hasn't been checked (it may not be type 65537).

- **Level restarts:** with four stations the cameras' level reading jumped 0.6-0.7 deg three times in two minutes
  (`the level moved, its mean starts over`), each time SteamVR re-placed a station. If trackers shift by a few cm
  each time, look at `kLevelJump` (0.6 deg) and SteamVR's re-tilts.
- **Virtual Desktop pose timing:** learned values ranged from 26 to 52 ms between sessions on the same setup.
- **Light check thresholds** rest on one room. Replay other rooms' recordings before trusting them widely.
  1.0 base stations have no channels, so neither light check runs for them.
- **Unverified:** how often SteamVR sends base station poses (the redraw fix was confirmed by eye, not measured).
- **Releases:** `QLHS_RELEASE` is still 1.28. A headset module with `peers=` should ship as 1.29, and the README's
  "module before 1.29" assumes that.
