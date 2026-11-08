# Galaxy XR (SM-I610) Support

QuestLHSync support for the Samsung Galaxy XR: the headset's own tracking cameras find the SteamVR base stations, and
the PC driver aligns the lighthouse universe to the headset's tracking, as on a Quest.

## Status

**Experimental, working end to end** (Virtual Desktop streaming to SteamVR). Live on 9 Nov (5884fb7 + 886c13e,
seated, one session): acceptable tracking right after the driver started; after moving back and forth in the chair two
or three times, the Galaxy XR controller lined up with the inside edge of a Tundra tracker to the millimetre. Replays
against a tracker on a rigid headset mount: 1.3–3.6 cm off over five sessions with the lens model (3.7–12.1 cm
without), and the lit-room sessions lock on the stations, not on lamps (below).

Known problems:

- **No autostart.** Root is ephemeral (below), and the module's `service.sh` doesn't run after it; `lhsyncd` has to
  be started by hand after every reboot.
- **Only tested seated.** Standing, walking, feet and hips over a long session, and how well a lock repeats between
  SteamVR sessions (levelled from station pairs since 5887d81; it came out 1.9° and 1.0° apart before) are untested.
- **Head and lighthouse disagree on a metre.** With a tracker fixed to the headset, the head covers 0.92-0.99 of the
  tracker's distance, differing by session (five sessions; the base stations' distances stay within 0.5%). The
  headset's own pose (Google's head tracking client) reaches SteamVR unchanged (1.0004), so it isn't the streamer. Which
  of the two is off isn't known. The driver reads the scale from the sightings (`Sync::ScaleStep`), see below.
- **Pose timing is not learned** on this headset (too few sightings per head turn); the driver starts it at 35 ms for a
  Galaxy XR (`kExpoGalaxyXR`, measured against a tracker on a headset mount), not the 18 ms default.

## Acquisition in a lit room

In a lit room the lamps' rays outnumber the stations' about a hundred to one (7 Nov, against a mounted tracker: 5–13
sightings of a station a minute among ~6000 spots). An alignment guessed from random pairs of rays and ranked by the
rays it gathers was a lamp's every time. On a Galaxy XR the stations' rotors lead instead (`Solver::RotorPoints`,
`RotorRank`, `LayoutHolds`):

- the bright rays of the last five minutes, paired within 5° from head positions 0.15 m apart, meet where a static
  light is; the lights vote on a 0.25 m grid
- a light is the station whose rotor its rays' times follow (`RotorZ`: n R², the frequency searched over ±100 ppm, as
  the headset's and the stations' clocks run ~28 ppm apart); two or more named stations give the alignment outright
- every candidate's stations must hold their rays under the layout once refit as named (60% within 1°, 30% within
  INLIER): a sweep's reflection off a glossy surface keeps the rotor's phase, but not the layout
- the frame times have to be exact for this (`"kind": "galaxy_xr_sensor"`); nothing else in it is particular to this
  headset

Replays: a lit session that didn't lock now locks at 63 s, 0.5–1.2 cm off; one where the rotors could name a single
station only stays unlocked (it locked 0.6–0.9 m off before); the others lock as before. Background:
[galaxy_xr_lamp_filter.md](galaxy_xr_lamp_filter.md).

## The lighthouse scale

Galaxy XR frames of three or more base stations: lighthouse devices are placed by `scale * q * p + t`. Every minute
the sightings of the last five minutes read the scale (`Solver::Scale`), and the reading counts when they come from a head
that went at least 1 m (2-98%, all axes together, so a seated session's one stand-up counts) and from three stations: from a seated head they say nothing (0.67-0.87
where a mounted tracker said 0.96). A session's readings are averaged; until its first one, `stations.json` holds the last
session's, and *Force recalibrate* keeps it. Recordings replayed against a tracker on the mount: from 7.7 to 3.3 cm off
(5 Nov, the head over 1.4 m; 0.949 for the tracker's 0.935), from 5.3 to 4.5 (30 Oct; 0.986 for 0.972); seated sessions read
nothing and stay at the scale they had. Run live once, for the last 26 s of a recording (0.950 for the tracker's 0.934); until
a session's first reading it's looked for every 10 s, then every minute. In the log: `lighthouse scale: ...`; in recordings:
`I scale read scale_before span stations n median readings held`.

## Requirements

- Samsung Galaxy XR (SM-I610) on firmware **I610UEU2AZF3**; the October 2026 update closes DFRoot's exploit
- [DFRoot](https://github.com/nickcbaker/DFRoot) 4.7+ and [KernelSU-GXR](https://github.com/4uva4ek/KernelSU-GXR)
  v1.0.16+: root is ephemeral and has to be re-established after every reboot, and `lhsyncd` started again then
  (`su -c 'setsid sh /data/adb/modules/questlhsync/service.sh </dev/null >/dev/null 2>&1 &'`)

## Architecture

| | Quest Pro / 3 | Galaxy XR |
|---|---|---|
| **Camera reader** | `lhsight.js`: a Frida CModule injected into the sensors HAL | `lhsight-gxr`: a process of its own, a QVR camera client (no Frida, nothing injected) |
| **Tracking cameras** | OV7251, 640×480 | S5K931, 640×640 Y8 global shutter, 4: trackingA–D |
| **Short exposures** | in the camera's own cycle | only while a camera client has the **frame pattern** on (below) |
| **Frame times** | when lhsight found the frame (the driver takes the poll lag off) | each frame's own start of exposure + half its exposure (`"kind": "galaxy_xr_sensor"`: exact times) |
| **Calibration** | `/persist/calibration/camera_calibration.json` | Google's OpenPX profile + this unit's records (below) |
| **Root** | Magisk (persistent) | DFRoot + KernelSU (per boot) |

### lhsight-gxr (`magisk/src/gxrsight.c`)

A QVR camera client, as Samsung's controller service (`vendor.samsung.hardware.secxrcontroller@1.0-service`) is one.
The QVR camera API's C wrappers are inline in Qualcomm's header, so it resolves `libqvrcamera_client.qti.so` itself:
`getQvrCameraClientInstance()`'s ops table makes a client, and the exported `QVRCameraClientImpl` /
`QVRCameraDeviceClient` methods do the rest:

- `AttachCameraWithParams("tracking" / "ctrl-tracking", {qvrcamdevice_get_frame_img_disable = "true"})`: the camera
  service refuses `"false"` ("frame merge is force off in config"); `Start()`
- `SetParamNum("qvrcamdevice_enable_frame_pattern", uint8, 1, [1])` on the ctrl-tracking client: the short (144 µs)
  exposures base station sweeps show in. Without it there are none unless the controllers are tracked (the controller
  service sets the same on its own client). Its client goes when lhsight-gxr exits, and the pattern with it.
- per client a reader thread: `GetCurrentFrameNumber`, `GetFrame` (blocking, explicit frame number),
  `FrameToHardwareBuffer` → hwbuffer entry 0: GraphicBuffer of physical camera 0 @+0 and 1 @+16, native handle @+0x70,
  its first fd @+12 → `mmap` (once per dmabuf) → `lhsight.c`'s `scan_frame()` (compiled as plain C:
  `lhsight_native.c`), `ReleaseFrame`
- frame: u32 number @0, u64 start of exposure (CLOCK_BOOTTIME ns) @8, u32 exposure (ns) @16; sent on CLOCK_MONOTONIC
- the reader threads run SCHED_FIFO 2 and never wait on lhsyncd while holding a frame: a held frame holds the camera
  service's capture loop for every camera

The Android linker lets an executable under `/data` (the module) load no vendor library (`/linkerconfig/ld.config.txt`
files `/data` under the `system` namespace); `/data/local/tmp` is `unrestricted`. `lhsyncd` copies `lhsight-gxr`
to `/data/local/tmp/questlhsync/` and runs it from there.

Cost: ~1.5 % of one core for lhsight-gxr, ~0.3 % for lhsyncd; scans ~125 µs per short frame.

### Calibration (`magisk/src/galaxy_xr.c`)

The QVR XML under `/data/vendor/qvr` is an unconfigured default (its rig is ~15° off what the cameras see). The real
calibration is Google's OpenPX profile: this unit's factory one, `/mnt/vendor/efs/device_profile.textproto`, with its
records in `/data/vendor/openpx/calibration/records/FRAME_IMU_0_t_CAMERA_*` (trackingA/B, the same as the factory's on the
one headset compared). `/vendor/etc/calibration_service/device_profile.textproto` is the fallback: it is the same on
every headset and isn't this unit (compared to the factory's on that headset, principal points up to 12 px off,
trackingC/D up to 1.2°).

- `frame_t_camera` quaternions are JPL (the Hamilton matrix transposed); `p` is the camera's place in IMU_0
- IMU_0 has OpenGL axes (x right, y up, z back)
- `distortion_fisheye_spline3` isn't a Kannala–Brandt polynomial. The factory profile's lens goes to the driver as it is
  (`"Lens"`: spline, `distortion_offset`, the `curved_window`), and the driver's rays follow it (`CamCal::UnprojectLens`, the
  model from stearnsbq's `src/prepare.c`). An ideal equidistant fisheye with the same focal lengths is 0.3° off (median), 1°
  (p90). Replayed against a tracker on the mount (five sessions): from 3.7-12.1 cm off to 1.3-3.6 cm, and acquired sooner
  (seated: 227 s for 398). Run live since 9 Nov. Rays past the profile's `max_valid_*_radius` are dropped (~2%).
- the HMD pose's frame is asked of Google's calibration service at each connection: `getLatestTransform("FRAME_IMU_0",
  "SENSOR_THIRD_EYE")` through `libperception_calibration_client_plugin.so`, by `lhsight-gxr --transform` (a vendor
  library, so from the copy in `/data/local/tmp`). The call is from stearnsbq's
  [questlhsync-galaxy-xr](https://github.com/stearnsbq/questlhsync-galaxy-xr) (`src/head_origin.c`, MIT), and its ABI is
  known on firmware I610UEU2AZF3 only. Runs in this module since 5 Nov (constant within a session). The result goes to the driver's log with
  `camera calibration:`, the cameras' own live transforms (a guess at their names) to the headset's logcat.
- `kEye`, the fallback when that can't answer: the HMD pose's origin in IMU_0 measured with a tracker on a rigid headset
  mount, (0.0725, −0.051, 0.106) m, ±2 cm. The display profile's eye points (`lut_frame_t_eye` of `eye-left/right`) put
  the eyes' middle at ~(0.061, −0.024, 0.051): Virtual Desktop's head pose isn't of that point.
- `calibration_override.json` in the module directory, if there, is sent instead (for fitted corrections), and the head's
  frame is still asked and logged. To run on the factory profile and the live head, move it away.
- the cameras' rotation against the HMD pose isn't the same from session to session: against a tracker on the
  headset mount the sightings wanted the override (md5 556be1ef...) turned about the head's z by -0.4 deg (30 Oct
  23:00, old pipeline), +1.2 (31 Oct 21:37), +0.9..1.1 (2 Nov 00:07) and -0.2..-0.3 (2 Nov 00:30, x +0.1..0.2,
  y +0.3..0.5). A fixed correction can't hold: turned by the 00:07 one, 00:30 came out worse (the level 1.0 deg for
  2.4, a head-mounted tracker 12 cm off while the sightings' median miss read 0.28 deg; with the plain override the
  level came out 1.9). The sightings can't see it themselves: a common turn of the cameras and a tilt of the
  lighthouse frame make up for each other. Its source (the system's own online calibration, Virtual Desktop's pose,
  ...) is open

## Building

```
python magisk/build_module.py galaxy_xr
```

`out/QuestLHSync-galaxy-xr-module-<version>.zip`: `lhsyncd` and `lhsight-gxr` (~50 KB, no Frida).
