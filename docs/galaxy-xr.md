# Galaxy XR backend compatibility

An experimental external KernelSU backend supplies observations from Galaxy XR's
four outward-facing tracking cameras using the existing daemon protocol. Live
alignment through Virtual Desktop has been confirmed with measured per-device
calibration; generic factory calibration and firmware support are not supplied
by this PC change.

The calibration JSON uses `kind: "galaxy_xr"` and `GxrCameraCalibration` in place
of `CameraCalibration`. The distinct root key ensures older drivers reject the
new format. `Projection.Model: "PinholeXY"` has four coefficients
`[fx, fy, cx, cy]`, preserving the measured horizontal and vertical focal lengths.
Both focal lengths must be positive and all four coefficients finite. Existing
Meta calibration and Steam Frame formats retain their current interpretation.

Enable `driver_questlhsync.anyHmd` for the external backend. An optional
`driver_questlhsync.profile` string isolates state under
`%LOCALAPPDATA%\QuestLHSync\profiles\NAME`; allowed characters are ASCII letters,
digits, hyphens and underscores. Missing or invalid values retain the existing
data directory. This prevents a Galaxy XR trial from inheriting another headset's
station geometry and learned timing. The startup log reports the selected path.

The module is maintained separately. No solver, acquisition cadence, confidence
threshold, station-identity, dashboard or existing-headset behavior is changed.
