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

Sensor-stamped backends use `kind: "galaxy_xr_sensor"` with the same camera
array. Their frame timestamps must already be in the daemon's CLOCK_MONOTONIC
timebase. This selects the existing sensor-timestamp path: no estimated polling
lag is subtracted, while automatic pose timing remains active. The QVR backend
converts its BOOTTIME sensor stamps to MONOTONIC using a freshly sampled clock
offset, including after suspend. Backends reporting delivery timestamps must
continue to use `galaxy_xr`. Older compatibility drivers reject the new kind
because they do not find a legacy `CameraCalibration` array.

The wire format places pixel centres at `n + 0.5`. A backend that converts blob
coordinates through an optical map must remove that half pixel before lookup
and restore it afterward; the driver already removes it when constructing rays.

Enable `driver_questlhsync.anyHmd` for the external backend. An optional
`driver_questlhsync.profile` string isolates state under
`%LOCALAPPDATA%\QuestLHSync\profiles\NAME`; allowed characters are ASCII letters,
digits, hyphens and underscores. Missing or invalid values retain the existing
data directory. This prevents a Galaxy XR trial from inheriting another headset's
station geometry and learned timing. The startup log reports the selected path.

The module is maintained separately. No solver, acquisition cadence, confidence
threshold, station-identity, dashboard or existing-headset behavior is changed.
