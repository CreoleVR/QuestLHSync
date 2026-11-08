// imu_probe: lists a headset's Android sensors, then prints the raw events of sensor type argv[1] (default 65537, Meta's
// HMD IMU) for argv[2] seconds (default 2), and their mean accelerometer direction. Read-only. docs/TESTING.md
//   <ndk>\toolchains\llvm\prebuilt\windows-x86_64\bin\clang.exe --target=aarch64-linux-android29 -O2 -fPIE -pie ^
//     src\tools\imu_probe.c -o out\imu_probe -landroid
//   adb push out\imu_probe /data/local/tmp/ && adb shell chmod 755 /data/local/tmp/imu_probe
//   adb shell su -c "/data/local/tmp/imu_probe 65537 2"
#include <android/looper.h>
#include <android/sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double now(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

int main(int argc, char **argv) {
  int want = argc > 1 ? atoi(argv[1]) : 65537;
  double dur = argc > 2 ? atof(argv[2]) : 2;
  ASensorManager *sm = ASensorManager_getInstanceForPackage("com.questlhsync.probe");
  if (!sm) { printf("no sensor manager\n"); return 1; }
  ASensorList list;
  int n = ASensorManager_getSensorList(sm, &list);
  const ASensor *pick = NULL;
  for (int i = 0; i < n; i++) {
    const ASensor *s = list[i];
    printf("sensor %d: type %d '%s' vendor '%s' min delay %d us, reporting %d, fifo %d\n", i, ASensor_getType(s),
           ASensor_getStringType(s), ASensor_getVendor(s), ASensor_getMinDelay(s), ASensor_getReportingMode(s),
           ASensor_getFifoMaxEventCount(s));
    if (ASensor_getType(s) == want && !pick) pick = s;
  }
  if (!pick) { printf("no sensor of type %d\n", want); return 1; }
  ALooper *lp = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  ASensorEventQueue *q = ASensorManager_createEventQueue(sm, lp, 1, NULL, NULL);
  int md = ASensor_getMinDelay(pick);
  if (ASensorEventQueue_registerSensor(q, pick, md > 0 ? md : 1000, 0) < 0 &&
      ASensorEventQueue_enableSensor(q, pick) < 0) {
    printf("can't enable it\n");
    return 1;
  }
  double t0 = now();
  int got = 0;
  double sx = 0, sy = 0, sz = 0;
  while (now() - t0 < dur) {
    ALooper_pollOnce(100, NULL, NULL, NULL);
    ASensorEvent ev[16];
    ssize_t k;
    while ((k = ASensorEventQueue_getEvents(q, ev, 16)) > 0) {
      for (ssize_t i = 0; i < k; i++, got++) {
        { double ax = ev[i].data[1], ay = ev[i].data[2], az = ev[i].data[3], m = __builtin_sqrt(ax*ax+ay*ay+az*az); if (m > 8.3 && m < 11.3) { sx += ax/m; sy += ay/m; sz += az/m; } }
        if (got < 12 || got % 200 == 0) {
          printf("event %d t %lld type %d:", got, (long long)ev[i].timestamp, ev[i].type);
          for (int j = 0; j < 16; j++) printf(" %.5g", ev[i].data[j]);
          printf("\n");
        }
      }
    }
  }
  printf("%d events in %.1f s; mean unit accel (IMU axes) %.4f %.4f %.4f\n", got, dur, sx / got, sy / got, sz / got);
  ASensorEventQueue_disableSensor(q, pick);
  ASensorManager_destroyEventQueue(sm, q);
  return 0;
}
