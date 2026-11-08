// peers_test: prints what a headset's lhsyncd puts in its discovery answer as peers= (the machines it has connections
// with: the PC that streams it among them), from lhsyncd.c itself, without stopping the lhsyncd that runs. Quest:
//   <ndk>\toolchains\llvm\prebuilt\windows-x86_64\bin\clang.exe --target=aarch64-linux-android29 -O2 -Wall -Wextra ^
//     -fPIE -pie -I src\headset src\tools\peers_test.c -o out\peers_test -llog
//   adb push out\peers_test /data/local/tmp/ && adb shell chmod 755 /data/local/tmp/peers_test
//   adb shell su -c /data/local/tmp/peers_test     ->  peers=192.168.1.88  (adb and ssh never show)
#include <stdio.h>
#include <sys/types.h>
#define main lhsyncd_main
#include "lhsyncd.c"
#undef main
const char *const hs_capture_name = "test";
void hs_log(const char *line) { fprintf(stderr, "%s\n", line); }
void hs_init(int argc, char **argv) { (void)argc; (void)argv; }
void hs_device_info(char *s, char *m, char *f, size_t n) { (void)s; (void)m; (void)f; (void)n; }
int hs_calibration(char **d, size_t *l, char *nm, size_t n) { (void)d; (void)l; (void)nm; (void)n; return -1; }
pid_t hs_capture_start(int fd, char *msg, size_t n) { (void)fd; (void)msg; (void)n; return -1; }
int main(void) {
  printf("peers=%s\n", peers());
  return 0;
}
