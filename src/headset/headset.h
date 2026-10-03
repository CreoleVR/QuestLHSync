// What lhsyncd's shared core (lhsyncd.c) needs from a headset: the Quest Pro (magisk/src/quest.c) or the Steam Frame
// (frame/src/frame.c). One of them is linked with the core.
#pragma once
#include <stddef.h>
#include <sys/types.h>

#ifndef MODULE_VERSION
#define MODULE_VERSION "dev"
#endif

// what stray output of the camera reader is labelled with on the PC ("E <name>: ...")
extern const char *const hs_capture_name;

void hs_log(const char *line);  // one line to the headset's log
// after the ports are bound (this is the only lhsyncd running): argv as lhsyncd got it
void hs_init(int argc, char **argv);
// serial, model and firmware, n bytes each; empty fields are filled in by the core
void hs_device_info(char *serial, char *model, char *fw, size_t n);
// the camera calibration for a new client: *data malloc'd (the caller frees it) and its name for the "C" line.
// Returns -1 with a message for the client in name ("E ..." without the "E ") when there's none.
int hs_calibration(char **data, size_t *len, char *name, size_t n);
// starts the camera reader with its stdout and stderr on out_fd (the caller closes its copy). Returns its pid, with what
// to tell the clients in msg ("I lhsyncd: <msg>"); 0 when it can't start yet, msg says why ("E <msg>"), tried again in
// 5 s; -1 when it couldn't be started at all.
pid_t hs_capture_start(int out_fd, char *msg, size_t n);
