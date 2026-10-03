// lhsight (Steam Frame): the camera reader lhsyncd runs in a child process, see lhsight.c
#pragma once

// lines on stdout until XRService or its buffers change; returns the exit status
int lhsight_main(void);
