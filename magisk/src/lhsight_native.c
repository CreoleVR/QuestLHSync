// lhsight.c as plain C, for the Galaxy XR's reader (gxrsight.c): a translation unit of its own, as lhsight.c declares
// the few libc functions it uses itself (it's written for Frida's CModule, which has no headers). Its state is here.
// Its Quest poller is renamed: an executable's symbols interpose on the libraries', and lhsight.c's poll() took libc's
// place for the QVR client library's network thread (the client hung in its constructor)
#define poll lhsight_poll
#include "lhsight.c"
struct st S;
