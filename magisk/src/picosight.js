'use strict';
// picosight: lhsight for the PICO 4 (see picosight.c). Runs inside pvrtrackingservice: a read-only hook on the
// function every camera frame passes through, the bright spots found on Frida's thread. Lines go to stdout via
// console.log:  F ... (frames)  T ... (heartbeat)  I/W ... (notes)
// QuestLHSync: lhsyncd runs this with frida-inject while a PC listens, and stops it after (MAX_MS 0: no limit)
var MAX_MS = parseInt("@DUR@");
// The cameras to scan, "id,id,..." (lhsyncd fills it in from the calibration: the ones that look up, where the base
// stations hang). Left unfilled (a hand run): all four.
var CAMS_SPEC = "@CAMS@";
var SCAN = (CAMS_SPEC.charAt(0) === "@" ? "0,1,2,3" : CAMS_SPEC).split(",").filter(function (e) { return e.length; })
  .map(function (v) { return parseInt(v, 10); }).filter(function (c) { return c >= 0 && c < 4; });
var SYM = "_ZN3pvr16AlgorithmManager15updateSlamImageENSt3__16vectorINS_16alg_frame_item_tENS1_9allocatorIS3_EEEEbi";
var l = Process.getModuleByName("libc.so"), g = function (n) { return l.getExportByName(n); };
var usleep = new NativeFunction(g("usleep"), "int", ["uint"]);
var clock_gettime = new NativeFunction(g("clock_gettime"), "int", ["int", "pointer"]);
// Keep every Memory.alloc in a global: JS GC frees unreferenced allocations under the C code.
var stateBuf = Memory.alloc(131072);
// mrs x0, cntvct_el0; ret / mrs x0, cntfrq_el0; ret: the arch counter PICO OS stamps the frames with
var counterCode = Memory.alloc(Process.pageSize);
Memory.patchCode(counterCode, 16, function (p) {
  p.writeU32(0xd53be040); p.add(4).writeU32(0xd65f03c0); p.add(8).writeU32(0xd53be000); p.add(12).writeU32(0xd65f03c0);
});
var cm = new CModule(`@CSRC@`, { clock_gettime: g("clock_gettime"), memcpy: g("memcpy"), snprintf: g("snprintf"), S: stateBuf,
                                 cntvct: counterCode, cntfrq: counterCode.add(8) });
var F = function (n, r, a) { return new NativeFunction(cm[n], r, a); };
var state_size = F("state_size", "int", []);
if (state_size() > 131072) throw new Error("picosight state " + state_size() + " bytes, 131072 allocated");
var setup = F("setup", "void", ["pointer", "pointer", "pointer", "pointer", "int"]);
var set_scan = F("set_scan", "void", ["int", "int"]), start = F("start", "void", []), stop = F("stop", "void", []);
var scan = F("scan", "int", []), drain = F("drain", "int", ["pointer", "int"]);
var n_drop = F("n_drop", "uint", []), scan_ns = F("scan_ns", "uint64", []), n_scan = F("n_scan", "uint", []);
var n_call = F("n_call", "uint", ["int"]), n_qdrop = F("n_qdrop", "uint", []), n_lap = F("n_lap", "uint", []);
var n_badclk = F("n_badclk", "uint", []), bad_fmt = F("bad_fmt", "uint", []), clock_kind = F("clock_kind", "int", []);
var take_age_max = F("take_age_max", "int64", []), item0 = F("item0", "int", ["pointer"]);
var NCAM = 4, NSLOT = 4, MAXPIX = 640 * 512;
var ringBuf = Memory.alloc(NCAM * NSLOT * MAXPIX), cellBuf = Memory.alloc(2048 * 2), labBuf = Memory.alloc(2048 * 4);
var outBuf = Memory.alloc(1 << 20), drainBuf = Memory.alloc(1 << 16), tsBuf = Memory.alloc(16), itemBuf = Memory.alloc(72);
setup(ringBuf, cellBuf, labBuf, outBuf, 64);
SCAN.forEach(function (c) { set_scan(c, 1); });
var MAX_GAIN = 0;  // no limit: see set_max_gain in picosight.c
F("set_max_gain", "void", ["int"])(MAX_GAIN);
var n_gain = F("n_gain", "uint", []);

var mod = Process.findModuleByName("libpvrtrackingservice.so");
var target = mod ? (mod.findExportByName(SYM) || mod.findSymbolByName(SYM)) : null;
if (!target) {
  console.log("W no camera frames: " + (mod ? "libpvrtrackingservice.so has no " + SYM : "libpvrtrackingservice.so isn't loaded") +
              " (a PICO OS build picosight doesn't know)");
} else {
  Interceptor.attach(target, { onEnter: cm.on_enter });
  start();
  console.log("I hooked AlgorithmManager::updateSlamImage at " + target + "; scanning cameras " + SCAN.join(",") +
              (MAX_GAIN ? ", frames up to gain " + MAX_GAIN : ""));
}

function mono_us() { clock_gettime(1, tsBuf); return tsBuf.readS64().toNumber() * 1e6 + Math.floor(tsBuf.add(8).readS64().toNumber() / 1000); }
var pending = "";
function flush() {
  for (;;) {
    var n = drain(drainBuf, 65536);
    if (n <= 0) break;
    pending += drainBuf.readUtf8String(n);
  }
  var cut = pending.lastIndexOf("\n");
  if (cut >= 0) { console.log(pending.slice(0, cut)); pending = pending.slice(cut + 1); }
}
var CLOCKS = ["unknown", "boottime", "monotonic", "the arch counter"];
var t0 = Date.now(), lastBeat = 0, said = {}, lastCalls = [0, 0], quietBeats = 0;
function note(key, line) { if (!said[key]) { said[key] = 1; console.log(line); } }
function step() {
  var budget = Date.now() + 20;
  while (Date.now() < budget) {
    var got = scan();
    usleep(got ? 1000 : 4000);
  }
  flush();
  if (Date.now() - lastBeat >= 1000) {
    lastBeat = Date.now();
    var ns = n_scan(), ch = n_call(0), hd = n_call(1), age = take_age_max().valueOf() / 1e6;
    if (item0(itemBuf) > 0) {  // the first frame's header, once: for checking the layout on a new OS build
      var w = [];
      for (var i = 0; i < 18; i++) w.push(("0000000" + (itemBuf.add(4 * i).readU32() >>> 0).toString(16)).slice(-8));
      note("item", "I first frame item: " + w.join(" "));
    }
    if (clock_kind()) note("clock", "I frame times are on " + CLOCKS[clock_kind()]);
    if (n_badclk()) note("badclk", "W frame times on no clock picosight knows (the arch counter, boottime, monotonic): " +
                         "timed when they came");
    if (bad_fmt()) note("fmt" + bad_fmt(), "W frames of an unknown pixel format " + bad_fmt() + " skipped");
    if (target && ch === lastCalls[0] && hd === lastCalls[1]) {
      if (++quietBeats === 5) console.log("W no camera frames for 5 s: the cameras only run while the headset tracks (put it on)");
    } else quietBeats = 0;
    lastCalls = [ch, hd];
    console.log("T " + mono_us() + " scans " + ns + " scan_us " + (scan_ns().valueOf() / 1e3 / Math.max(1, ns)).toFixed(0) +
                " drops " + n_drop() + " faults 0 polls " + (ch + hd) + " ctrl " + ch + " head " + hd + " qdrop " + n_qdrop() +
                " lap " + n_lap() + " gain_skip " + n_gain() + " age_ms " + age.toFixed(1));
  }
  if (!MAX_MS || Date.now() - t0 < MAX_MS) setImmediate(step);
  else { stop(); console.log("I done"); }
}
setImmediate(step);
