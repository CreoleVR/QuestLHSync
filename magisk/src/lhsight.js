'use strict';
// lhsight: stream bright spots from the four tracking cameras (see lhsight.c). Runs inside the
// sensors HAL; reads only. Lines go to stdout via console.log:  F ... (frames)  T ... (heartbeat)
// QuestLHSync: lhsyncd runs this with frida-inject while a PC listens, and stops it after (MAX_MS 0: no limit)
var MAX_MS = parseInt("@DUR@");
var RING_CAM = [3, 2, 1, 0];   // rings sorted by address -> calibration camera id
var l = Process.getModuleByName("libc.so"), g = function (n) { return l.getExportByName(n); };
var fopen = new NativeFunction(g("fopen"), "pointer", ["pointer", "pointer"]);
var fgets = new NativeFunction(g("fgets"), "pointer", ["pointer", "int", "pointer"]);
var fclose = new NativeFunction(g("fclose"), "int", ["pointer"]);
var usleep = new NativeFunction(g("usleep"), "int", ["uint"]);
var clock_gettime = new NativeFunction(g("clock_gettime"), "int", ["int", "pointer"]);
// Keep every Memory.alloc in a global: JS GC frees unreferenced allocations under the C code.
var stateBuf = Memory.alloc(16384);
var cm = new CModule(`@CSRC@`, { clock_gettime: g("clock_gettime"), memcpy: g("memcpy"), snprintf: g("snprintf"), S: stateBuf });
var F = function (n, r, a) { return new NativeFunction(cm[n], r, a); };
var add_slot = F("add_slot", "void", ["pointer", "int", "int", "int"]);
var set_bufs = F("set_bufs", "void", ["pointer", "pointer", "pointer", "pointer", "int"]);
var prime = F("prime", "void", []), poll = F("poll", "int", []), reset_slots = F("reset_slots", "void", []);
var drain = F("drain", "int", ["pointer", "int"]);
var n_drop = F("n_drop", "uint", []), scan_ns = F("scan_ns", "uint64", []), n_scan = F("n_scan", "uint", []);
var scratchBuf = Memory.alloc(1280 * 1024), cellBuf = Memory.alloc(80 * 64 * 2), labBuf = Memory.alloc(80 * 64 * 4);
var outBuf = Memory.alloc(1 << 20), drainBuf = Memory.alloc(1 << 16), tsBuf = Memory.alloc(16);
set_bufs(scratchBuf, cellBuf, labBuf, outBuf, 250);
function discover() {
  reset_slots();
  var fp = fopen(Memory.allocUtf8String("/proc/self/maps"), Memory.allocUtf8String("r")), buf = Memory.alloc(512), regs = [];
  while (!fgets(buf, 512, fp).isNull()) {
    var m = /^([0-9a-f]+)-([0-9a-f]+)\s+rw-s.*\/dmabuf/.exec(buf.readCString());
    if (!m) continue;
    var a = parseInt(m[1], 16), b = parseInt(m[2], 16);
    if (b - a === 0x140000) regs.push([a, 1280, 1024]);
    else if (b - a === 0x4b000) regs.push([a, 640, 480]);
  }
  fclose(fp);
  regs.sort(function (x, y) { return x[0] - y[0]; });
  if (regs.length !== 40) console.log("W unexpected slot count " + regs.length);
  regs.forEach(function (r, k) { add_slot(ptr(r[0]), r[1], r[2], RING_CAM[Math.floor(k / 10)]); });
  prime();
  return regs.length;
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
console.log("I slots " + discover() + " max_ms " + MAX_MS);
var t0 = Date.now(), lastBeat = 0, faults = 0;
function step() {
  var budget = Date.now() + 20;
  while (Date.now() < budget) {
    try { poll(); } catch (e) { faults++; usleep(200000); console.log("W fault " + e.message + " -> " + discover() + " slots"); }
    usleep(1000);
  }
  flush();
  if (Date.now() - lastBeat >= 1000) {
    lastBeat = Date.now();
    var ns = n_scan();
    console.log("T " + mono_us() + " scans " + ns + " scan_us " + (scan_ns().valueOf() / 1e3 / Math.max(1, ns)).toFixed(0) + " drops " + n_drop() + " faults " + faults);
  }
  if (!MAX_MS || Date.now() - t0 < MAX_MS) setImmediate(step);
  else console.log("I done");
}
setImmediate(step);
