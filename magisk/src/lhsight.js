'use strict';
// lhsight: stream bright spots from the side tracking cameras (see lhsight.c). Runs inside the
// sensors HAL; reads only. Lines go to stdout via console.log:  F ... (frames)  T ... (heartbeat)
// QuestLHSync: lhsyncd runs this with frida-inject while a PC listens, and stops it after (MAX_MS 0: no limit)
var MAX_MS = parseInt("@DUR@");
// The headset's global-shutter cameras from its calibration file, "id:width:height:scan,..." (lhsyncd fills it in).
// scan 1: read this one. lhsyncd picks the side cameras (OV7251, 640x480): on the Quest Pro the front pair (OG01A,
// 1280x1024) never caught a base station, only other lights, and cost three quarters of the scan time. The Quest 3
// has the same four sensors. Left unfilled (a hand run): the Quest Pro's.
var CAMS_SPEC = "@CAMS@";
var CAMS = (CAMS_SPEC.charAt(0) === "@" ? "0:1280:1024:0,1:1280:1024:0,2:640:480:1,3:640:480:1" : CAMS_SPEC)
  .split(",").filter(function (e) { return e.length; }).map(function (e) {
    var f = e.split(":").map(function (v) { return parseInt(v, 10); });
    return { id: f[0], w: f[1], h: f[2], scan: f[3] === 1 };
  }).filter(function (c) { return c.id >= 0 && c.id < 16 && c.w > 0 && c.h > 0; });
var SCAN = CAMS.filter(function (c) { return c.scan; });
var MAXW = Math.max.apply(null, SCAN.map(function (c) { return c.w; }).concat([16]));
var MAXH = Math.max.apply(null, SCAN.map(function (c) { return c.h; }).concat([16]));
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
var n_poll = F("n_poll", "uint", []), idle_us = F("idle_us", "int", []);
var scratchBuf = Memory.alloc(MAXW * MAXH), cellBuf = Memory.alloc((MAXW >> 4) * (MAXH >> 4) * 2);
var labBuf = Memory.alloc((MAXW >> 4) * (MAXH >> 4) * 4);
var outBuf = Memory.alloc(1 << 20), drainBuf = Memory.alloc(1 << 16), tsBuf = Memory.alloc(16);
set_bufs(scratchBuf, cellBuf, labBuf, outBuf, 64);
var PAGE = 4096;
function bufSize(c) { return Math.ceil(c.w * c.h / PAGE) * PAGE; }  // one 8-bit frame, whole pages
function discover() {
  reset_slots();
  var fp = fopen(Memory.allocUtf8String("/proc/self/maps"), Memory.allocUtf8String("r")), buf = Memory.alloc(512), all = [];
  while (!fgets(buf, 512, fp).isNull()) {
    var m = /^([0-9a-f]+)-([0-9a-f]+)\s+rw-s.*\/dmabuf(?::dmabuf(\d+))?/.exec(buf.readCString());
    if (!m) continue;
    var a = parseInt(m[1], 16), b = parseInt(m[2], 16);
    all.push({ a: a, size: b - a, id: m[3] === undefined ? -1 : parseInt(m[3], 10) });
  }
  fclose(fp);
  // Each camera has a ring of frame buffers. The HAL allocates the rings in turn, lowest camera id first: the dmabuf
  // names count up in that order. Their addresses usually run the other way, but not always (one Quest Pro boot had
  // two of camera 2's buffers mapped above the front cameras'), so addresses are only the fallback. Cameras of the
  // same size share the buffers of that size equally (the Quest Pro: 10 each).
  var sizes = {}, n = 0, got = [];
  CAMS.forEach(function (c) { (sizes[bufSize(c)] = sizes[bufSize(c)] || []).push(c); });
  Object.keys(sizes).forEach(function (sz) {
    var cams = sizes[sz].sort(function (x, y) { return x.id - y.id; });
    var regs = all.filter(function (r) { return r.size === +sz; });
    var ids = cams.map(function (c) { return c.id; }).join("+");
    if (!regs.length || regs.length % cams.length) {
      console.log("W cameras " + ids + " (" + cams[0].w + "x" + cams[0].h + "): " + regs.length + " buffers, not " +
                  cams.length + " equal rings");
      return;
    }
    var per = regs.length / cams.length, byName = regs.every(function (r) { return r.id >= 0; });
    if (byName) regs.sort(function (x, y) { return x.id - y.id; });
    else {
      console.log("W cameras " + ids + ": buffers not named in allocation order, mapping them by address");
      regs.sort(function (x, y) { return y.a - x.a; });
    }
    regs.forEach(function (r, k) {
      var c = cams[Math.floor(k / per)];
      if (c.scan) { add_slot(ptr(r.a), c.w, c.h, c.id); n++; }
    });
    cams.forEach(function (c) { if (c.scan) got.push(c.id + ":" + per); });
  });
  if (!n) {  // what the HAL has instead: what a new headset or OS build needs looked at
    var hist = {};
    all.forEach(function (r) { hist[r.size] = (hist[r.size] || 0) + 1; });
    console.log("W no camera buffers found; dmabuf sizes here: " + (Object.keys(hist).map(function (s) {
      return "0x" + (+s).toString(16) + "x" + hist[s];
    }).join(" ") || "none"));
  } else console.log("I cameras " + got.join(" "));
  prime();
  return n;
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
    usleep(Math.max(1000, idle_us()));
  }
  flush();
  if (Date.now() - lastBeat >= 1000) {
    lastBeat = Date.now();
    var ns = n_scan();
    console.log("T " + mono_us() + " scans " + ns + " scan_us " + (scan_ns().valueOf() / 1e3 / Math.max(1, ns)).toFixed(0) + " drops " + n_drop() + " faults " + faults + " polls " + n_poll());
  }
  if (!MAX_MS || Date.now() - t0 < MAX_MS) setImmediate(step);
  else console.log("I done");
}
setImmediate(step);
