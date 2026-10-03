# Third-party notices

## OpenVR SDK 2.15.6

`third_party/openvr` holds `openvr.h`, `openvr_driver.h`, `openvr_api.lib` and
`openvr_api.dll` from ValveSoftware/openvr tag `v2.15.6`. The release ships
`openvr_api.dll` next to `QuestLHSync.exe`.

Copyright (c) 2015, Valve Corporation. Licensed under the 3-clause BSD license;
the complete text is in `third_party/openvr/LICENSE`.

Source: https://github.com/ValveSoftware/openvr

## MinHook

MinHook under `third_party/minhook` is Copyright (C) 2009-2017 Tsuda Kageyu,
with Hacker Disassembler Engine portions Copyright (c) 2008-2009 Vyacheslav
Patkov. It is distributed under a 2-clause BSD license; see
`third_party/minhook/LICENSE.txt`.

Source: https://github.com/TsudaKageyu/minhook

## Frida 17.10.0

The Magisk module ships `frida-inject` 17.10.0 for android-arm64, unmodified,
as published on Frida's GitHub release. It is not in this repository:
`magisk/build_module.py` downloads it and checks its SHA-256.

Copyright (C) Ole André Vadla Ravnås and contributors. Licensed under the
wxWindows Library Licence, Version 3.1.

Source: https://github.com/frida/frida
