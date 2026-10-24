// OpenVR's public openvr_api_public.cpp (2.x) calls an internal function its source drop doesn't include. It cleans up
// interfaces of in-process SteamVR builds; for a client linking openvr_api statically there is nothing to clean up.
namespace vr {
void CleanupInternalInterfaces() {}
}  // namespace vr
