# QuestLHSync install: picosight reads the tracking cameras of a PICO 4 inside its tracking service
DEV=$(getprop ro.product.device)
MODEL=$(getprop ro.product.model)
case "$MODEL" in
  A8110*) ui_print "- PICO 4 ($MODEL, $DEV)" ;;
  A81*|A8E*) ui_print "- $MODEL ($DEV): a PICO 4 kind not tried yet, the driver's log says what picosight finds" ;;
  *) ui_print "! this is $MODEL ($DEV), not a PICO 4: picosight may not find its cameras" ;;
esac
if [ ! -f /mnt/vendor/persist/pvr/mav0/cam0/sensor.yaml ] && [ ! -f /mnt/vendor/factory/pvr/mav0/cam0/sensor.yaml ]; then
  ui_print "! no camera calibration in /mnt/vendor/persist/pvr/mav0: the PC can't use the cameras"
fi
set_perm "$MODPATH/lhsyncd" 0 0 0755
set_perm "$MODPATH/frida-inject" 0 0 0755
ui_print "- lhsyncd starts at boot. To start it now:"
ui_print "  su -c 'setsid sh $MODPATH/service.sh </dev/null >/dev/null 2>&1 &'"
