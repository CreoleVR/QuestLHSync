# QuestLHSync install: only a Quest Pro has the cameras lhsight reads
DEV=$(getprop ro.product.device)
case "$DEV" in
  seacliff*) ui_print "- Quest Pro ($DEV)" ;;
  *) ui_print "! this is $DEV, not a Quest Pro (seacliff): lhsight won't find its cameras" ;;
esac
set_perm "$MODPATH/lhsyncd" 0 0 0755
set_perm "$MODPATH/frida-inject" 0 0 0755
ui_print "- lhsyncd starts at boot. To start it now:"
ui_print "  su -c 'setsid sh $MODPATH/service.sh </dev/null >/dev/null 2>&1 &'"
