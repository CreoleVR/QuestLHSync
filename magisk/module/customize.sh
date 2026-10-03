# QuestLHSync install: lhsight reads the tracking cameras of a Quest Pro, 3 or 3S
DEV=$(getprop ro.product.device)
case "$DEV" in
  seacliff*) ui_print "- Quest Pro ($DEV)" ;;
  eureka*) ui_print "- Quest 3 ($DEV): not tried yet, the driver's log says what lhsight finds" ;;
  panther*) ui_print "- Quest 3S ($DEV): not tried yet, the driver's log says what lhsight finds" ;;
  *) ui_print "! this is $DEV, not a Quest Pro, 3 or 3S: lhsight may not find its cameras" ;;
esac
set_perm "$MODPATH/lhsyncd" 0 0 0755
set_perm "$MODPATH/frida-inject" 0 0 0755
ui_print "- lhsyncd starts at boot. To start it now:"
ui_print "  su -c 'setsid sh $MODPATH/service.sh </dev/null >/dev/null 2>&1 &'"
