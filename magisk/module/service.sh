#!/system/bin/sh
# QuestLHSync: keeps lhsyncd running (it starts and stops lhsight itself, per connected PC)
MODDIR=${0%/*}
RUN=/dev/.questlhsync
until [ "$(getprop sys.boot_completed)" = 1 ]; do sleep 2; done
mkdir -p $RUN/tmp
chmod 0755 $RUN $RUN/tmp
# started again without a reboot (an update): replace the running loop, then its lhsyncd (which stops its lhsight)
[ -f $RUN/service.pid ] && kill "$(cat $RUN/service.pid)" 2>/dev/null
if pkill -x lhsyncd; then
  i=0
  while pidof lhsyncd >/dev/null && [ $i -lt 10 ]; do sleep 1; i=$((i + 1)); done
fi
(
  while [ ! -e "$MODDIR/disable" ] && [ ! -e "$MODDIR/remove" ]; do
    "$MODDIR/lhsyncd" "$MODDIR"
    [ $? = 3 ] && sleep 30   # another lhsyncd holds the ports
    sleep 5
  done
) </dev/null >/dev/null 2>&1 &
echo $! > $RUN/service.pid
