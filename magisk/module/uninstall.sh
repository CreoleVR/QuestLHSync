#!/system/bin/sh
pkill -f lhsight.js 2>/dev/null
pkill -x lhsight-gxr 2>/dev/null
pkill -x lhsyncd 2>/dev/null
rm -rf /dev/.questlhsync /data/local/tmp/questlhsync
