#!/bin/bash
# start roscore and bridge
# this script is added as a root cronjob
# to see and edit the job
# $ crontab -e
#
/usr/bin/roscore &
# wait for ROSCORE to start
sleep 1
date >> rebootinfo.txt
#start the bridge from user 'local's home directory
cd /home/local/
./bridge -d &
# save PID for debugging
echo "Bridge started with PID:" >> rebootinfo.txt
pgrep -l bridge >> rebootinfo.txt
exit 0

