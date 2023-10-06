#!/bin/bash

command_string=""

if [[ "$1" == "core" || "$2" == "core" || "$3" == "core" || "$4" == "core" ]]; then
    command_string+=" --tab -e 'bash -c \"roscore; bash\"'"
fi
command_string+=" --tab -e 'bash -c \"sleep 0; sh scripts/odometry.sh vins; bash\"'"

if [[ "$1" == "rqt" || "$2" == "rqt" || "$3" == "rqt" || "$4" == "rqt" ]]; then
    command_string+=" --tab -e 'bash -c \"sleep 0; rqt; bash\"'"
fi

if [[ "$1" == "loop" || "$2" == "loop" || "$3" == "loop" || "$4" == "loop" ]]; then
    command_string+=" --tab -e 'bash -c \"sleep 0.5; sh scripts/odometry.sh loop; bash\"'"
fi

if [[ "$1" == "viz" || "$2" == "viz" || "$3" == "viz" || "$4" == "viz" ]]; then
    command_string+=" --tab -e 'bash -c \"sleep 0.5; sh scripts/odometry.sh viz; bash\"'"
fi
echo $command_string
eval "gnome-terminal $command_string"
