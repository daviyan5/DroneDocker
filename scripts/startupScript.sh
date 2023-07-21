#!/bin/bash
xhost +local:docker;
QGROUND="QGroundControl.AppImage";
if [ ! -f $QGROUND ]; then
    curl https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -o QGroundControl.AppImage;
    chmod +x ./QGroundControl.AppImage;
fi

