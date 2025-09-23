#!/bin/bash

IP_ADDRESS=192.168.8.156
echo $IP_ADDRESS

isRunning=`docker ps -f name=zenoh | grep -c "zenoh"`;
if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm astroviz
    docker run  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTHORITY \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e 'QT_X11_NO_MITSHM=1' \
        --env ROS_DOMAIN_ID=17 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /mnt:/mnt \
        -v /media/jetson:/media/jetson \
        --name zenoh \
        --net host \
        --ipc host \
        --pid host \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        --privileged \
        --ulimit rtprio=99 \
        --entrypoint /zenoh_entrypoint.sh \
        astroviz:latest $IP_ADDRESS

else
    echo "Docker already running."
    docker exec -it astroviz_zenoh /bin/bash
fi
