# Ensure we are using the native engine
docker context use default

isRunning=`docker ps -f name=astroviz | grep -c "astroviz"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm -f astroviz 2>/dev/null || true
    
    docker run \
        --name astroviz \
        -it \
        --net host \
        --ipc host \
        --privileged \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --device /dev/dri:/dev/dri \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v `pwd`/../:/ros2_ws/src/astroviz \
        -v /dev:/dev \
        -w /ros2_ws \
        --entrypoint /bin/bash \
        astroviz:latest
else
    docker exec -it astroviz /bin/bash
fi
