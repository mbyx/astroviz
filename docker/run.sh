#!/bin/bash

COMMAND=$1
DOMAIN_ID=42
EXTRA_MOUNT=""

if [ ! -z "$1" ] && [[ "$1" != -* ]]; then
    shift
fi

while getopts "d:m:" opt; do
  case $opt in
    d) DOMAIN_ID="$OPTARG" ;;
    m) EXTRA_MOUNT="$OPTARG" ;;
    *) echo "Usage: $0 [-d domain_id] [-m mount_path]"; exit 1 ;;
  esac
done

# Ensure we are using the native engine
docker context use default

# -isRunning=`docker ps -f name=astroviz | grep -c "astroviz"`;
isRunning=0;

if [ $isRunning -eq 0 ]; then
    # Allow showing GUIs locally from docker.
    xhost +local:docker
    docker rm -f astroviz 2>/dev/null || true

    MOUNT_ARG=""
    if [ ! -z "$EXTRA_MOUNT" ]; then
        ABS_MOUNT=$(realpath "$EXTRA_MOUNT")
        MOUNT_ARG=("-v" "$ABS_MOUNT:/external_data")
        echo "Mounting $ABS_MOUNT to /external_data"
    fi

    case "$COMMAND" in
        "dashboard")
            STARTUP_CMD="source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch astroviz astroviz.launch.py"
            ;;
        *)
            STARTUP_CMD="/bin/bash"
            ;;
    esac

    # FIXME(docker): This is quite unsafe, preferably change to be more secure.
    docker run \
        --name astroviz \
        -it \
        --net host \
        --ipc host \
        --privileged \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --env ROS_DOMAIN_ID=$DOMAIN_ID \
        --device /dev/dri:/dev/dri \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v `pwd`/../:/ros2_ws/src/astroviz \
        "${MOUNT_ARG[@]}" \
        -v /dev:/dev \
        -w /ros2_ws \
        --entrypoint /bin/bash \
        astroviz:latest \
        -c "$STARTUP_CMD"
else
    docker exec -it astroviz /bin/bash
fi
