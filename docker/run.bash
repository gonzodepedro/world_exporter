#!/usr/bin/env bash
#
# Purpose
#   This script is designed to run a docker image built using build.bash.
#   See README.md and build.bash for more information.

if [ $# -lt 1 ]
then
    echo "Usage: $0 <docker image> [optional arguments to ign-gazebo]"
    exit 1
fi

IMG=$(basename $1)

ARGS=("$@")

# Make sure processes in the container can connect to the x server.
# This is necessary so Gazebo can create a context for OpenGL rendering
# (even headless).
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

CONTAINER_WS_PATH_="/home/developer/workspaces/src/"
WS_DIR=$2
echo "Workspace: $WS_DIR -> $CONTAINER_WS_PATH_"
DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:$CONTAINER_WS_PATH_"

docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -e IGN_PARTITION \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  --rm \
  --runtime=nvidia \
  --security-opt seccomp=unconfined \
  $DOCKER_OPTS \
  $IMG \
  #${@:2}
