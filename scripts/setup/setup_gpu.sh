#!/bin/bash

# ROOT_DIR="$(git rev-parse --show-toplevel)"
ROOT_DIR="/home/dsliwowski/Projects/REASSEMBLE/REASSEMBLE"
DOCKER_COMPOSE_DIR="$ROOT_DIR/.docker/gpu_pc"
DOCKER_COMPOSE_FILE="$DOCKER_COMPOSE_DIR/docker-compose.yaml"

# ensure GUI window is accessible from container
echo -e "Set Docker Xauth for x11 forwarding \n"

export DOCKER_XAUTH=/tmp/.docker.xauth
rm $DOCKER_XAUTH
touch $DOCKER_XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $DOCKER_XAUTH nmerge -

# build client server container

read -p "Do you want to rebuild the container image? (yes/no): " first_time

if [ "$first_time" = "yes" ]; then
	echo -e "build control server container \n"
	cd $DOCKER_COMPOSE_DIR && docker compose -f $DOCKER_COMPOSE_FILE build
fi

echo -e "run client application \n"
docker compose -f $DOCKER_COMPOSE_FILE up -d