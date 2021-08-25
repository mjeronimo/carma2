#!/usr/bin/env bash

ORG=openrobotics
IMAGE=carma2
TAG=rolling

BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”`
VCS_REF=""

echo ""
echo "##### Building Space ROS Docker Image #####"
echo ""

docker build --no-cache -t $ORG/$IMAGE:$TAG \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg DISTRO="$TAG" .

echo ""
echo "##### Done! #####"

