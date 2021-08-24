#!/usr/bin/env bash

ORG=openrobotics
IMAGE=carma2
BRANCH=master

BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”`
VCS_REF=""
DISTRO=rolling

echo ""
echo "##### Building Space ROS Docker Image #####"
echo ""

docker build --no-cache -t $ORG/$IMAGE:$BRANCH \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg DISTRO="$DISTRO" .

echo ""
echo "##### Done! #####"

