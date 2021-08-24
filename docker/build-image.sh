#!/usr/bin/env bash

ORG=openrobotics
IMAGE=carma2
BRANCH=master
VERSION=rolling
VCS_REF=""
BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”`

echo ""
echo "##### Building Space ROS Docker Image #####"
echo ""

docker build --no-cache -t $ORG/$IMAGE:$BRANCH \
    --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

echo ""
echo "##### Done! #####"

