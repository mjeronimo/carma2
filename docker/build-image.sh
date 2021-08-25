#!/usr/bin/env bash

# Default to ROS 2 Foxy
rosdistro=foxy

optspec=":h-:"
while getopts "$optspec" optchar; do
    case "${optchar}" in
        -)
            case "${OPTARG}" in
                rosdistro)
                    val="${!OPTIND}"; OPTIND=$(( $OPTIND + 1 ))
                    rosdistro=${val}
                    ;;
                rosdistro=*)
                    val=${OPTARG#*=}
                    opt=${OPTARG%=$val}
                    rosdistro=${val}
                    ;;
                *)
                    echo "Unknown option --${OPTARG}" >&2
                    ;;
            esac;;
        h)
            echo "usage: $0 [-h] [--rosdistro[=]<foxy|rolling>]" >&2
            exit 1
            ;;
        *)
            echo "Unknown option: '-${OPTARG}'" >&2
            ;;
    esac
done

if [ "$rosdistro" = "" ]; then
    echo "The rosdistro option requires a ROS distro name"
    echo "usage: $0 [-h] [--rosdistro[=]<foxy|rolling>]" >&2
    exit 1
elif [ "$rosdistro" != "foxy" ] && [ "$rosdistro" != "rolling" ]; then
    echo "Unknown ROS distro: ${rosdistro}"
    echo "usage: $0 [-h] [--rosdistro[=]<foxy|rolling>]" >&2
    exit 1
fi

ORG=openrobotics
IMAGE=carma2
TAG=$rosdistro

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

