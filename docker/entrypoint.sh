#!/bin/bash
set -e

# Setup the CARMA 2 environment
source install/setup.bash
exec "$@"
