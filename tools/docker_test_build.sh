#!/usr/bin/env bash

set -e

BASE_IMAGE=ubuntu:20.04
echo "*** Building project on docker on image ${BASE_IMAGE}"
docker run -it --volume=$(pwd):/last_letter_lib ${BASE_IMAGE} bash -c " \\
    cd last_letter_lib && \\
    ./tools/install_dependencies.sh && \\
    ./tools/build_project.sh && \\
    ./tools/install_project.sh && \\
    ls -al ~/last_letter_models/ && \\
    ls -al ~/.gazebo/worlds
"
