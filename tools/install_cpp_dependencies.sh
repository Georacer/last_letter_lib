#!/usr/bin/env bash

# Dependencies for the C++ codebase. Assume an ubuntu 20.04 image.

set -e
# Avoid getting asked for prompts
export DEBIAN_FRONTEND=noninteractive

# Install system dependencies
apt-get update && apt-get install -y\
    python3-pip \
    cmake \
    libeigen3-dev \
    libblas-dev \
    liblapack-dev \
    git \
    wget \
    python3-venv
# git needed to clone submodules

# Dependencies specific to pcg-gazebo python package
apt-get install -y\
    libgeos-dev
    # libfcl-dev \ # libfcl-dev needed for pip to build pcg-gazebo wheel

# Update pip
pip3 install --upgrade pip

# Install pip packages for the C++ codebase
pip3 install -r tools/requirements.txt
