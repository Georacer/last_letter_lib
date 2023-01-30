#!/usr/bin/env bash
# Run this script from the source root directory

set -e

rm -rf build
mkdir build
cd build
cmake ..
make
