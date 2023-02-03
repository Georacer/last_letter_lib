#!/usr/bin/env bash

set -e

# Set current working directory to the directory of the script
cd "$(dirname "$0")"

cd ../models

# Generate the model folders in the user home
./default.py
