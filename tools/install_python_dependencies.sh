#!/usr/bin/env bash

# Dependencies for the Python codebase. Assume a python-based image.

set -e
# Avoid getting asked for prompts
export DEBIAN_FRONTEND=noninteractive

# Install pyenv
curl https://pyenv.run | bash

# Install poetry
wget https://raw.githubusercontent.com/python-poetry/poetry/master/install-poetry.py
python3 install-poetry.py
rm install-poetry.py

# Install nox
pip3 install nox
pip3 install nox-poetry
