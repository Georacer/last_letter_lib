# last_letter_lib

Software libraries for UAV simulation.

## Build instructions

### To build the C++ code

Run the following under the `uv` venv (`uv run`).

- pip3 install tools/requirements.txt
- pre-commit install
- mkdir build && cd build
- cmake ..
- make

### To build the Python code

<!-- Build the C++/Python bindings with Pybind11 -->

To build the Python wheels do

- uv build

To build the code so that you can import the Python code in editable mode do

- uv venv [--python 3.10]
- uv pip install --editable . [--force-reinstall]

### Build and run C++ tests

- cmake -S . -B build && cmake --build build && ./build/all_tests

or straight after compiling the Python package:

- ./build/cp310-cp310-linux_x86_64//all_tests

## Testing instructions
