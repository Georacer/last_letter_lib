# last_letter_lib

Software libraries for UAV simulation.

## Build instructions

### To build the C++ code

- pip3 install tools/requirements.txt
- pre-commit install
- mkdir build && cd build
- cmake ..
- make

## Build and run C++ tests

- cmake -S . -B build && cmake --build build && ./build/all_tests

### To build the Python code

<!-- Build the C++/Python bindings with Pybind11 -->

<!-- - poetry build -->

- poetry install
