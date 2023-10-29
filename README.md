# last_letter_lib

Software libraries for UAV simulation.

## Build instructions

### To build the C++ code

- pip3 install tools/requirements.txt
- pre-commit install
- mkdir build && cd build
- cmake ..
- make

### To build the Python code

<!-- Build the C++/Python bindings with Pybind11 -->

- poetry build && poetry install

### To run the Python tests

- poetry run pytest tests/python

### Build and run C++ tests

- cmake -S . -B build && cmake --build build && ./build/all_tests

or straight after compiling the Python package:

- ./build/temp.linux-x86_64-cpython-310/last_letter_lib.cpp_last_letter_lib/all_tests
