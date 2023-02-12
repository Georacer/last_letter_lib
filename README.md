# last_letter_lib

Software libraries for UAV simulation.

## Build instructions

- pip3 install tools/requirements.txt
- pre-commit install
- mkdir build && cd build
- cmake ..
- make

## Build and run tests

- cmake -S . -B build && cmake --build build && ./build/all_tests
