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

A nix-shell pure configuration has been provided which can be optionally used.

### To build the Python code

To build the Python wheels do

- uv build

To build the code so that you can import the Python code in editable mode do

- uv pip install --editable . [--force-reinstall]

## Testing instructions

### Build and run C++ tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DLAST_LETTER_BUILD_TESTS=ON && cmake --build build && cmake --build build && ./build/all_tests
```

### Build everything

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DLAST_LETTER_BUILD_TESTS=ON -DLAST_LETTER_BUILD_TESTS=ON && cmake --build build && cmake --build build && ./build/all_tests
```

### Run the Python tests

- uv run pytest tests/python
