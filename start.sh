#!/bin/bash

# Calulate LUT with GNU Octave
echo "Calculating LUT..."
cd octave && octave --silent --eval "main; exit;" && cd ..

# Build the project
echo "Building project..."
rm -rf build
mkdir build
cd build
cmake ..
make

# Run the project
echo "Running project..."
./test_motor

