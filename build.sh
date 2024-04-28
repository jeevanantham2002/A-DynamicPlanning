#!/bin/bash
brew install qt qt-creator eigen llvm cmake
if [ ! -d "build" ]; then
  mkdir build
fi
if [ ! -d "bin" ]; then
    mkdir bin
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target all -- -j 8
