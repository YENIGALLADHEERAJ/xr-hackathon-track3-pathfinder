#!/bin/sh
# Build ./pathfinder with g++ or clang++ (Linux / macOS / MinGW).
set -e
cd "$(dirname "$0")"
CXX="${CXX:-$(command -v g++ || command -v clang++)}"
"$CXX" -O2 -std=c++17 -Wall -o pathfinder main.cpp
echo "built $(pwd)/pathfinder"
