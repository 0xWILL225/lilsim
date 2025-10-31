#!/bin/bash
# Launch script for lilsim with proper ASAN settings

cd "$(dirname "$0")"
export ASAN_OPTIONS="detect_leaks=0:suppressions=$(pwd)/asan_suppressions.txt"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd)/third_party/wgpu-native/lib"
./build/debug/app/lilsim "$@"

