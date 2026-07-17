#!/bin/bash

# Exits on error
set -e

# Gets the source directory
source_dir=$(cd $(dirname $0) && pwd)

# Builds the project
build_dir="$source_dir/build"
mkdir -p $build_dir && cd $build_dir
rm -rf *
echo "[INFO] All files in [$build_dir] have been cleared."
