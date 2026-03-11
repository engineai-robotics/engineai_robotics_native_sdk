#!/bin/bash

# Exits on error
set -e

# sudo chmod 666 /dev/ttyACM0

# Gets the source directory
readonly source_dir=$(cd $(dirname $0) && pwd)
readonly symbol_dir=""
readonly report_dir="/tmp/crashpad/report"
readonly dmpfile_dir="/tmp/crashpad/coredump/pending"

echo "[INFO] Exports the environment variables:"
cd $source_dir
source ./env.sh

echo "[INFO] Start the crash monitor:"
# record the core dump files before the executor is run
before_list_file=$(mktemp)
trap 'rm -f -- "$before_list_file"' EXIT
find "$dmpfile_dir" -maxdepth 1 -type f -name "*.dmp" | sort > "$before_list_file"

echo "[INFO] Run the executor:"
# Opens the executor path
install_dir="$source_dir/build/_install"
cd $install_dir/bin

# Runs executable files
set +e
if [ $# -gt 0 ]; then
    ./src_executor "$1"
else
    ./src_executor
fi

# Process the core dump files
exit_code=$?
if [ $exit_code -gt 128 ]; then
    new_dmp_files=$(comm -13 "$before_list_file" <(find "$dmpfile_dir" -maxdepth 1 -type f -name "*.dmp" | sort))
    if [ -n "$new_dmp_files" ]; then
        echo "[INFO] New core dump files found: $new_dmp_files"
        cd $source_dir
        ./scripts/process_dump.sh "$new_dmp_files" "$symbol_dir" "$report_dir"

        # print the full report with loaded modules
        # ./scripts/process_dump.sh -v "$new_dmp_files" "$symbol_dir" "$report_dir"
    fi
fi
