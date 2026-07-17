#!/bin/bash

# Exits on error
set -e

RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Gets the source directory
readonly source_dir=$(cd $(dirname $0) && pwd)
readonly symbol_dir=""
readonly report_dir="/tmp/crashpad/report"
readonly dmpfile_dir="/tmp/crashpad/coredump/pending"

if pgrep -f src_executor > /dev/null; then
    echo -e "${RED}⚠️ WARNING: ⚠️${NC}"
    echo -e "${YELLOW}The src_executor process is currently running.${NC}"
    echo -e ${YELLOW}PID: $(pgrep -f src_executor)${NC}
    exit 1
fi

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
