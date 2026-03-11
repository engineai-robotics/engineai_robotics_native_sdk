#!/usr/bin/env bash
set -e

function print_help() {
    echo "Usage: $0 [-v] DMP_PATH SYM_ROOT OUTPUT_DIR"
    echo "      <DMP_PATH>: the path to the dmp file"
    echo "      <SYM_ROOT>: the path to the symbol directory (empty for runtime generation)"
    echo "      <OUTPUT_DIR>: the path to the output directory"
    echo "      -v: verbose mode, print all the output, including the loading modules"
    echo "      -h: help"
    echo "example:"
    echo "      ./process_dump.sh /path/to/dmp /path/to/sym_root /path/to/output"
    echo "      ./process_dump.sh -v /path/to/dmp /path/to/sym_root /path/to/output"
    echo "      ./process_dump.sh -h"
}

# 检查文件是否需要更新符号表
check_symbol_needs_update() {
    local binary_file="$1"
    local symbol_file="$2"
    
    # 如果符号文件不存在，需要生成
    if [ ! -f "$symbol_file" ]; then
        return 0  # 需要更新
    fi
    
    # 如果二进制文件比符号文件新，需要更新
    if [ "$binary_file" -nt "$symbol_file" ]; then
        return 0  # 需要更新
    fi
    
    return 1  # 不需要更新
}

# 检查目录是否需要更新符号表
check_directory_needs_update() {
    local install_dir="$1"
    local symbols_dir="$2"
    
    # 如果符号目录不存在，需要生成
    if [ ! -d "$symbols_dir" ]; then
        return 0  # 需要更新
    fi
    
    # 检查install目录中是否有文件比符号目录新
    # 使用find找到install目录中最新的文件，然后与符号目录比较
    local newest_binary=$(find -L "$install_dir" -type f \( -name "*.so" -o -name "src_executor" \) -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
    
    if [ -n "$newest_binary" ] && [ "$newest_binary" -nt "$symbols_dir" ]; then
        return 0  # 需要更新
    fi
    
    return 1  # 不需要更新
}

VERBOSE=0
ARGS=()
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)
            print_help
            exit 0 ;;
        --) shift; ARGS+=("$@"); break ;;
        *)  ARGS+=("$arg") ;;
    esac
done

if [[ ${#ARGS[@]} -lt 3 ]]; then
    echo "Error: missing arguments"
    print_help
    exit 1
fi

readonly DMP="${ARGS[0]}"
readonly SYM_ROOT="${ARGS[1]}"
readonly OUTPUT_DIR="${ARGS[2]}"
readonly TOOLS="/opt/engineai_robotics_third_party/bin"

if [ ! -d "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
fi

# 如果SYM_ROOT为空，则动态生成符号表
if [ -z "$SYM_ROOT" ]; then
    echo "[Processor] Symbol directory is empty, checking for cached symbols..."
    
    # 获取根目录（脚本目录的上一层）
    readonly root_dir=$(cd $(dirname $0)/.. && pwd)
    readonly runtime_symbols_dir="$root_dir/build/runtime_symbols"
    readonly install_dir="$root_dir/build/_install"
    readonly dump_syms_bin="$TOOLS/dump_syms"
    
    # 先检查整个目录是否需要更新
    if check_directory_needs_update "$install_dir" "$runtime_symbols_dir"; then
        echo "[Processor] Binary files updated, regenerating symbols..."
        
        # 创建运行时符号表目录
        mkdir -p "$runtime_symbols_dir"
        
        # 为所有.so文件生成符号表（仅在需要时）
        find -L "$install_dir" -type f -name "*.so" -print0 | while IFS= read -r -d $'\0' so_file; do
            module_id=$("$dump_syms_bin" -i "$so_file" | head -n1 | awk '{print $4}')
            outdir="$runtime_symbols_dir/$(basename "$so_file")/$module_id"
            symbol_file="$outdir/$(basename "$so_file").sym"
            
            if check_symbol_needs_update "$so_file" "$symbol_file"; then
                echo "[Processor] Generating symbol for: $so_file"
                mkdir -p "$outdir"
                "$dump_syms_bin" "$so_file" > "$symbol_file"
            # else
            #     echo "[Processor] Using cached symbol for: $so_file"
            fi
        done
        
        # 为主执行文件生成符号表（仅在需要时）
        executor_bin="$install_dir/bin/src_executor"
        if [ -f "$executor_bin" ]; then
            module_id=$("$dump_syms_bin" -i "$executor_bin" | head -n1 | awk '{print $4}')
            outdir="$runtime_symbols_dir/$(basename "$executor_bin")/$module_id"
            symbol_file="$outdir/$(basename "$executor_bin").sym"
            
            if check_symbol_needs_update "$executor_bin" "$symbol_file"; then
                echo "[Processor] Generating symbol for: $executor_bin"
                mkdir -p "$outdir"
                "$dump_syms_bin" "$executor_bin" > "$symbol_file"
            else
                echo "[Processor] Using cached symbol for: $executor_bin"
            fi
        fi
        
        # 更新符号目录的时间戳
        touch "$runtime_symbols_dir"
    else
        echo "[Processor] Using all cached symbols (no binary files updated)"
    fi
    
    # 使用生成的符号表目录
    SYM_ROOT_TO_USE="$runtime_symbols_dir"
else
    SYM_ROOT_TO_USE="$SYM_ROOT"
fi

echo "[Processor] Symbolizing crash..."
stamp=$(date +%Y%m%d%H%M%S)
REPORT="$OUTPUT_DIR/report_${stamp}.txt"

"$TOOLS/minidump_stackwalk" -c "$DMP" "$SYM_ROOT_TO_USE" 2>&1 | grep -v 'INFO:' > "$REPORT"

if (( VERBOSE )); then
    cat "$REPORT"
else
    cat "$REPORT" | sed '/^Loaded modules:/,/^$/d'
fi

echo "[Processor] Crash report saved to $REPORT"
