#!/bin/bash

set -euo pipefail

source_dir=$(cd "$(dirname "$0")" && pwd)
assets_dir="${source_dir}/assets"
runtime_scripts=(
    "${source_dir}/scripts/run_robot.sh"
    "${source_dir}/scripts/set_imu_tty.sh"
)

build_type="release"
arch="x86_64"
ros_version="humble"
debian_prefix="robotics-engineai"
debian_name=""
version="1.0.0"
product_name=""
active_mode=""

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

usage() {
    echo "Usage: $0 -p product_name -m active_mode [-v version] [-a arch] [-r ros_version] [-h]"
    echo "  -p: Product name (required)"
    echo "  -m: active_mode written into mode.yaml (required)"
    echo "  -v: Debian package version (default: 1.0.0)"
    echo "  -a: Architecture: x86_64, aarch64 (default: ${arch})"
    echo "  -r: ROS version: humble, jazzy (default: ${ros_version})"
    echo "  -h: Show this help message and exit"
    echo "Debian package: ${debian_prefix}-<product>_<version>_<arch>.deb"
    echo "Underscores in the product name become hyphens in the package name."
    echo ""
    echo "Examples:"
    echo "  $0 -p t800 -m robot"
    echo "  $0 -p t800 -m robot -v 1.2.0 -a aarch64 -r jazzy"
}

parse_args() {
    local opt
    OPTIND=1
    while getopts ":p:v:m:a:r:h" opt; do
        case "${opt}" in
            p) product_name="${OPTARG}" ;;
            v) version="${OPTARG}" ;;
            m) active_mode="${OPTARG}" ;;
            a) arch="${OPTARG}" ;;
            r) ros_version="${OPTARG}" ;;
            h)
                usage
                exit 0
                ;;
            \?)
                echo "Invalid option: -${OPTARG}" >&2
                usage >&2
                exit 1
                ;;
            :)
                die "Option -${OPTARG} requires an argument."
                ;;
        esac
    done

    shift $((OPTIND - 1))
    if [ $# -gt 0 ]; then
        echo "Unexpected argument: $1" >&2
        usage >&2
        exit 1
    fi
}

validate_args() {
    local raw_version="${version}"

    if [ -z "${product_name}" ] || [ -z "${active_mode}" ]; then
        echo "[ERROR] -p product_name and -m active_mode are required." >&2
        echo ""
        usage >&2
        exit 1
    fi

    [ -n "${version}" ] || die "Option -v requires a version."
    version="${version//_/.}"
    if [ "${version}" != "${raw_version}" ]; then
        echo "[WARN] Version '${raw_version}' contains underscores (invalid in Debian); using '${version}' instead" >&2
    fi

    case "${ros_version}" in
        humble|jazzy) ;;
        *) die "Invalid ros_version: ${ros_version}. Valid: humble, jazzy" ;;
    esac

    case "${arch}" in
        x86_64|aarch64) ;;
        *) die "Invalid arch: ${arch}. Valid: x86_64, aarch64" ;;
    esac

    if [ "${arch}" = "x86_64" ] && [ "${ros_version}" = "jazzy" ]; then
        die "目前 x86_64 不支持 ROS 2 Jazzy，请选择 Humble，或使用 aarch64 Jazzy。"
    fi

    [ -d "${assets_dir}/config/${product_name}" ] || die "Product config not found: ${assets_dir}/config/${product_name}"

    local package_product="${product_name,,}"
    package_product="${package_product//_/-}"
    debian_name="${debian_prefix}-${package_product}"
    if [[ ! "${debian_name}" =~ ^[a-z0-9][a-z0-9+.-]*$ ]]; then
        die "Debian package name '${debian_name}' is invalid. Package names may only contain lowercase letters, digits, and -+."
    fi
    if [ "${package_product}" != "${product_name}" ]; then
        echo "[WARN] Product '${product_name}' cannot be used as a Debian package suffix; using '${debian_name}'." >&2
    fi
}

build_dir_name() {
    if [ "${arch}" = "aarch64" ] && [ "${ros_version}" = "jazzy" ]; then
        echo "aarch64-jazzy"
    else
        echo "${arch}"
    fi
}

dpkg_arch() {
    case "${arch}" in
        x86_64) echo "amd64" ;;
        aarch64) echo "arm64" ;;
    esac
}

ensure_package_build() {
    [ -d "${install_dir}" ] || die "Install directory not found: ${install_dir}. Build the ${arch} target first."
}

copy_runtime_scripts() {
    local script
    local scripts=()
    for script in "${runtime_scripts[@]}"; do
        [ -f "${script}" ] && scripts+=("${script}")
    done
    [ "${#scripts[@]}" -gt 0 ] || die "No runtime scripts found to package."
    rsync -a "${scripts[@]}" "${package_dir}/"
}

copy_product_assets() {
    local config_src="${assets_dir}/config/${product_name}"
    local urdf_src="${assets_dir}/resource/robot/${product_name}/urdf"
    local name

    [ -d "${urdf_src}" ] || die "URDF directory not found: ${urdf_src}"

    echo "Copying assets for ${product_name}..."
    mkdir -p "${package_dir}/assets/config" "${package_dir}/assets/resource/robot/${product_name}"
    rsync -a "${config_src}" "${package_dir}/assets/config/"
    rsync -a "${urdf_src}" "${package_dir}/assets/resource/robot/${product_name}/"

    for name in rc02 i18n; do
        if [ -d "${assets_dir}/resource/${name}" ]; then
            echo "Copying ${name} resources..."
            rsync -a "${assets_dir}/resource/${name}" "${package_dir}/assets/resource/"
        fi
    done
}

update_active_mode() {
    local mode_file="${package_dir}/assets/config/${product_name}/mode.yaml"
    [ -f "${mode_file}" ] || die "mode.yaml not found: ${mode_file}"

    sed -i "s/^active_mode:.*$/active_mode: ${active_mode}/" "${mode_file}"
    grep -q "^active_mode: ${active_mode}$" "${mode_file}" || die "Failed to update active_mode in ${mode_file}"
    echo "Updated active_mode to '${active_mode}'"
}

copy_install_tree() {
    rm -rf "${package_dir}/_install"
    rsync -a "${install_dir}/" "${package_dir}/_install"
}

write_release_note() {
    local note="${package_dir}/release_note.txt"
    local git_dir=""

    if [ -d "${source_dir}/.git" ]; then
        git_dir="${source_dir}/.git"
    elif [ -f "${source_dir}/.git" ]; then
        git_dir="$(sed 's/^gitdir: //' "${source_dir}/.git")"
    fi

    {
        if [ -n "${git_dir}" ]; then
            echo "commit_id: $(git --git-dir="${git_dir}" --work-tree="${source_dir}" rev-parse HEAD)"
            echo "branch: $(git --git-dir="${git_dir}" --work-tree="${source_dir}" rev-parse --abbrev-ref HEAD)"
            echo "commit date: $(git --git-dir="${git_dir}" --work-tree="${source_dir}" show -s --format=%ci HEAD)"
        fi
        echo "package date: $(date '+%Y-%m-%d %H:%M:%S')"
    } > "${note}"
}

install_systemd_service() {
    local stage="$1"
    local service_file="${stage}/lib/systemd/system/robotics.service"

    mkdir -p "$(dirname "${service_file}")"
    cp "${source_dir}/scripts/bringup/robotics.service" "${service_file}"
    chmod 644 "${service_file}"
    sed -i "s|ExecStart=/bin/bash /apps/engineai_robotics/run_robot.sh .*|ExecStart=/bin/bash /apps/engineai_robotics/run_robot.sh ${product_name}|" "${service_file}"
}

create_debian_package() {
    local pkg_arch deb_file stage app_dir
    pkg_arch="$(dpkg_arch)"
    deb_file="${source_dir}/${debian_name}_${version}_${pkg_arch}.deb"
    stage="${build_dir}/debian/${debian_name}-pkg"
    app_dir="${stage}/apps/engineai_robotics"

    rm -rf "${build_dir}/debian"
    mkdir -p "${app_dir}"
    cp -a "${source_dir}/scripts/DEBIAN" "${stage}/"
    sed -i \
        -e "s/^Package: .*/Package: ${debian_name}/" \
        -e "s/^Version: .*/Version: ${version}/" \
        -e "s/^Architecture: .*/Architecture: ${pkg_arch}/" \
        "${stage}/DEBIAN/control"

    install_systemd_service "${stage}"
    mv "${package_dir}/"* "${app_dir}/"
    chmod -R 755 "${stage}/DEBIAN"
    dpkg-deb -b -Zzstd -z9 "${stage}" "${deb_file}"
    echo "Debian package: ${deb_file}"
}

main() {
    local build_name
    parse_args "$@"
    validate_args

    build_name="$(build_dir_name)"
    build_dir="${source_dir}/build/${build_name}"
    install_dir="${build_dir}/_install"
    package_dir="${build_dir}/package/engineai_robotics"

    echo "Packaging ${product_name} as ${debian_name}_${version} (arch: ${arch}, ros: ${ros_version}, mode: ${active_mode})"

    ensure_package_build
    rm -rf "${package_dir}"
    mkdir -p "${package_dir}"
    copy_runtime_scripts
    copy_product_assets
    update_active_mode
    copy_install_tree
    write_release_note
    create_debian_package
}

main "$@"
