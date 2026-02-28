#!/bin/sh
set -eu

REPO="bobir01/mcap2vid"
BINARY="mcap2vid"

detect_arch() {
    os="$(uname -s)"
    case "$os" in
        Linux) ;;
        *) echo "Error: unsupported OS: $os (only Linux is supported)" >&2; exit 1 ;;
    esac

    arch="$(uname -m)"
    case "$arch" in
        x86_64|amd64)  echo "x86_64-unknown-linux-gnu" ;;
        aarch64|arm64) echo "aarch64-unknown-linux-gnu" ;;
        *) echo "Error: unsupported architecture: $arch" >&2; exit 1 ;;
    esac
}

fetch_latest_version() {
    url="https://api.github.com/repos/${REPO}/releases/latest"
    if command -v curl >/dev/null 2>&1; then
        curl -sSfL "$url" | grep '"tag_name"' | head -1 | sed 's/.*"tag_name": *"\([^"]*\)".*/\1/'
    elif command -v wget >/dev/null 2>&1; then
        wget -qO- "$url" | grep '"tag_name"' | head -1 | sed 's/.*"tag_name": *"\([^"]*\)".*/\1/'
    else
        echo "Error: curl or wget required" >&2; exit 1
    fi
}

download_and_verify() {
    version="$1"
    target="$2"
    tmpdir="$(mktemp -d)"

    base_url="https://github.com/${REPO}/releases/download/${version}"
    tarball="${BINARY}-${version}-${target}.tar.gz"

    echo "Downloading ${tarball}..."
    if command -v curl >/dev/null 2>&1; then
        curl -sSfL -o "${tmpdir}/${tarball}" "${base_url}/${tarball}"
        curl -sSfL -o "${tmpdir}/checksums.txt" "${base_url}/checksums.txt"
    else
        wget -q -O "${tmpdir}/${tarball}" "${base_url}/${tarball}"
        wget -q -O "${tmpdir}/checksums.txt" "${base_url}/checksums.txt"
    fi

    echo "Verifying checksum..."
    cd "$tmpdir"
    grep "$tarball" checksums.txt | sha256sum -c - >/dev/null 2>&1
    cd - >/dev/null

    echo "$tmpdir/$tarball"
}

install_binary() {
    tarball="$1"
    install_dir="$2"

    mkdir -p "$install_dir"
    tar -xzf "$tarball" -C "$install_dir"
    chmod +x "${install_dir}/${BINARY}"
    echo "Installed ${BINARY} to ${install_dir}/${BINARY}"
}

post_install_checks() {
    install_dir="$1"

    case ":${PATH}:" in
        *":${install_dir}:"*) ;;
        *) echo "Warning: ${install_dir} is not in your PATH" >&2 ;;
    esac

    if ! command -v ffmpeg >/dev/null 2>&1; then
        echo "Warning: ffmpeg not found in PATH (required for video encoding)" >&2
    fi
}

main() {
    install_dir=""
    for arg in "$@"; do
        case "$arg" in
            --dir=*) install_dir="${arg#--dir=}" ;;
            --dir)   shift; install_dir="$1" ;;
        esac
    done

    if [ -z "$install_dir" ]; then
        if [ "$(id -u)" = "0" ]; then
            install_dir="/usr/local/bin"
        else
            install_dir="${HOME}/.local/bin"
        fi
    fi

    target="$(detect_arch)"
    echo "Detected target: ${target}"

    version="$(fetch_latest_version)"
    echo "Latest version: ${version}"

    tarball="$(download_and_verify "$version" "$target")"
    install_binary "$tarball" "$install_dir"
    rm -rf "$(dirname "$tarball")"

    post_install_checks "$install_dir"
}

main "$@"
