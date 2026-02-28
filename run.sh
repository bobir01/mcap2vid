#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="$SCRIPT_DIR/target/release/mcap2vid"

# Defaults
INPUT_DIR="${1:-}"
OUTPUT_DIR="${2:-}"
TOPIC="${3:-}"
SUFFIX="${4:-}"

usage() {
    echo "Usage: $0 <input_dir> <output_dir> [topic] [suffix]"
    echo ""
    echo "Export videos from MCAP files"
    echo ""
    echo "Arguments:"
    echo "  input_dir   Directory containing .mcap files"
    echo "  output_dir  Directory for output .mp4 files"
    echo "  topic       Camera topic (optional, auto-detected if omitted)"
    echo "  suffix      Suffix appended to output filename (e.g., 'ego' -> name_ego.mp4)"
}

if [[ -z "$INPUT_DIR" || -z "$OUTPUT_DIR" ]]; then
    usage
    exit 1
fi

if [[ ! -x "$BINARY" ]]; then
    echo "Error: Binary not found. Run 'cargo build --release' first."
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

detect_topic() {
    local mcap_file="$1"
    "$BINARY" list -i "$mcap_file" 2>/dev/null | head -1 | awk '{print $1}'
}

export_file() {
    local mcap_file="$1"
    local basename
    basename=$(basename "$mcap_file" .mcap)
    local output_dir="${OUTPUT_DIR}/${basename}/video"
    local output_name="${basename}"
    if [[ -n "$SUFFIX" ]]; then
        output_name="${basename}_${SUFFIX}"
    fi
    local output="${output_dir}/${output_name}.mp4"

    mkdir -p "$output_dir"

    if [[ -f "$output" ]]; then
        echo "Skip (exists): ${output_name}"
        return 0
    fi

    local topic_to_use="$TOPIC"
    if [[ -z "$topic_to_use" ]]; then
        topic_to_use=$(detect_topic "$mcap_file")
        if [[ -z "$topic_to_use" ]]; then
            echo "Error: No topic found in $basename"
            return 1
        fi
    fi

    echo "Exporting: ${output_name} (topic: $topic_to_use)"
    "$BINARY" export -i "$mcap_file" -t "$topic_to_use" -o "$output" --preset ultrafast
}

# Process all mcap files
for mcap_file in "$INPUT_DIR"/*.mcap; do
    [[ -f "$mcap_file" ]] || continue
    export_file "$mcap_file" || echo "Failed: $mcap_file"
done

echo "Done. Output: $OUTPUT_DIR"
