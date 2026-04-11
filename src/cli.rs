use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "mcap2vid")]
#[command(about = "High-performance MCAP to MP4 video extractor with timestamp embedding")]
#[command(version)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand)]
pub enum Commands {
    /// List available video topics in an MCAP file
    List {
        /// Input MCAP file path or s3c:// URL
        #[arg(short, long)]
        input: String,
    },
    /// Export video from MCAP to MP4
    Export {
        /// Input MCAP file path or s3c:// URL
        #[arg(short, long)]
        input: String,

        /// Video topic to extract (e.g., /camera/image_raw)
        #[arg(short, long)]
        topic: String,

        /// Output MP4 file path, or "-" for stdout (zero disk writes)
        #[arg(short, long)]
        output: String,

        /// Suffix appended to the output filename (e.g., "ego" produces name_ego.mp4)
        #[arg(short, long)]
        suffix: Option<String>,

        /// Number of threads for parallel processing (0 = auto-detect)
        #[arg(long, default_value = "0")]
        threads: usize,

        /// FFmpeg encoding preset (ultrafast, fast, medium, slow)
        #[arg(long, default_value = "medium")]
        preset: String,

        /// Constant Rate Factor for quality (0-51, lower = better quality)
        #[arg(long, default_value = "18")]
        crf: u8,

        /// Maximum bad frames always tolerated regardless of percentage.
        /// First N decode failures never trip the threshold — set to 0 to
        /// enforce the percentage immediately.
        #[arg(long, default_value = "10")]
        max_bad_frames: usize,

        /// Maximum bad frames as percent of total (beyond --max-bad-frames).
        /// When `bad_count > --max-bad-frames` and `bad_count / total > P%`,
        /// the export aborts with a non-zero exit code.
        #[arg(long, default_value = "1.0")]
        max_bad_frames_pct: f64,
    },
    /// Verify and read embedded timestamps from an MP4 file
    Verify {
        /// MP4 file path
        #[arg(short, long)]
        input: PathBuf,

        /// Show all timestamps (default: first and last 5)
        #[arg(long)]
        all: bool,
    },
    /// Export metadata from MCAP file (CameraInfo, TF)
    Metadata {
        #[command(subcommand)]
        action: MetadataAction,
    },
}

#[derive(Subcommand)]
pub enum MetadataAction {
    /// List available metadata topics (CameraInfo, TF)
    List {
        /// Input MCAP file path or s3c:// URL
        #[arg(short, long)]
        input: String,
    },
    /// Export CameraInfo to JSON
    CameraInfo {
        /// Input MCAP file path or s3c:// URL
        #[arg(short, long)]
        input: String,

        /// CameraInfo topic to extract
        #[arg(short, long)]
        topic: String,

        /// Output JSON file path (omit for stdout)
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Export only the first message (default: all)
        #[arg(long)]
        first_only: bool,

        /// Pretty-print JSON output
        #[arg(long)]
        pretty: bool,
    },
    /// Export TF transforms to JSON
    Tf {
        /// Input MCAP file path or s3c:// URL
        #[arg(short, long)]
        input: String,

        /// TF topic to extract (default: /tf)
        #[arg(short, long, default_value = "/tf")]
        topic: String,

        /// Output JSON file path (omit for stdout)
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Filter by parent frame_id
        #[arg(long)]
        parent_frame: Option<String>,

        /// Filter by child frame_id
        #[arg(long)]
        child_frame: Option<String>,

        /// Pretty-print JSON output
        #[arg(long)]
        pretty: bool,
    },
}
