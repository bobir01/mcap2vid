mod cli;
mod decoder;
mod encoder;
mod mcap_index;
mod mcap_reader;
mod ros2_msgs;
mod s3_reader;
mod timestamp;

use anyhow::Result;
use clap::Parser;
use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};
use rayon::ThreadPoolBuilder;
use std::path::{Path, PathBuf};

use cli::{Cli, Commands, MetadataAction};
use decoder::{decode_frame, decode_frames_parallel};
use encoder::{EncoderConfig, FfmpegEncoder};
use mcap_reader::{McapReader, VideoFrame};
use s3_reader::{is_s3_url, S3Client, S3Url};
use timestamp::{embed_timestamps, read_timestamps};

/// Max frames decoded in memory at once. At 1080p (~6 MB/frame) this is ~3 GB peak.
const BATCH_SIZE: usize = 500;

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::List { input } => list_topics(&input),
        Commands::Export {
            input,
            topic,
            output,
            suffix,
            threads,
            preset,
            crf,
        } => export_video(
            &input,
            &topic,
            &output,
            suffix.as_deref(),
            threads,
            &preset,
            crf,
        ),
        Commands::Verify { input, all } => verify_timestamps(&input, all),
        Commands::Metadata { action } => handle_metadata(action),
    }
}

/// Helper for directing output to stderr vs stdout
struct Logger {
    to_stderr: bool,
}

impl Logger {
    fn log(&self, msg: &str) {
        if self.to_stderr {
            eprintln!("{}", msg);
        } else {
            println!("{}", msg);
        }
    }

    fn progress_bar(&self, total: u64) -> ProgressBar {
        let target = if self.to_stderr {
            ProgressDrawTarget::stderr()
        } else {
            ProgressDrawTarget::stdout()
        };
        ProgressBar::with_draw_target(Some(total), target)
    }

    fn spinner(&self) -> ProgressBar {
        let target = if self.to_stderr {
            ProgressDrawTarget::stderr()
        } else {
            ProgressDrawTarget::stdout()
        };
        ProgressBar::with_draw_target(None, target)
    }
}

fn handle_metadata(action: MetadataAction) -> Result<()> {
    match action {
        MetadataAction::List { input } => list_metadata_topics(&input),
        MetadataAction::CameraInfo {
            input,
            topic,
            output,
            first_only,
            pretty,
        } => export_camera_info(&input, &topic, output.as_deref(), first_only, pretty),
        MetadataAction::Tf {
            input,
            topic,
            output,
            parent_frame,
            child_frame,
            pretty,
        } => export_tf(
            &input,
            &topic,
            output.as_deref(),
            parent_frame.as_deref(),
            child_frame.as_deref(),
            pretty,
        ),
    }
}

fn list_topics(input: &str) -> Result<()> {
    let topics = if is_s3_url(input) {
        eprintln!("Reading index from: {}", input);
        let s3_url = S3Url::parse(input)?;
        let client = S3Client::from_env()?;
        client.list_video_topics(&s3_url)?
    } else {
        println!("Opening MCAP: {}", input);
        let reader = McapReader::open(Path::new(input))?;
        reader.list_video_topics()?
    };

    if topics.is_empty() {
        println!("No video topics found in this MCAP file.");
        return Ok(());
    }

    println!("\nAvailable video topics:");
    println!("{:-<80}", "");
    println!(
        "{:<50} {:<20} {:>8}",
        "Topic", "Type", "Messages"
    );
    println!("{:-<80}", "");

    for topic in topics {
        println!(
            "{:<50} {:<20} {:>8}",
            topic.name,
            topic.schema_name
                .split('/')
                .last()
                .unwrap_or(&topic.schema_name),
            topic.message_count
        );
    }

    Ok(())
}

fn export_video(
    input: &str,
    topic: &str,
    output: &str,
    suffix: Option<&str>,
    threads: usize,
    preset: &str,
    crf: u8,
) -> Result<()> {
    let to_stdout = output == "-";
    let log = Logger {
        to_stderr: to_stdout,
    };

    // Configure thread pool
    if threads > 0 {
        ThreadPoolBuilder::new()
            .num_threads(threads)
            .build_global()?;
    }

    let num_threads = rayon::current_num_threads();
    log.log(&format!(
        "Using {} threads for parallel processing",
        num_threads
    ));

    // Phase 1: Get video properties + prepare frame source.
    // S3: all compressed frames loaded (selective by chunk).
    // Local: only scan metadata — frames streamed in batches later.
    enum FrameSource {
        S3(Vec<VideoFrame>),
        Local(McapReader),
    }

    let (source, total, fps, width, height);

    if is_s3_url(input) {
        log.log(&format!("Reading from S3: {}", input));
        let s3_url = S3Url::parse(input)?;
        let client = S3Client::from_env()?;
        let frames = client.extract_frames(&s3_url, topic)?;

        if frames.is_empty() {
            anyhow::bail!("No frames found for topic '{}'", topic);
        }

        let first_decoded = decode_frame(&frames[0])?;
        width = first_decoded.width;
        height = first_decoded.height;
        drop(first_decoded);

        total = frames.len();
        fps = if frames.len() >= 2 {
            let duration = frames.last().unwrap().timestamp - frames[0].timestamp;
            if duration > 0.0 {
                ((frames.len() - 1) as f64 / duration).clamp(1.0, 120.0)
            } else {
                30.0
            }
        } else {
            30.0
        };

        source = FrameSource::S3(frames);
    } else {
        log.log(&format!("Opening MCAP: {}", input));
        let reader = McapReader::open(Path::new(input))?;

        log.log(&format!("Scanning topic: {}", topic));
        let scan = reader.scan_topic(topic)?;
        total = scan.count;
        log.log(&format!("Found {} frames", total));

        fps = if scan.count >= 2 {
            let duration_s =
                (scan.last_log_time_ns - scan.first_log_time_ns) as f64 / 1_000_000_000.0;
            if duration_s > 0.0 {
                ((scan.count - 1) as f64 / duration_s).clamp(1.0, 120.0)
            } else {
                30.0
            }
        } else {
            30.0
        };

        let first_frame = reader.extract_first_frame(topic)?;
        let first_decoded = decode_frame(&first_frame)?;
        width = first_decoded.width;
        height = first_decoded.height;
        drop(first_decoded);
        drop(first_frame);

        source = FrameSource::Local(reader);
    }

    log.log(&format!(
        "Video: {}x{} @ {:.2} FPS, {} frames",
        width, height, fps, total
    ));

    // Phase 2: Setup encoder
    let config = EncoderConfig {
        width,
        height,
        fps,
        preset: preset.to_string(),
        crf,
        threads,
    };

    let output_path = if !to_stdout {
        Some(if let Some(sfx) = suffix {
            let p = PathBuf::from(output);
            let stem = p.file_stem().unwrap_or_default().to_string_lossy();
            let ext = p
                .extension()
                .map(|e| e.to_string_lossy())
                .unwrap_or_default();
            p.with_file_name(format!("{}_{}.{}", stem, sfx, ext))
        } else {
            PathBuf::from(output)
        })
    } else {
        None
    };

    let mut encoder = if to_stdout {
        log.log(&format!(
            "Encoding to H.264 (preset: {}, CRF: {}) -> stdout",
            preset, crf
        ));
        FfmpegEncoder::new_stdout(config)?
    } else {
        log.log(&format!(
            "Encoding to H.264 (preset: {}, CRF: {})...",
            preset, crf
        ));
        FfmpegEncoder::new(output_path.as_ref().unwrap(), config)?
    };

    let num_batches = (total + BATCH_SIZE - 1) / BATCH_SIZE;
    log.log(&format!(
        "Processing {} frames in {} batches of {}...",
        total, num_batches, BATCH_SIZE
    ));

    let pb = log.progress_bar(total as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template(
                "{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta})",
            )
            .unwrap()
            .progress_chars("#>-"),
    );

    let mut timestamps = Vec::with_capacity(total);

    // Phase 3: Batched decode + encode
    // S3: frames already in memory, chunk them.
    // Local: stream from mmap — only one batch of compressed data in memory at a time.
    match source {
        FrameSource::S3(frames) => {
            for batch in frames.chunks(BATCH_SIZE) {
                let decoded = decode_frames_parallel(batch)?;
                for frame in &decoded {
                    encoder.write_frame(frame)?;
                    timestamps.push(frame.timestamp);
                    pb.inc(1);
                }
            }
        }
        FrameSource::Local(reader) => {
            reader.process_frames_batched(topic, BATCH_SIZE, |batch| {
                let decoded = decode_frames_parallel(batch)?;
                for frame in &decoded {
                    encoder.write_frame(frame)?;
                    timestamps.push(frame.timestamp);
                    pb.inc(1);
                }
                Ok(())
            })?;
        }
    }

    pb.finish_with_message("Encoding complete");
    encoder.finish()?;

    // Phase 4: Summary
    let first_ts = timestamps.first().unwrap_or(&0.0);
    let last_ts = timestamps.last().unwrap_or(&0.0);
    let duration = last_ts - first_ts;

    if to_stdout {
        eprintln!("\nExport complete!");
        eprintln!("  Duration: {:.2}s", duration);
        eprintln!("  Frames: {}", total);
        eprintln!("  Time range: {:.6} - {:.6}", first_ts, last_ts);
    } else {
        let output_path = output_path.unwrap();

        log.log(&format!(
            "Embedding {} timestamps into MP4...",
            timestamps.len()
        ));
        embed_timestamps(&output_path, &timestamps)?;

        println!("\nExport complete!");
        println!("  Output: {}", output_path.display());
        println!("  Duration: {:.2}s", duration);
        println!("  Frames: {}", total);
        println!("  Time range: {:.6} - {:.6}", first_ts, last_ts);
        println!("\nTimestamps stored in FTSS atom. Use 'verify' command to extract them.");
    }

    Ok(())
}

fn list_metadata_topics(input: &str) -> Result<()> {
    let topics = if is_s3_url(input) {
        eprintln!("Reading index from: {}", input);
        let s3_url = S3Url::parse(input)?;
        let client = S3Client::from_env()?;
        client.list_metadata_topics(&s3_url)?
    } else {
        println!("Opening MCAP: {}", input);
        let reader = McapReader::open(Path::new(input))?;
        reader.list_metadata_topics()?
    };

    if topics.is_empty() {
        println!("No metadata topics found in this MCAP file.");
        return Ok(());
    }

    println!("\nAvailable metadata topics:");
    println!("{:-<80}", "");
    println!(
        "{:<50} {:<20} {:>8}",
        "Topic", "Type", "Messages"
    );
    println!("{:-<80}", "");

    for topic in topics {
        println!(
            "{:<50} {:<20} {:>8}",
            topic.name,
            topic.schema_name
                .split('/')
                .last()
                .unwrap_or(&topic.schema_name),
            topic.message_count
        );
    }

    Ok(())
}

fn export_camera_info(
    input: &str,
    topic: &str,
    output: Option<&Path>,
    first_only: bool,
    pretty: bool,
) -> Result<()> {
    println!("Opening MCAP: {}", input);
    let reader = if is_s3_url(input) {
        let s3_url = S3Url::parse(input)?;
        let client = S3Client::from_env()?;
        let data = client.get_object(&s3_url)?;
        McapReader::from_bytes(data)
    } else {
        McapReader::open(Path::new(input))?
    };

    println!("Extracting CameraInfo from topic: {}", topic);
    let messages = reader.extract_camera_info(topic, first_only)?;

    #[derive(serde::Serialize)]
    struct CameraInfoOutput {
        topic: String,
        message_count: usize,
        messages: Vec<ros2_msgs::CameraInfo>,
    }

    let output_data = CameraInfoOutput {
        topic: topic.to_string(),
        message_count: messages.len(),
        messages,
    };

    let json = if pretty {
        serde_json::to_string_pretty(&output_data)?
    } else {
        serde_json::to_string(&output_data)?
    };

    if let Some(path) = output {
        std::fs::write(path, &json)?;
        println!("CameraInfo exported to: {}", path.display());
    } else {
        println!("{}", json);
    }

    Ok(())
}

fn export_tf(
    input: &str,
    topic: &str,
    output: Option<&Path>,
    parent_frame: Option<&str>,
    child_frame: Option<&str>,
    pretty: bool,
) -> Result<()> {
    println!("Opening MCAP: {}", input);
    let reader = if is_s3_url(input) {
        let s3_url = S3Url::parse(input)?;
        let client = S3Client::from_env()?;
        let data = client.get_object(&s3_url)?;
        McapReader::from_bytes(data)
    } else {
        McapReader::open(Path::new(input))?
    };

    println!("Extracting TF from topic: {}", topic);
    let messages = reader.extract_tf_messages(topic)?;

    let mut all_transforms: Vec<ros2_msgs::TransformStamped> = messages
        .into_iter()
        .flat_map(|msg| msg.transforms)
        .collect();

    if let Some(parent) = parent_frame {
        all_transforms.retain(|t| t.header.frame_id == parent);
    }
    if let Some(child) = child_frame {
        all_transforms.retain(|t| t.child_frame_id == child);
    }

    #[derive(serde::Serialize)]
    struct TfOutput {
        topic: String,
        transform_count: usize,
        transforms: Vec<ros2_msgs::TransformStamped>,
    }

    let output_data = TfOutput {
        topic: topic.to_string(),
        transform_count: all_transforms.len(),
        transforms: all_transforms,
    };

    let json = if pretty {
        serde_json::to_string_pretty(&output_data)?
    } else {
        serde_json::to_string(&output_data)?
    };

    if let Some(path) = output {
        std::fs::write(path, &json)?;
        println!("TF exported to: {}", path.display());
    } else {
        println!("{}", json);
    }

    Ok(())
}

fn verify_timestamps(input: &Path, show_all: bool) -> Result<()> {
    println!("Reading timestamps from: {}", input.display());

    let timestamps = read_timestamps(input)?;

    println!("\nFTSS Atom Contents:");
    println!("  Frame count: {}", timestamps.len());

    if timestamps.is_empty() {
        return Ok(());
    }

    let first_ts = timestamps.first().unwrap();
    let last_ts = timestamps.last().unwrap();
    let duration = last_ts - first_ts;

    println!("  Duration: {:.2}s", duration);
    println!("  Time range: {:.6} - {:.6}", first_ts, last_ts);

    if timestamps.len() > 1 {
        let avg_interval = duration / (timestamps.len() - 1) as f64;
        let fps = 1.0 / avg_interval;
        println!("  Average FPS: {:.2}", fps);
    }

    println!("\nTimestamps:");
    println!("{:-<60}", "");
    println!(
        "{:>8}  {:>20}  {:>20}",
        "Frame", "Unix Timestamp", "Delta (ms)"
    );
    println!("{:-<60}", "");

    if show_all {
        let mut prev_ts = *first_ts;
        for (i, &ts) in timestamps.iter().enumerate() {
            let delta_ms = (ts - prev_ts) * 1000.0;
            println!("{:>8}  {:>20.6}  {:>20.3}", i, ts, delta_ms);
            prev_ts = ts;
        }
    } else {
        let show_count = 5.min(timestamps.len());

        let mut prev_ts = *first_ts;
        for (i, &ts) in timestamps.iter().take(show_count).enumerate() {
            let delta_ms = (ts - prev_ts) * 1000.0;
            println!("{:>8}  {:>20.6}  {:>20.3}", i, ts, delta_ms);
            prev_ts = ts;
        }

        if timestamps.len() > show_count * 2 {
            println!("{:>8}  {:>20}  {:>20}", "...", "...", "...");
        }

        if timestamps.len() > show_count {
            let start_idx = timestamps.len().saturating_sub(show_count);
            prev_ts = if start_idx > 0 {
                timestamps[start_idx - 1]
            } else {
                *first_ts
            };

            for (i, &ts) in timestamps.iter().skip(start_idx).enumerate() {
                let delta_ms = (ts - prev_ts) * 1000.0;
                println!(
                    "{:>8}  {:>20.6}  {:>20.3}",
                    start_idx + i,
                    ts,
                    delta_ms
                );
                prev_ts = ts;
            }
        }
    }

    Ok(())
}
