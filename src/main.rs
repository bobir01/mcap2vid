mod cli;
mod decoder;
mod encoder;
mod mcap_reader;
mod ros2_msgs;
mod timestamp;

use anyhow::Result;
use clap::Parser;
use indicatif::{ProgressBar, ProgressStyle};
use rayon::ThreadPoolBuilder;

use cli::{Cli, Commands, MetadataAction};
use decoder::decode_frames_parallel;
use encoder::{calculate_fps, get_frame_dimensions, validate_frame_dimensions, EncoderConfig, FfmpegEncoder};
use mcap_reader::McapReader;
use timestamp::{embed_timestamps, read_timestamps};

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
        } => {
            let output = if let Some(ref sfx) = suffix {
                let stem = output.file_stem().unwrap_or_default().to_string_lossy();
                let ext = output.extension().map(|e| e.to_string_lossy()).unwrap_or_default();
                output.with_file_name(format!("{}_{}.{}", stem, sfx, ext))
            } else {
                output
            };
            export_video(&input, &topic, &output, threads, &preset, crf)
        }
        Commands::Verify { input, all } => verify_timestamps(&input, all),
        Commands::Metadata { action } => handle_metadata(action),
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

fn list_topics(input: &std::path::Path) -> Result<()> {
    println!("Opening MCAP file: {}", input.display());
    let reader = McapReader::open(input)?;

    let topics = reader.list_video_topics()?;

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
            topic.schema_name.split('/').last().unwrap_or(&topic.schema_name),
            topic.message_count
        );
    }

    Ok(())
}

fn export_video(
    input: &std::path::Path,
    topic: &str,
    output: &std::path::Path,
    threads: usize,
    preset: &str,
    crf: u8,
) -> Result<()> {
    // Configure thread pool
    if threads > 0 {
        ThreadPoolBuilder::new()
            .num_threads(threads)
            .build_global()?;
    }

    let num_threads = rayon::current_num_threads();
    println!("Using {} threads for parallel processing", num_threads);

    // Open MCAP file
    println!("Opening MCAP file: {}", input.display());
    let reader = McapReader::open(input)?;

    // Extract frames
    println!("Extracting frames from topic: {}", topic);
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.green} {msg}")
            .unwrap(),
    );
    pb.set_message("Reading MCAP messages...");

    let frames = reader.extract_frames(topic)?;
    pb.finish_with_message(format!("Found {} frames", frames.len()));

    if frames.is_empty() {
        anyhow::bail!("No frames found for topic '{}'", topic);
    }

    // Decode frames in parallel
    println!("Decoding {} frames in parallel...", frames.len());
    let pb = ProgressBar::new(frames.len() as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta})")
            .unwrap()
            .progress_chars("#>-"),
    );

    let decoded_frames = decode_frames_parallel(&frames)?;
    pb.finish_with_message("Decoding complete");

    // Validate all frames have consistent dimensions
    println!("Validating frame dimensions...");
    validate_frame_dimensions(&decoded_frames)?;

    // Get frame dimensions and calculate FPS
    let (width, height) = get_frame_dimensions(&decoded_frames)
        .ok_or_else(|| anyhow::anyhow!("No frames to encode"))?;
    let fps = calculate_fps(&decoded_frames);

    let expected_size = (width as usize) * (height as usize) * 3;
    println!(
        "Video: {}x{} @ {:.2} FPS, {} frames (frame size: {} bytes)",
        width,
        height,
        fps,
        decoded_frames.len(),
        expected_size
    );

    // Collect timestamps for embedding
    let timestamps: Vec<f64> = decoded_frames.iter().map(|f| f.timestamp).collect();

    // Encode with FFmpeg
    println!("Encoding to H.264 (preset: {}, CRF: {})...", preset, crf);
    let config = EncoderConfig {
        width,
        height,
        fps,
        preset: preset.to_string(),
        crf,
        threads,
    };

    let mut encoder = FfmpegEncoder::new(output, config)?;

    let pb = ProgressBar::new(decoded_frames.len() as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} ({eta})")
            .unwrap()
            .progress_chars("#>-"),
    );

    for frame in &decoded_frames {
        encoder.write_frame(frame)?;
        pb.inc(1);
    }

    pb.finish_with_message("Encoding complete");

    encoder.finish()?;
    println!("FFmpeg encoding finished");

    // Embed timestamps
    println!("Embedding {} timestamps into MP4...", timestamps.len());
    embed_timestamps(output, &timestamps)?;
    println!("Timestamps embedded successfully");

    // Summary
    let first_ts = timestamps.first().unwrap_or(&0.0);
    let last_ts = timestamps.last().unwrap_or(&0.0);
    let duration = last_ts - first_ts;

    println!("\nExport complete!");
    println!("  Output: {}", output.display());
    println!("  Duration: {:.2}s", duration);
    println!("  Frames: {}", decoded_frames.len());
    println!("  Time range: {:.6} - {:.6}", first_ts, last_ts);
    println!(
        "\nTimestamps stored in FTSS atom. Use 'verify' command to extract them."
    );

    Ok(())
}

fn list_metadata_topics(input: &std::path::Path) -> Result<()> {
    println!("Opening MCAP file: {}", input.display());
    let reader = McapReader::open(input)?;

    let topics = reader.list_metadata_topics()?;

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
            topic.schema_name.split('/').last().unwrap_or(&topic.schema_name),
            topic.message_count
        );
    }

    Ok(())
}

fn export_camera_info(
    input: &std::path::Path,
    topic: &str,
    output: Option<&std::path::Path>,
    first_only: bool,
    pretty: bool,
) -> Result<()> {
    println!("Opening MCAP file: {}", input.display());
    let reader = McapReader::open(input)?;

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
    input: &std::path::Path,
    topic: &str,
    output: Option<&std::path::Path>,
    parent_frame: Option<&str>,
    child_frame: Option<&str>,
    pretty: bool,
) -> Result<()> {
    println!("Opening MCAP file: {}", input.display());
    let reader = McapReader::open(input)?;

    println!("Extracting TF from topic: {}", topic);
    let messages = reader.extract_tf_messages(topic)?;

    // Flatten all transforms and apply filters
    let mut all_transforms: Vec<ros2_msgs::TransformStamped> = messages
        .into_iter()
        .flat_map(|msg| msg.transforms)
        .collect();

    // Apply filters
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

fn verify_timestamps(input: &std::path::Path, show_all: bool) -> Result<()> {
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
    println!("{:>8}  {:>20}  {:>20}", "Frame", "Unix Timestamp", "Delta (ms)");
    println!("{:-<60}", "");

    if show_all {
        let mut prev_ts = *first_ts;
        for (i, &ts) in timestamps.iter().enumerate() {
            let delta_ms = (ts - prev_ts) * 1000.0;
            println!("{:>8}  {:>20.6}  {:>20.3}", i, ts, delta_ms);
            prev_ts = ts;
        }
    } else {
        // Show first 5 and last 5
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
                println!("{:>8}  {:>20.6}  {:>20.3}", start_idx + i, ts, delta_ms);
                prev_ts = ts;
            }
        }
    }

    Ok(())
}
