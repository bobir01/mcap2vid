use anyhow::{anyhow, Result};
use image::ImageFormat;
use rayon::prelude::*;

use crate::mcap_reader::{FrameData, VideoFrame};

/// A decoded frame ready for encoding
#[derive(Debug, Clone)]
pub struct DecodedFrame {
    pub timestamp: f64,
    pub sequence: u64,
    pub width: u32,
    pub height: u32,
    pub rgb_data: Vec<u8>, // RGB24 format
}

/// Decode a single frame to RGB24 format
pub fn decode_frame(frame: &VideoFrame) -> Result<DecodedFrame> {
    match &frame.data {
        FrameData::Compressed { format, data } => decode_compressed(frame, format, data),
        FrameData::Raw {
            width,
            height,
            encoding,
            step,
            data,
        } => decode_raw(frame, *width, *height, *step, encoding, data),
    }
}

/// Decode compressed image (JPEG/PNG)
fn decode_compressed(frame: &VideoFrame, format: &str, data: &[u8]) -> Result<DecodedFrame> {
    // Determine image format from the format string
    let img_format = if format.contains("jpeg") || format.contains("jpg") {
        Some(ImageFormat::Jpeg)
    } else if format.contains("png") {
        Some(ImageFormat::Png)
    } else {
        None
    };

    let img = if let Some(fmt) = img_format {
        image::load_from_memory_with_format(data, fmt)?
    } else {
        // Try to guess format from data
        image::load_from_memory(data)?
    };

    let rgb = img.to_rgb8();
    let (width, height) = rgb.dimensions();

    Ok(DecodedFrame {
        timestamp: frame.timestamp,
        sequence: frame.sequence,
        width,
        height,
        rgb_data: rgb.into_raw(),
    })
}

/// Get bytes per pixel for an encoding
fn bytes_per_pixel(encoding: &str) -> Option<usize> {
    match encoding {
        "rgb8" | "RGB8" | "bgr8" | "BGR8" | "8UC3" => Some(3),
        "rgba8" | "RGBA8" | "bgra8" | "BGRA8" | "8UC4" => Some(4),
        "mono8" | "MONO8" | "8UC1" => Some(1),
        "16UC1" | "mono16" => Some(2),
        _ => None,
    }
}

/// Decode raw image data based on encoding
fn decode_raw(
    frame: &VideoFrame,
    width: u32,
    height: u32,
    step: u32,
    encoding: &str,
    data: &[u8],
) -> Result<DecodedFrame> {
    let bpp = bytes_per_pixel(encoding)
        .ok_or_else(|| anyhow!("Unsupported image encoding: {}", encoding))?;

    let row_size = (width as usize) * bpp;

    // Use step from message if valid, otherwise calculate from data
    let actual_stride = if step > 0 {
        step as usize
    } else if data.len() > 0 && height > 0 {
        // Calculate stride from data size
        data.len() / (height as usize)
    } else {
        row_size
    };

    // Validate we have enough data
    let min_data_size = actual_stride * (height as usize);
    if data.len() < min_data_size {
        return Err(anyhow!(
            "Insufficient image data: got {} bytes, need {} for {}x{} {} (stride: {})",
            data.len(),
            min_data_size,
            width,
            height,
            encoding,
            actual_stride
        ));
    }

    let output_size = (width as usize) * (height as usize) * 3;
    let mut rgb_data = Vec::with_capacity(output_size);

    // Process row by row to handle stride
    for row in 0..(height as usize) {
        let row_start = row * actual_stride;
        let row_data = &data[row_start..row_start + row_size];

        match encoding {
            "rgb8" | "RGB8" | "8UC3" => {
                // Already RGB, just copy
                rgb_data.extend_from_slice(row_data);
            }
            "bgr8" | "BGR8" => {
                // Convert BGR to RGB
                for chunk in row_data.chunks_exact(3) {
                    rgb_data.push(chunk[2]); // R
                    rgb_data.push(chunk[1]); // G
                    rgb_data.push(chunk[0]); // B
                }
            }
            "rgba8" | "RGBA8" | "8UC4" => {
                // Convert RGBA to RGB (drop alpha)
                for chunk in row_data.chunks_exact(4) {
                    rgb_data.push(chunk[0]); // R
                    rgb_data.push(chunk[1]); // G
                    rgb_data.push(chunk[2]); // B
                }
            }
            "bgra8" | "BGRA8" => {
                // Convert BGRA to RGB
                for chunk in row_data.chunks_exact(4) {
                    rgb_data.push(chunk[2]); // R
                    rgb_data.push(chunk[1]); // G
                    rgb_data.push(chunk[0]); // B
                }
            }
            "mono8" | "MONO8" | "8UC1" => {
                // Convert grayscale to RGB
                for &pixel in row_data {
                    rgb_data.push(pixel);
                    rgb_data.push(pixel);
                    rgb_data.push(pixel);
                }
            }
            "16UC1" | "mono16" => {
                // Convert 16-bit grayscale to 8-bit RGB
                for chunk in row_data.chunks_exact(2) {
                    // Take high byte for 8-bit representation
                    let pixel = chunk[1];
                    rgb_data.push(pixel);
                    rgb_data.push(pixel);
                    rgb_data.push(pixel);
                }
            }
            _ => unreachable!(),
        }
    }

    // Final validation
    if rgb_data.len() != output_size {
        return Err(anyhow!(
            "RGB conversion error: got {} bytes, expected {}",
            rgb_data.len(),
            output_size
        ));
    }

    Ok(DecodedFrame {
        timestamp: frame.timestamp,
        sequence: frame.sequence,
        width,
        height,
        rgb_data,
    })
}

/// Decode multiple frames in parallel using rayon
pub fn decode_frames_parallel(frames: &[VideoFrame]) -> Result<Vec<DecodedFrame>> {
    let results: Vec<Result<DecodedFrame>> = frames.par_iter().map(|f| decode_frame(f)).collect();

    // Collect results, propagating first error
    let mut decoded: Vec<DecodedFrame> = Vec::with_capacity(results.len());
    for result in results {
        decoded.push(result?);
    }

    // Sort by sequence to maintain order
    decoded.sort_by_key(|f| f.sequence);

    Ok(decoded)
}
