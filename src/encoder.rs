use anyhow::{anyhow, Result};
use std::io::{BufWriter, Read, Write};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};

use crate::decoder::DecodedFrame;

/// FFmpeg encoder configuration
#[derive(Debug, Clone)]
pub struct EncoderConfig {
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    pub preset: String,
    pub crf: u8,
    pub threads: usize,
}

/// FFmpeg video encoder using subprocess
pub struct FfmpegEncoder {
    child: Child,
    stdin_writer: Option<BufWriter<std::process::ChildStdin>>,
    config: EncoderConfig,
    expected_frame_size: usize,
    frames_written: usize,
    stderr_buf: Arc<Mutex<Vec<u8>>>,
    stderr_thread: Option<JoinHandle<()>>,
}

impl FfmpegEncoder {
    /// Start FFmpeg encoder process
    pub fn new<P: AsRef<Path>>(output_path: P, config: EncoderConfig) -> Result<Self> {
        let threads_arg = if config.threads == 0 {
            "0".to_string() // Auto-detect
        } else {
            config.threads.to_string()
        };

        let expected_frame_size = (config.width as usize) * (config.height as usize) * 3;

        let mut cmd = Command::new("ffmpeg");
        cmd.args([
            "-y", // Overwrite output
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", &format!("{}x{}", config.width, config.height),
            "-r", &format!("{}", config.fps),
            "-i", "-", // Read from stdin
            "-c:v", "libx264",
            "-preset", &config.preset,
            "-crf", &config.crf.to_string(),
            "-pix_fmt", "yuv420p", // Required for compatibility
            "-threads", &threads_arg,
            "-movflags", "+faststart", // Enable streaming
        ]);
        cmd.arg(output_path.as_ref());

        cmd.stdin(Stdio::piped());
        cmd.stdout(Stdio::null());
        cmd.stderr(Stdio::piped());

        let mut child = cmd.spawn().map_err(|e| {
            anyhow!(
                "Failed to start FFmpeg. Is it installed? Error: {}",
                e
            )
        })?;

        let stdin_writer = BufWriter::new(child.stdin.take().unwrap());
        let (stderr_buf, stderr_thread) = Self::spawn_stderr_drain(&mut child);

        Ok(Self {
            child,
            stdin_writer: Some(stdin_writer),
            config,
            expected_frame_size,
            frames_written: 0,
            stderr_buf,
            stderr_thread: Some(stderr_thread),
        })
    }

    /// Spawn a background thread that continuously drains FFmpeg's stderr
    /// to prevent pipe buffer deadlock.
    fn spawn_stderr_drain(child: &mut Child) -> (Arc<Mutex<Vec<u8>>>, JoinHandle<()>) {
        let stderr_buf = Arc::new(Mutex::new(Vec::new()));
        let buf_clone = Arc::clone(&stderr_buf);
        let mut stderr = child.stderr.take().expect("stderr was piped");

        let handle = thread::spawn(move || {
            let mut tmp = [0u8; 4096];
            loop {
                match stderr.read(&mut tmp) {
                    Ok(0) => break, // EOF — FFmpeg exited
                    Ok(n) => {
                        if let Ok(mut buf) = buf_clone.lock() {
                            // Keep only the last 64KB to bound memory
                            if buf.len() + n > 65536 {
                                let drain = buf.len() + n - 65536;
                                buf.drain(..drain);
                            }
                            buf.extend_from_slice(&tmp[..n]);
                        }
                    }
                    Err(_) => break,
                }
            }
        });

        (stderr_buf, handle)
    }

    /// Write a decoded frame to FFmpeg
    pub fn write_frame(&mut self, frame: &DecodedFrame) -> Result<()> {
        // Validate frame size
        if frame.rgb_data.len() != self.expected_frame_size {
            return Err(anyhow!(
                "Frame {} size mismatch: got {} bytes, expected {} ({}x{}x3). Frame dimensions: {}x{}",
                self.frames_written,
                frame.rgb_data.len(),
                self.expected_frame_size,
                self.config.width,
                self.config.height,
                frame.width,
                frame.height
            ));
        }

        let writer = self
            .stdin_writer
            .as_mut()
            .ok_or_else(|| anyhow!("FFmpeg stdin not available"))?;

        match writer.write_all(&frame.rgb_data) {
            Ok(()) => {
                self.frames_written += 1;
                Ok(())
            }
            Err(e) => {
                // Read captured stderr from drain thread
                let stderr_msg = self.get_stderr();
                Err(anyhow!(
                    "Failed to write frame {} to FFmpeg: {}. FFmpeg stderr: {}",
                    self.frames_written,
                    e,
                    stderr_msg
                ))
            }
        }
    }

    /// Get captured stderr output from the drain thread
    fn get_stderr(&self) -> String {
        match self.stderr_buf.lock() {
            Ok(buf) if !buf.is_empty() => String::from_utf8_lossy(&buf).to_string(),
            _ => "No stderr available".to_string(),
        }
    }

    /// Start FFmpeg encoder that outputs to stdout as fragmented MP4 (for streaming)
    pub fn new_stdout(config: EncoderConfig) -> Result<Self> {
        let threads_arg = if config.threads == 0 {
            "0".to_string()
        } else {
            config.threads.to_string()
        };

        let expected_frame_size = (config.width as usize) * (config.height as usize) * 3;

        let mut cmd = Command::new("ffmpeg");
        cmd.args([
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", &format!("{}x{}", config.width, config.height),
            "-r", &format!("{}", config.fps),
            "-i", "-",
            "-c:v", "libx264",
            "-preset", &config.preset,
            "-crf", &config.crf.to_string(),
            "-pix_fmt", "yuv420p",
            "-threads", &threads_arg,
            "-f", "mp4",
            "-movflags", "frag_keyframe+empty_moov+default_base_moof",
            "pipe:1",
        ]);

        cmd.stdin(Stdio::piped());
        cmd.stdout(Stdio::inherit()); // FFmpeg stdout → process stdout (the video stream)
        cmd.stderr(Stdio::piped());

        let mut child = cmd.spawn().map_err(|e| {
            anyhow!(
                "Failed to start FFmpeg. Is it installed? Error: {}",
                e
            )
        })?;

        let stdin_writer = BufWriter::new(child.stdin.take().unwrap());
        let (stderr_buf, stderr_thread) = Self::spawn_stderr_drain(&mut child);

        Ok(Self {
            child,
            stdin_writer: Some(stdin_writer),
            config,
            expected_frame_size,
            frames_written: 0,
            stderr_buf,
            stderr_thread: Some(stderr_thread),
        })
    }

    /// Finish encoding and wait for FFmpeg to complete
    pub fn finish(mut self) -> Result<()> {
        // Flush buffered writes, then close stdin to signal end of input
        if let Some(writer) = self.stdin_writer.take() {
            drop(writer);
        }

        let status = self.child.wait()?;

        // Join the stderr drain thread so we have the full output
        if let Some(handle) = self.stderr_thread.take() {
            let _ = handle.join();
        }

        if !status.success() {
            let stderr = self.get_stderr();
            return Err(anyhow!("FFmpeg encoding failed: {}", stderr));
        }

        Ok(())
    }
}

/// Calculate appropriate FPS from frame timestamps
pub fn calculate_fps(frames: &[DecodedFrame]) -> f64 {
    if frames.len() < 2 {
        return 30.0; // Default
    }

    let first_ts = frames.first().unwrap().timestamp;
    let last_ts = frames.last().unwrap().timestamp;
    let duration = last_ts - first_ts;

    if duration <= 0.0 {
        return 30.0;
    }

    let fps = (frames.len() - 1) as f64 / duration;

    // Clamp to reasonable range
    fps.clamp(1.0, 120.0)
}

/// Get frame dimensions (assumes all frames have same dimensions)
pub fn get_frame_dimensions(frames: &[DecodedFrame]) -> Option<(u32, u32)> {
    frames.first().map(|f| (f.width, f.height))
}

/// Validate that all frames have consistent dimensions
pub fn validate_frame_dimensions(frames: &[DecodedFrame]) -> Result<()> {
    if frames.is_empty() {
        return Ok(());
    }

    let (expected_width, expected_height) = (frames[0].width, frames[0].height);
    let expected_size = (expected_width as usize) * (expected_height as usize) * 3;

    for (i, frame) in frames.iter().enumerate() {
        if frame.width != expected_width || frame.height != expected_height {
            return Err(anyhow!(
                "Frame {} has inconsistent dimensions: {}x{}, expected {}x{}",
                i,
                frame.width,
                frame.height,
                expected_width,
                expected_height
            ));
        }
        if frame.rgb_data.len() != expected_size {
            return Err(anyhow!(
                "Frame {} has wrong data size: {} bytes, expected {} for {}x{}",
                i,
                frame.rgb_data.len(),
                expected_size,
                expected_width,
                expected_height
            ));
        }
    }

    Ok(())
}
