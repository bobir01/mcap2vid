use anyhow::{anyhow, Result};
use memmap2::Mmap;
use std::collections::HashMap;
use std::fs::File;
use std::path::Path;

use crate::ros2_msgs::{
    parse_camera_info, parse_compressed_image, parse_image, parse_tf_message, CameraInfo,
    ImageMessageType, MetadataMessageType, TFMessage,
};

/// Information about a video topic in an MCAP file
#[derive(Debug, Clone)]
pub struct TopicInfo {
    pub name: String,
    pub schema_name: String,
    pub message_count: usize,
    pub message_type: Option<ImageMessageType>,
}

/// Information about a metadata topic in an MCAP file
#[derive(Debug, Clone)]
pub struct MetadataTopicInfo {
    pub name: String,
    pub schema_name: String,
    pub message_count: usize,
    pub message_type: MetadataMessageType,
}

/// A video frame extracted from MCAP
#[derive(Debug, Clone)]
pub struct VideoFrame {
    pub timestamp: f64, // Unix timestamp with nanosecond precision
    pub sequence: u64,
    pub data: FrameData,
}

/// Frame data can be either raw or compressed
#[derive(Debug, Clone)]
pub enum FrameData {
    Raw {
        width: u32,
        height: u32,
        encoding: String,
        step: u32, // Row stride in bytes
        data: Vec<u8>,
    },
    Compressed {
        format: String,
        data: Vec<u8>,
    },
}

/// Frame count and time range from a quick topic scan (no payload parsing)
pub struct FrameScanInfo {
    pub count: usize,
    pub first_log_time_ns: u64,
    pub last_log_time_ns: u64,
}

/// Backing storage for MCAP data — either memory-mapped or in-memory
enum McapData {
    Mmap(Mmap),
    InMemory(Vec<u8>),
}

impl AsRef<[u8]> for McapData {
    fn as_ref(&self) -> &[u8] {
        match self {
            McapData::Mmap(m) => m.as_ref(),
            McapData::InMemory(v) => v.as_ref(),
        }
    }
}

/// MCAP file reader supporting both memory-mapped files and in-memory data
pub struct McapReader {
    data: McapData,
}

impl McapReader {
    /// Open an MCAP file with memory mapping for efficient access
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::open(path.as_ref())?;
        let mmap = unsafe { Mmap::map(&file)? };
        Ok(Self { data: McapData::Mmap(mmap) })
    }

    /// Create a reader from in-memory data (e.g., downloaded from S3)
    pub fn from_bytes(bytes: Vec<u8>) -> Self {
        Self { data: McapData::InMemory(bytes) }
    }

    /// List all topics that contain image data
    pub fn list_video_topics(&self) -> Result<Vec<TopicInfo>> {
        let mut topics = Vec::new();
        // Use topic name as key instead of channel id
        let mut topic_message_counts: HashMap<String, usize> = HashMap::new();
        let mut topic_schema: HashMap<String, String> = HashMap::new();

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            match record {
                Ok(msg) => {
                    let topic_name = msg.channel.topic.clone();
                    *topic_message_counts.entry(topic_name.clone()).or_insert(0) += 1;

                    if !topic_schema.contains_key(&topic_name) {
                        let schema_name = msg
                            .channel
                            .schema
                            .as_ref()
                            .map(|s| s.name.clone())
                            .unwrap_or_default();
                        topic_schema.insert(topic_name, schema_name);
                    }
                }
                Err(e) => {
                    eprintln!("Warning: Error reading message: {}", e);
                }
            }
        }

        for (topic_name, schema_name) in topic_schema {
            let msg_type = ImageMessageType::from_schema_name(&schema_name);
            if msg_type.is_some() {
                topics.push(TopicInfo {
                    name: topic_name.clone(),
                    schema_name,
                    message_count: *topic_message_counts.get(&topic_name).unwrap_or(&0),
                    message_type: msg_type,
                });
            }
        }

        // Sort by topic name
        topics.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(topics)
    }

    /// Extract all video frames from a specific topic
    pub fn extract_frames(&self, topic: &str) -> Result<Vec<VideoFrame>> {
        let mut frames = Vec::new();
        let mut message_type: Option<ImageMessageType> = None;
        let mut sequence = 0u64;

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            match record {
                Ok(msg) => {
                    if msg.channel.topic != topic {
                        continue;
                    }

                    // Determine message type from schema on first message
                    if message_type.is_none() {
                        let schema_name = msg
                            .channel
                            .schema
                            .as_ref()
                            .map(|s| s.name.as_str())
                            .unwrap_or("");
                        message_type = ImageMessageType::from_schema_name(schema_name);
                        if message_type.is_none() {
                            return Err(anyhow!(
                                "Topic '{}' does not contain image messages (schema: {})",
                                topic,
                                schema_name
                            ));
                        }
                    }

                    let frame = match message_type.unwrap() {
                        ImageMessageType::Image => {
                            let img = parse_image(&msg.data)?;
                            VideoFrame {
                                timestamp: img.header.stamp.to_unix_timestamp(),
                                sequence,
                                data: FrameData::Raw {
                                    width: img.width,
                                    height: img.height,
                                    encoding: img.encoding,
                                    step: img.step,
                                    data: img.data,
                                },
                            }
                        }
                        ImageMessageType::CompressedImage => {
                            let img = parse_compressed_image(&msg.data)?;
                            VideoFrame {
                                timestamp: img.header.stamp.to_unix_timestamp(),
                                sequence,
                                data: FrameData::Compressed {
                                    format: img.format,
                                    data: img.data,
                                },
                            }
                        }
                    };

                    frames.push(frame);
                    sequence += 1;
                }
                Err(e) => {
                    eprintln!("Warning: Error reading message: {}", e);
                }
            }
        }

        if frames.is_empty() {
            return Err(anyhow!("No frames found for topic '{}'", topic));
        }

        // Sort frames by timestamp to ensure correct order
        frames.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());

        Ok(frames)
    }

    /// Quick scan: count matching frames and get time range.
    /// Does NOT parse CDR payloads — only checks topic and records log_time.
    pub fn scan_topic(&self, topic: &str) -> Result<FrameScanInfo> {
        let mut count = 0usize;
        let mut first_ts = u64::MAX;
        let mut last_ts = 0u64;

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            if let Ok(msg) = record {
                if msg.channel.topic != topic {
                    continue;
                }
                count += 1;
                let t = msg.log_time;
                if t < first_ts {
                    first_ts = t;
                }
                if t > last_ts {
                    last_ts = t;
                }
            }
        }

        if count == 0 {
            return Err(anyhow!("No frames found for topic '{}'", topic));
        }

        Ok(FrameScanInfo {
            count,
            first_log_time_ns: first_ts,
            last_log_time_ns: last_ts,
        })
    }

    /// Extract only the first frame from a topic (for getting dimensions)
    pub fn extract_first_frame(&self, topic: &str) -> Result<VideoFrame> {
        for record in mcap::MessageStream::new(self.data.as_ref())? {
            if let Ok(msg) = record {
                if msg.channel.topic != topic {
                    continue;
                }

                let schema_name = msg
                    .channel
                    .schema
                    .as_ref()
                    .map(|s| s.name.as_str())
                    .unwrap_or("");
                let msg_type = ImageMessageType::from_schema_name(schema_name)
                    .ok_or_else(|| anyhow!("Topic '{}' is not an image topic", topic))?;

                return Ok(match msg_type {
                    ImageMessageType::Image => {
                        let img = parse_image(&msg.data)?;
                        VideoFrame {
                            timestamp: img.header.stamp.to_unix_timestamp(),
                            sequence: 0,
                            data: FrameData::Raw {
                                width: img.width,
                                height: img.height,
                                encoding: img.encoding,
                                step: img.step,
                                data: img.data,
                            },
                        }
                    }
                    ImageMessageType::CompressedImage => {
                        let img = parse_compressed_image(&msg.data)?;
                        VideoFrame {
                            timestamp: img.header.stamp.to_unix_timestamp(),
                            sequence: 0,
                            data: FrameData::Compressed {
                                format: img.format,
                                data: img.data,
                            },
                        }
                    }
                });
            }
        }
        Err(anyhow!("No frames found for topic '{}'", topic))
    }

    /// Stream frames in batches — only one batch of compressed data in memory at a time.
    /// Calls `on_batch` with each batch of up to `batch_size` frames.
    pub fn process_frames_batched<F>(
        &self,
        topic: &str,
        batch_size: usize,
        mut on_batch: F,
    ) -> Result<()>
    where
        F: FnMut(&[VideoFrame]) -> Result<()>,
    {
        let mut batch = Vec::with_capacity(batch_size);
        let mut message_type: Option<ImageMessageType> = None;
        let mut sequence = 0u64;

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            match record {
                Ok(msg) => {
                    if msg.channel.topic != topic {
                        continue;
                    }

                    if message_type.is_none() {
                        let schema_name = msg
                            .channel
                            .schema
                            .as_ref()
                            .map(|s| s.name.as_str())
                            .unwrap_or("");
                        message_type = ImageMessageType::from_schema_name(schema_name);
                        if message_type.is_none() {
                            return Err(anyhow!(
                                "Topic '{}' does not contain image messages (schema: {})",
                                topic,
                                schema_name
                            ));
                        }
                    }

                    let frame = match message_type.unwrap() {
                        ImageMessageType::Image => {
                            let img = parse_image(&msg.data)?;
                            VideoFrame {
                                timestamp: img.header.stamp.to_unix_timestamp(),
                                sequence,
                                data: FrameData::Raw {
                                    width: img.width,
                                    height: img.height,
                                    encoding: img.encoding,
                                    step: img.step,
                                    data: img.data,
                                },
                            }
                        }
                        ImageMessageType::CompressedImage => {
                            let img = parse_compressed_image(&msg.data)?;
                            VideoFrame {
                                timestamp: img.header.stamp.to_unix_timestamp(),
                                sequence,
                                data: FrameData::Compressed {
                                    format: img.format,
                                    data: img.data,
                                },
                            }
                        }
                    };

                    batch.push(frame);
                    sequence += 1;

                    if batch.len() >= batch_size {
                        on_batch(&batch)?;
                        batch.clear();
                    }
                }
                Err(e) => {
                    eprintln!("Warning: Error reading message: {}", e);
                }
            }
        }

        if !batch.is_empty() {
            on_batch(&batch)?;
        }

        Ok(())
    }

    /// List all topics that contain metadata (CameraInfo, TF)
    pub fn list_metadata_topics(&self) -> Result<Vec<MetadataTopicInfo>> {
        let mut topics = Vec::new();
        let mut topic_message_counts: HashMap<String, usize> = HashMap::new();
        let mut topic_schema: HashMap<String, String> = HashMap::new();

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            match record {
                Ok(msg) => {
                    let topic_name = msg.channel.topic.clone();
                    *topic_message_counts.entry(topic_name.clone()).or_insert(0) += 1;

                    if !topic_schema.contains_key(&topic_name) {
                        let schema_name = msg
                            .channel
                            .schema
                            .as_ref()
                            .map(|s| s.name.clone())
                            .unwrap_or_default();
                        topic_schema.insert(topic_name, schema_name);
                    }
                }
                Err(e) => {
                    eprintln!("Warning: Error reading message: {}", e);
                }
            }
        }

        for (topic_name, schema_name) in topic_schema {
            if let Some(msg_type) = MetadataMessageType::from_schema_name(&schema_name) {
                topics.push(MetadataTopicInfo {
                    name: topic_name.clone(),
                    schema_name,
                    message_count: *topic_message_counts.get(&topic_name).unwrap_or(&0),
                    message_type: msg_type,
                });
            }
        }

        // Sort by topic name
        topics.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(topics)
    }

    /// Extract CameraInfo messages from a specific topic
    pub fn extract_camera_info(&self, topic: &str, only_first: bool) -> Result<Vec<CameraInfo>> {
        let mut messages = Vec::new();

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            match record {
                Ok(msg) => {
                    if msg.channel.topic != topic {
                        continue;
                    }

                    let camera_info = parse_camera_info(&msg.data)?;
                    messages.push(camera_info);

                    if only_first {
                        break;
                    }
                }
                Err(e) => {
                    eprintln!("Warning: Error reading message: {}", e);
                }
            }
        }

        if messages.is_empty() {
            return Err(anyhow!("No CameraInfo messages found for topic '{}'", topic));
        }

        Ok(messages)
    }

    /// Extract TFMessage messages from a specific topic
    pub fn extract_tf_messages(&self, topic: &str) -> Result<Vec<TFMessage>> {
        let mut messages = Vec::new();

        for record in mcap::MessageStream::new(self.data.as_ref())? {
            match record {
                Ok(msg) => {
                    if msg.channel.topic != topic {
                        continue;
                    }

                    let tf_msg = parse_tf_message(&msg.data)?;
                    messages.push(tf_msg);
                }
                Err(e) => {
                    eprintln!("Warning: Error reading message: {}", e);
                }
            }
        }

        if messages.is_empty() {
            return Err(anyhow!("No TF messages found for topic '{}'", topic));
        }

        Ok(messages)
    }
}
