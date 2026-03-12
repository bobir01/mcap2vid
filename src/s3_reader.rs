use anyhow::{anyhow, Context, Result};
use hmac::{Hmac, Mac};
use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::io::Read;

use crate::mcap_index::{self, McapSummary};
use crate::mcap_reader::{FrameData, MetadataTopicInfo, TopicInfo, VideoFrame};
use crate::ros2_msgs::{
    parse_compressed_image, parse_image, ImageMessageType, MetadataMessageType,
};

type HmacSha256 = Hmac<Sha256>;

/// Parsed s3c:// URL
pub struct S3Url {
    pub bucket: String,
    pub key: String,
}

/// S3-compatible storage client using AWS Signature V4
pub struct S3Client {
    endpoint: String,
    access_key: String,
    secret_key: String,
    region: String,
}

impl S3Url {
    /// Parse an s3c:// URL into bucket and key
    ///
    /// Format: s3c://bucket/path/to/object.mcap
    pub fn parse(url: &str) -> Result<Self> {
        let rest = url
            .strip_prefix("s3c://")
            .ok_or_else(|| anyhow!("Invalid S3 URL: must start with s3c://"))?;

        let (bucket, key) = rest
            .split_once('/')
            .ok_or_else(|| anyhow!("Invalid S3 URL: must be s3c://bucket/key"))?;

        if bucket.is_empty() || key.is_empty() {
            return Err(anyhow!(
                "Invalid S3 URL: bucket and key must not be empty"
            ));
        }

        Ok(Self {
            bucket: bucket.to_string(),
            key: key.to_string(),
        })
    }
}

impl S3Client {
    /// Create a new S3 client from environment variables.
    ///
    /// Required: S3_ENDPOINT, S3_ACCESS_KEY, S3_SECRET_KEY
    pub fn from_env() -> Result<Self> {
        let endpoint = std::env::var("S3_ENDPOINT")
            .context("S3_ENDPOINT environment variable not set")?;
        let access_key = std::env::var("S3_ACCESS_KEY")
            .context("S3_ACCESS_KEY environment variable not set")?;
        let secret_key = std::env::var("S3_SECRET_KEY")
            .context("S3_SECRET_KEY environment variable not set")?;

        Ok(Self {
            endpoint: endpoint.trim_end_matches('/').to_string(),
            access_key,
            secret_key,
            region: "us-east-1".to_string(),
        })
    }

    // ---- High-level selective MCAP operations ----

    /// List video topics by reading only the MCAP summary (a few KB from the end of the file).
    pub fn list_video_topics(&self, s3_url: &S3Url) -> Result<Vec<TopicInfo>> {
        let summary = self.read_summary(s3_url)?;
        let schema_map = build_schema_map(&summary);

        let mut topics = Vec::new();
        for ch in &summary.channels {
            let schema_name = schema_map.get(&ch.schema_id).copied().unwrap_or("");
            let msg_type = ImageMessageType::from_schema_name(schema_name);
            if msg_type.is_some() {
                let count = summary
                    .channel_message_counts
                    .get(&ch.id)
                    .copied()
                    .unwrap_or(0);
                topics.push(TopicInfo {
                    name: ch.topic.clone(),
                    schema_name: schema_name.to_string(),
                    message_count: count as usize,
                    message_type: msg_type,
                });
            }
        }
        topics.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(topics)
    }

    /// List metadata topics by reading only the MCAP summary.
    pub fn list_metadata_topics(&self, s3_url: &S3Url) -> Result<Vec<MetadataTopicInfo>> {
        let summary = self.read_summary(s3_url)?;
        let schema_map = build_schema_map(&summary);

        let mut topics = Vec::new();
        for ch in &summary.channels {
            let schema_name = schema_map.get(&ch.schema_id).copied().unwrap_or("");
            if let Some(msg_type) = MetadataMessageType::from_schema_name(schema_name) {
                let count = summary
                    .channel_message_counts
                    .get(&ch.id)
                    .copied()
                    .unwrap_or(0);
                topics.push(MetadataTopicInfo {
                    name: ch.topic.clone(),
                    schema_name: schema_name.to_string(),
                    message_count: count as usize,
                    message_type: msg_type,
                });
            }
        }
        topics.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(topics)
    }

    /// Extract video frames for a specific topic, downloading only the relevant chunks.
    pub fn extract_frames(&self, s3_url: &S3Url, topic: &str) -> Result<Vec<VideoFrame>> {
        let summary = self.read_summary(s3_url)?;
        let schema_map = build_schema_map(&summary);

        // Find channel IDs for the target topic
        let channel_ids: Vec<u16> = summary
            .channels
            .iter()
            .filter(|c| c.topic == topic)
            .map(|c| c.id)
            .collect();

        if channel_ids.is_empty() {
            let available: Vec<&str> = summary.channels.iter().map(|c| c.topic.as_str()).collect();
            return Err(anyhow!(
                "Topic '{}' not found. Available topics: {:?}",
                topic,
                available
            ));
        }

        // Determine message type
        let ch = summary
            .channels
            .iter()
            .find(|c| c.topic == topic)
            .unwrap();
        let schema_name = schema_map.get(&ch.schema_id).copied().unwrap_or("");
        let message_type = ImageMessageType::from_schema_name(schema_name).ok_or_else(|| {
            anyhow!(
                "Topic '{}' is not an image topic (schema: {})",
                topic,
                schema_name
            )
        })?;

        // Find chunks containing the target channels
        let relevant_chunks: Vec<_> = summary
            .chunk_indexes
            .iter()
            .filter(|ci| ci.channel_ids.iter().any(|id| channel_ids.contains(id)))
            .collect();

        let total_chunks = summary.chunk_indexes.len();
        let total_bytes: u64 = relevant_chunks.iter().map(|c| c.chunk_length).sum();
        eprintln!(
            "Selective read: {}/{} chunks ({:.1} MB to download)",
            relevant_chunks.len(),
            total_chunks,
            total_bytes as f64 / 1_048_576.0
        );

        // Download and parse each relevant chunk
        let pb = ProgressBar::with_draw_target(
            Some(relevant_chunks.len() as u64),
            ProgressDrawTarget::stderr(),
        );
        pb.set_style(
            ProgressStyle::default_bar()
                .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} chunks ({eta})")
                .unwrap()
                .progress_chars("#>-"),
        );

        let mut frames = Vec::new();
        let mut sequence = 0u64;

        for ci in &relevant_chunks {
            let end = ci.chunk_start_offset + ci.chunk_length - 1;
            let chunk_data = self.get_object_range(s3_url, ci.chunk_start_offset, end)?;

            let raw_msgs =
                mcap_index::extract_messages_from_chunk(&chunk_data, &channel_ids)?;

            for msg in raw_msgs {
                let frame = match message_type {
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

            pb.inc(1);
        }

        pb.finish_and_clear();

        // Sort by timestamp
        frames.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap());

        eprintln!(
            "Extracted {} frames from {} chunks",
            frames.len(),
            relevant_chunks.len()
        );

        if frames.is_empty() {
            return Err(anyhow!("No frames found for topic '{}'", topic));
        }

        Ok(frames)
    }

    // ---- Low-level S3 operations ----

    /// Download an entire object from S3 into memory with progress (fallback for metadata export).
    pub fn get_object(&self, s3_url: &S3Url) -> Result<Vec<u8>> {
        let size = self.head_object(s3_url).unwrap_or(0);

        if size > 0 {
            eprintln!(
                "Downloading s3c://{}/{} ({:.1} MB)",
                s3_url.bucket,
                s3_url.key,
                size as f64 / 1_048_576.0
            );
        } else {
            eprintln!("Downloading s3c://{}/{}", s3_url.bucket, s3_url.key);
        }

        let url = format!(
            "{}/{}/{}",
            self.endpoint,
            s3_url.bucket,
            encode_uri_path(&s3_url.key)
        );
        let now = utc_now();
        let headers = self.sign_request("GET", &s3_url.bucket, &s3_url.key, &now);

        let mut req = ureq::get(&url);
        for (key, value) in &headers {
            req = req.set(key, value);
        }

        let resp = req.call().map_err(|e| match e {
            ureq::Error::Status(code, resp) => {
                let body = resp.into_string().unwrap_or_default();
                anyhow!("S3 GET failed (HTTP {}): {}", code, body)
            }
            ureq::Error::Transport(t) => {
                anyhow!("S3 connection error: {}", t)
            }
        })?;

        let pb = if size > 0 {
            let pb = ProgressBar::with_draw_target(Some(size), ProgressDrawTarget::stderr());
            pb.set_style(
                ProgressStyle::default_bar()
                    .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({bytes_per_sec})")
                    .unwrap()
                    .progress_chars("#>-"),
            );
            pb
        } else {
            let pb = ProgressBar::with_draw_target(None, ProgressDrawTarget::stderr());
            pb.set_style(
                ProgressStyle::default_spinner()
                    .template("{spinner:.green} [{elapsed_precise}] {bytes} ({bytes_per_sec})")
                    .unwrap(),
            );
            pb
        };

        let mut reader = pb.wrap_read(resp.into_reader());
        let mut data = Vec::with_capacity(if size > 0 { size as usize } else { 1024 * 1024 });
        reader.read_to_end(&mut data)?;
        pb.finish_and_clear();
        eprintln!("Downloaded {:.1} MB", data.len() as f64 / 1_048_576.0);

        Ok(data)
    }

    /// Read the MCAP summary section via two small range requests (footer + summary).
    fn read_summary(&self, s3_url: &S3Url) -> Result<McapSummary> {
        let file_size = self.head_object(s3_url)?;
        if file_size < mcap_index::footer_size() {
            return Err(anyhow!("File too small to be a valid MCAP ({} bytes)", file_size));
        }

        // Read footer (last 37 bytes)
        let tail_start = file_size - mcap_index::footer_size();
        let tail = self.get_object_range(s3_url, tail_start, file_size - 1)?;
        let footer = mcap_index::parse_footer(&tail)?;

        // Read summary section (from summary_start to just before footer)
        let summary_end = tail_start - 1; // byte before the footer record
        let summary_data = self.get_object_range(s3_url, footer.summary_start, summary_end)?;

        eprintln!(
            "Read index: {:.1} KB (from {:.1} GB file)",
            (tail.len() + summary_data.len()) as f64 / 1024.0,
            file_size as f64 / 1_073_741_824.0
        );

        mcap_index::parse_summary(&summary_data)
    }

    /// Download a byte range from an S3 object (inclusive range).
    fn get_object_range(&self, s3_url: &S3Url, start: u64, end: u64) -> Result<Vec<u8>> {
        let url = format!(
            "{}/{}/{}",
            self.endpoint,
            s3_url.bucket,
            encode_uri_path(&s3_url.key)
        );
        let now = utc_now();
        let headers = self.sign_request("GET", &s3_url.bucket, &s3_url.key, &now);
        let range_header = format!("bytes={}-{}", start, end);

        let mut req = ureq::get(&url);
        for (key, value) in &headers {
            req = req.set(key, value);
        }
        req = req.set("Range", &range_header);

        let resp = req.call().map_err(|e| match e {
            ureq::Error::Status(code, resp) => {
                let body = resp.into_string().unwrap_or_default();
                anyhow!("S3 range GET failed (HTTP {}): {}", code, body)
            }
            ureq::Error::Transport(t) => {
                anyhow!("S3 connection error: {}", t)
            }
        })?;

        let mut data = Vec::new();
        resp.into_reader().read_to_end(&mut data)?;
        Ok(data)
    }

    /// HEAD request to get object size.
    fn head_object(&self, s3_url: &S3Url) -> Result<u64> {
        let url = format!(
            "{}/{}/{}",
            self.endpoint,
            s3_url.bucket,
            encode_uri_path(&s3_url.key)
        );
        let now = utc_now();
        let headers = self.sign_request("HEAD", &s3_url.bucket, &s3_url.key, &now);

        let mut req = ureq::request("HEAD", &url);
        for (key, value) in &headers {
            req = req.set(key, value);
        }

        let resp = req.call().map_err(|e| match e {
            ureq::Error::Status(code, _) => {
                anyhow!("S3 HEAD failed (HTTP {})", code)
            }
            ureq::Error::Transport(t) => {
                anyhow!("S3 connection error: {}", t)
            }
        })?;

        let size = resp
            .header("content-length")
            .and_then(|s| s.parse::<u64>().ok())
            .unwrap_or(0);

        Ok(size)
    }

    /// Sign an S3 request using AWS Signature V4 (path-style addressing).
    fn sign_request(
        &self,
        method: &str,
        bucket: &str,
        key: &str,
        now: &UtcTimestamp,
    ) -> Vec<(String, String)> {
        let canonical_uri = format!("/{}/{}", bucket, encode_uri_path(key));
        let canonical_querystring = "";
        let payload_hash = "UNSIGNED-PAYLOAD";

        let host = extract_host(&self.endpoint);

        let canonical_headers = format!(
            "host:{}\nx-amz-content-sha256:{}\nx-amz-date:{}\n",
            host, payload_hash, now.amz_date
        );
        let signed_headers = "host;x-amz-content-sha256;x-amz-date";

        let canonical_request = format!(
            "{}\n{}\n{}\n{}\n{}\n{}",
            method,
            canonical_uri,
            canonical_querystring,
            canonical_headers,
            signed_headers,
            payload_hash
        );

        let credential_scope =
            format!("{}/{}/s3/aws4_request", now.date_stamp, self.region);
        let string_to_sign = format!(
            "AWS4-HMAC-SHA256\n{}\n{}\n{}",
            now.amz_date,
            credential_scope,
            hex::encode(Sha256::digest(canonical_request.as_bytes()))
        );

        let signing_key = self.derive_signing_key(&now.date_stamp);
        let signature = hex::encode(hmac_sha256(&signing_key, string_to_sign.as_bytes()));

        let authorization = format!(
            "AWS4-HMAC-SHA256 Credential={}/{}, SignedHeaders={}, Signature={}",
            self.access_key, credential_scope, signed_headers, signature
        );

        vec![
            ("Authorization".to_string(), authorization),
            ("x-amz-date".to_string(), now.amz_date.clone()),
            (
                "x-amz-content-sha256".to_string(),
                payload_hash.to_string(),
            ),
        ]
    }

    /// Derive the AWS SigV4 signing key.
    fn derive_signing_key(&self, date_stamp: &str) -> Vec<u8> {
        let k_date = hmac_sha256(
            format!("AWS4{}", self.secret_key).as_bytes(),
            date_stamp.as_bytes(),
        );
        let k_region = hmac_sha256(&k_date, self.region.as_bytes());
        let k_service = hmac_sha256(&k_region, b"s3");
        hmac_sha256(&k_service, b"aws4_request")
    }
}

/// Check if a string is an s3c:// URL.
pub fn is_s3_url(s: &str) -> bool {
    s.starts_with("s3c://")
}

// ---- Internal helpers ----

fn build_schema_map(summary: &McapSummary) -> HashMap<u16, &str> {
    summary
        .schemas
        .iter()
        .map(|s| (s.id, s.name.as_str()))
        .collect()
}

struct UtcTimestamp {
    amz_date: String,
    date_stamp: String,
}

fn utc_now() -> UtcTimestamp {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before epoch");
    let secs = now.as_secs();

    let days = secs / 86400;
    let time_of_day = secs % 86400;
    let hours = time_of_day / 3600;
    let minutes = (time_of_day % 3600) / 60;
    let seconds = time_of_day % 60;

    let (year, month, day) = days_to_ymd(days);

    UtcTimestamp {
        amz_date: format!(
            "{:04}{:02}{:02}T{:02}{:02}{:02}Z",
            year, month, day, hours, minutes, seconds
        ),
        date_stamp: format!("{:04}{:02}{:02}", year, month, day),
    }
}

fn days_to_ymd(days: u64) -> (u32, u32, u32) {
    let z = days + 719468;
    let era = z / 146097;
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    (y as u32, m as u32, d as u32)
}

fn hmac_sha256(key: &[u8], data: &[u8]) -> Vec<u8> {
    let mut mac = HmacSha256::new_from_slice(key).expect("HMAC key length error");
    mac.update(data);
    mac.finalize().into_bytes().to_vec()
}

fn encode_uri_path(s: &str) -> String {
    let mut encoded = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' | b'/' => {
                encoded.push(b as char);
            }
            _ => {
                encoded.push_str(&format!("%{:02X}", b));
            }
        }
    }
    encoded
}

fn extract_host(endpoint: &str) -> String {
    let without_scheme = endpoint
        .strip_prefix("https://")
        .or_else(|| endpoint.strip_prefix("http://"))
        .unwrap_or(endpoint);
    without_scheme
        .split('/')
        .next()
        .unwrap_or(without_scheme)
        .to_string()
}
