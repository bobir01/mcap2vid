use anyhow::{anyhow, Result};

use crate::ros2_msgs::{CompressedImage, Header, Time};

#[derive(Debug, Clone)]
pub struct CompressedVideo {
    pub timestamp: Time,
    pub frame_id: String,
    pub data: Vec<u8>,
    pub format: String,
}

/// Parse protobuf-encoded foxglove.CompressedImage.
///
/// Fields used by Foxglove:
/// 1. timestamp: google.protobuf.Timestamp
/// 2. frame_id: string
/// 3. data: bytes
/// 4. format: string
pub fn parse_compressed_image(data: &[u8]) -> Result<CompressedImage> {
    let mut cursor = ProtoCursor::new(data);
    let mut stamp = Time { sec: 0, nanosec: 0 };
    let mut frame_id = String::new();
    let mut image_data = Vec::new();
    let mut format = String::new();

    while let Some((field, wire_type)) = cursor.read_key()? {
        match (field, wire_type) {
            (1, WIRE_LEN) => {
                let timestamp = cursor.read_len_delimited()?;
                stamp = parse_timestamp(timestamp)?;
            }
            (2, WIRE_LEN) => {
                frame_id = String::from_utf8_lossy(cursor.read_len_delimited()?).to_string();
            }
            (3, WIRE_LEN) => {
                image_data = cursor.read_len_delimited()?.to_vec();
            }
            (4, WIRE_LEN) => {
                format = String::from_utf8_lossy(cursor.read_len_delimited()?).to_string();
            }
            _ => cursor.skip_field(wire_type)?,
        }
    }

    if image_data.is_empty() {
        return Err(anyhow!("foxglove.CompressedImage has empty data field"));
    }

    Ok(CompressedImage {
        header: Header { stamp, frame_id },
        format,
        data: image_data,
    })
}

pub fn parse_compressed_image_timestamp(data: &[u8]) -> Result<f64> {
    let mut cursor = ProtoCursor::new(data);
    while let Some((field, wire_type)) = cursor.read_key()? {
        match (field, wire_type) {
            (1, WIRE_LEN) => {
                return Ok(parse_timestamp(cursor.read_len_delimited()?)?.to_unix_timestamp());
            }
            _ => cursor.skip_field(wire_type)?,
        }
    }

    Err(anyhow!("foxglove.CompressedImage missing timestamp field"))
}

/// Parse protobuf-encoded foxglove.CompressedVideo.
///
/// Foxglove fields:
/// 1. timestamp: google.protobuf.Timestamp
/// 2. frame_id: string
/// 3. data: bytes
/// 4. format: string
pub fn parse_compressed_video(data: &[u8]) -> Result<CompressedVideo> {
    let mut cursor = ProtoCursor::new(data);
    let mut timestamp = Time { sec: 0, nanosec: 0 };
    let mut frame_id = String::new();
    let mut video_data = Vec::new();
    let mut format = String::new();

    while let Some((field, wire_type)) = cursor.read_key()? {
        match (field, wire_type) {
            (1, WIRE_LEN) => {
                timestamp = parse_timestamp(cursor.read_len_delimited()?)?;
            }
            (2, WIRE_LEN) => {
                frame_id = String::from_utf8_lossy(cursor.read_len_delimited()?).to_string();
            }
            (3, WIRE_LEN) => {
                video_data = cursor.read_len_delimited()?.to_vec();
            }
            (4, WIRE_LEN) => {
                format = String::from_utf8_lossy(cursor.read_len_delimited()?).to_string();
            }
            _ => cursor.skip_field(wire_type)?,
        }
    }

    if video_data.is_empty() {
        return Err(anyhow!("foxglove.CompressedVideo has empty data field"));
    }
    if format.trim().is_empty() {
        return Err(anyhow!("foxglove.CompressedVideo has empty format field"));
    }

    Ok(CompressedVideo {
        timestamp,
        frame_id,
        data: video_data,
        format,
    })
}

pub fn parse_compressed_video_timestamp(data: &[u8]) -> Result<f64> {
    let mut cursor = ProtoCursor::new(data);
    while let Some((field, wire_type)) = cursor.read_key()? {
        match (field, wire_type) {
            (1, WIRE_LEN) => {
                return Ok(parse_timestamp(cursor.read_len_delimited()?)?.to_unix_timestamp());
            }
            _ => cursor.skip_field(wire_type)?,
        }
    }

    Err(anyhow!("foxglove.CompressedVideo missing timestamp field"))
}

fn parse_timestamp(data: &[u8]) -> Result<Time> {
    let mut cursor = ProtoCursor::new(data);
    let mut sec = 0i64;
    let mut nanosec = 0i32;

    while let Some((field, wire_type)) = cursor.read_key()? {
        match (field, wire_type) {
            (1, WIRE_VARINT) => sec = cursor.read_varint()? as i64,
            (2, WIRE_VARINT) => nanosec = cursor.read_varint()? as i32,
            _ => cursor.skip_field(wire_type)?,
        }
    }

    Ok(Time {
        sec: sec
            .try_into()
            .map_err(|_| anyhow!("timestamp seconds out of i32 range"))?,
        nanosec: nanosec
            .try_into()
            .map_err(|_| anyhow!("timestamp nanos out of u32 range"))?,
    })
}

const WIRE_VARINT: u8 = 0;
const WIRE_64BIT: u8 = 1;
const WIRE_LEN: u8 = 2;
const WIRE_32BIT: u8 = 5;

struct ProtoCursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> ProtoCursor<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn read_key(&mut self) -> Result<Option<(u32, u8)>> {
        if self.pos >= self.data.len() {
            return Ok(None);
        }
        let key = self.read_varint()?;
        Ok(Some(((key >> 3) as u32, (key & 0x07) as u8)))
    }

    fn read_varint(&mut self) -> Result<u64> {
        let mut result = 0u64;
        for shift in (0..64).step_by(7) {
            if self.pos >= self.data.len() {
                return Err(anyhow!("truncated protobuf varint"));
            }
            let byte = self.data[self.pos];
            self.pos += 1;
            result |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(result);
            }
        }
        Err(anyhow!("protobuf varint too long"))
    }

    fn read_len_delimited(&mut self) -> Result<&'a [u8]> {
        let len = self.read_varint()? as usize;
        let end = self
            .pos
            .checked_add(len)
            .ok_or_else(|| anyhow!("protobuf length overflow"))?;
        if end > self.data.len() {
            return Err(anyhow!("truncated protobuf length-delimited field"));
        }
        let value = &self.data[self.pos..end];
        self.pos = end;
        Ok(value)
    }

    fn skip_field(&mut self, wire_type: u8) -> Result<()> {
        match wire_type {
            WIRE_VARINT => {
                self.read_varint()?;
            }
            WIRE_64BIT => self.skip_bytes(8)?,
            WIRE_LEN => {
                let len = self.read_varint()? as usize;
                self.skip_bytes(len)?;
            }
            WIRE_32BIT => self.skip_bytes(4)?,
            other => return Err(anyhow!("unsupported protobuf wire type {}", other)),
        }
        Ok(())
    }

    fn skip_bytes(&mut self, len: usize) -> Result<()> {
        self.pos = self
            .pos
            .checked_add(len)
            .ok_or_else(|| anyhow!("protobuf skip overflow"))?;
        if self.pos > self.data.len() {
            return Err(anyhow!("truncated protobuf field"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_compressed_image, parse_compressed_video};

    #[test]
    fn parses_foxglove_compressed_image() {
        let mut timestamp = Vec::new();
        timestamp.extend_from_slice(&[0x08, 0x7b]); // seconds = 123
        timestamp.extend_from_slice(&[0x10, 0x80, 0xca, 0xb5, 0xee, 0x01]); // nanos = 500000000

        let mut msg = Vec::new();
        msg.extend_from_slice(&[0x0a, timestamp.len() as u8]);
        msg.extend_from_slice(&timestamp);
        msg.extend_from_slice(&[0x12, 0x03]);
        msg.extend_from_slice(b"cam");
        msg.extend_from_slice(&[0x1a, 0x03]);
        msg.extend_from_slice(&[1, 2, 3]);
        msg.extend_from_slice(&[0x22, 0x04]);
        msg.extend_from_slice(b"jpeg");

        let parsed = parse_compressed_image(&msg).unwrap();
        assert_eq!(parsed.header.stamp.sec, 123);
        assert_eq!(parsed.header.stamp.nanosec, 500_000_000);
        assert_eq!(parsed.header.frame_id, "cam");
        assert_eq!(parsed.format, "jpeg");
        assert_eq!(parsed.data, vec![1, 2, 3]);
    }

    #[test]
    fn parses_foxglove_compressed_video() {
        let mut timestamp = Vec::new();
        timestamp.extend_from_slice(&[0x08, 0x7b]); // seconds = 123
        timestamp.extend_from_slice(&[0x10, 0x80, 0xca, 0xb5, 0xee, 0x01]); // nanos = 500000000

        let mut msg = Vec::new();
        msg.extend_from_slice(&[0x0a, timestamp.len() as u8]);
        msg.extend_from_slice(&timestamp);
        msg.extend_from_slice(&[0x12, 0x03]);
        msg.extend_from_slice(b"cam");
        msg.extend_from_slice(&[0x1a, 0x04]);
        msg.extend_from_slice(&[0, 0, 0, 1]);
        msg.extend_from_slice(&[0x22, 0x04]);
        msg.extend_from_slice(b"h264");

        let parsed = parse_compressed_video(&msg).unwrap();
        assert_eq!(parsed.timestamp.sec, 123);
        assert_eq!(parsed.timestamp.nanosec, 500_000_000);
        assert_eq!(parsed.frame_id, "cam");
        assert_eq!(parsed.format, "h264");
        assert_eq!(parsed.data, vec![0, 0, 0, 1]);
    }
}
