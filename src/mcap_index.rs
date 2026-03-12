//! Low-level MCAP binary format parsing for selective/indexed reading.
//!
//! Parses the MCAP footer, summary section, and individual chunk records
//! to enable reading only the data needed (e.g., specific topics) without
//! downloading or scanning the entire file.

use anyhow::{anyhow, Result};
use byteorder::{LittleEndian, ReadBytesExt};
use std::collections::HashMap;
use std::io::{Cursor, Read};

// MCAP record opcodes
const OP_SCHEMA: u8 = 0x03;
const OP_CHANNEL: u8 = 0x04;
const OP_MESSAGE: u8 = 0x05;
const OP_CHUNK: u8 = 0x06;
const OP_STATISTICS: u8 = 0x0B;
const OP_CHUNK_INDEX: u8 = 0x08;

const MCAP_MAGIC: &[u8] = &[0x89, b'M', b'C', b'A', b'P', b'0', b'\r', b'\n'];

/// Size of the MCAP footer record (opcode + length + content) + trailing magic
const FOOTER_PLUS_MAGIC: usize = 1 + 8 + 20 + 8; // 37 bytes

pub struct McapFooter {
    pub summary_start: u64,
}

pub struct ChannelInfo {
    pub id: u16,
    pub schema_id: u16,
    pub topic: String,
}

pub struct SchemaInfo {
    pub id: u16,
    pub name: String,
}

pub struct ChunkIndexInfo {
    pub chunk_start_offset: u64,
    pub chunk_length: u64,
    pub channel_ids: Vec<u16>,
}

pub struct McapSummary {
    pub channels: Vec<ChannelInfo>,
    pub schemas: Vec<SchemaInfo>,
    pub chunk_indexes: Vec<ChunkIndexInfo>,
    pub channel_message_counts: HashMap<u16, u64>,
}

pub struct RawMessage {
    pub log_time: u64, // nanoseconds
    pub data: Vec<u8>, // CDR-encoded payload
}

/// Parse the MCAP footer from the last [FOOTER_PLUS_MAGIC] bytes of the file.
pub fn parse_footer(tail: &[u8]) -> Result<McapFooter> {
    if tail.len() < FOOTER_PLUS_MAGIC {
        return Err(anyhow!("Data too short for MCAP footer"));
    }

    let start = tail.len() - FOOTER_PLUS_MAGIC;
    let record = &tail[start..];

    // Verify trailing magic
    if &record[29..37] != MCAP_MAGIC {
        return Err(anyhow!("Invalid MCAP magic at end of file"));
    }

    // Footer opcode
    if record[0] != 0x02 {
        return Err(anyhow!(
            "Expected footer opcode 0x02, got 0x{:02X}",
            record[0]
        ));
    }

    let mut cursor = Cursor::new(&record[9..]); // skip opcode(1) + length(8)
    let summary_start = cursor.read_u64::<LittleEndian>()?;

    if summary_start == 0 {
        return Err(anyhow!(
            "MCAP file has no summary section. Re-write with summary enabled, or use local file mode."
        ));
    }

    Ok(McapFooter { summary_start })
}

/// How many bytes to fetch from the end of the file to get the footer.
pub fn footer_size() -> u64 {
    FOOTER_PLUS_MAGIC as u64
}

/// Parse the summary section to extract channels, schemas, chunk indexes, and statistics.
pub fn parse_summary(data: &[u8]) -> Result<McapSummary> {
    let mut channels = Vec::new();
    let mut schemas = Vec::new();
    let mut chunk_indexes = Vec::new();
    let mut channel_message_counts = HashMap::new();

    let mut pos = 0usize;

    while pos + 9 <= data.len() {
        let opcode = data[pos];
        let content_len =
            u64::from_le_bytes(data[pos + 1..pos + 9].try_into().unwrap()) as usize;
        pos += 9;

        if pos + content_len > data.len() {
            break;
        }

        let record_data = &data[pos..pos + content_len];

        match opcode {
            OP_SCHEMA => {
                if let Ok(s) = parse_schema_record(record_data) {
                    schemas.push(s);
                }
            }
            OP_CHANNEL => {
                if let Ok(c) = parse_channel_record(record_data) {
                    channels.push(c);
                }
            }
            OP_CHUNK_INDEX => {
                if let Ok(ci) = parse_chunk_index_record(record_data) {
                    chunk_indexes.push(ci);
                }
            }
            OP_STATISTICS => {
                if let Ok(counts) = parse_statistics_channel_counts(record_data) {
                    channel_message_counts = counts;
                }
            }
            _ => {} // skip DataEnd, SummaryOffset, etc.
        }

        pos += content_len;
    }

    Ok(McapSummary {
        channels,
        schemas,
        chunk_indexes,
        channel_message_counts,
    })
}

/// Extract messages for specific channels from a downloaded chunk record.
///
/// `chunk_record` must start with the chunk opcode byte (0x06).
pub fn extract_messages_from_chunk(
    chunk_record: &[u8],
    channel_filter: &[u16],
) -> Result<Vec<RawMessage>> {
    if chunk_record.len() < 9 {
        return Err(anyhow!("Chunk record too short"));
    }

    if chunk_record[0] != OP_CHUNK {
        return Err(anyhow!(
            "Expected chunk opcode 0x06, got 0x{:02X}",
            chunk_record[0]
        ));
    }

    let mut cursor = Cursor::new(chunk_record);
    cursor.set_position(9); // skip opcode(1) + record_length(8)

    let _msg_start_time = cursor.read_u64::<LittleEndian>()?;
    let _msg_end_time = cursor.read_u64::<LittleEndian>()?;
    let _uncompressed_size = cursor.read_u64::<LittleEndian>()?;
    let _uncompressed_crc = cursor.read_u32::<LittleEndian>()?;

    // Compression string
    let comp_len = cursor.read_u32::<LittleEndian>()? as usize;
    let mut comp_buf = vec![0u8; comp_len];
    cursor.read_exact(&mut comp_buf)?;
    let compression = String::from_utf8_lossy(&comp_buf);

    // Compressed records data
    let records_length = cursor.read_u64::<LittleEndian>()? as usize;
    let data_start = cursor.position() as usize;

    if data_start + records_length > chunk_record.len() {
        return Err(anyhow!("Chunk records data extends beyond record boundary"));
    }

    let compressed = &chunk_record[data_start..data_start + records_length];

    // Decompress
    let decompressed = match compression.as_ref() {
        "" | "none" => compressed.to_vec(),
        "lz4" => {
            let mut decoder = lz4_flex::frame::FrameDecoder::new(compressed);
            let mut buf = Vec::new();
            decoder.read_to_end(&mut buf)?;
            buf
        }
        "zstd" => zstd::decode_all(compressed)?,
        other => {
            return Err(anyhow!("Unsupported chunk compression: {}", other));
        }
    };

    // Parse message records from decompressed data
    Ok(extract_messages_from_records(
        &decompressed,
        channel_filter,
    ))
}

// ---- Internal parsing helpers ----

fn parse_schema_record(data: &[u8]) -> Result<SchemaInfo> {
    let mut cursor = Cursor::new(data);
    let id = cursor.read_u16::<LittleEndian>()?;
    let name = read_mcap_string(&mut cursor)?;
    // skip encoding + data fields — we only need id and name
    Ok(SchemaInfo { id, name })
}

fn parse_channel_record(data: &[u8]) -> Result<ChannelInfo> {
    let mut cursor = Cursor::new(data);
    let id = cursor.read_u16::<LittleEndian>()?;
    let schema_id = cursor.read_u16::<LittleEndian>()?;
    let topic = read_mcap_string(&mut cursor)?;
    // skip message_encoding + metadata — we only need id, schema_id, topic
    Ok(ChannelInfo {
        id,
        schema_id,
        topic,
    })
}

fn parse_chunk_index_record(data: &[u8]) -> Result<ChunkIndexInfo> {
    let mut cursor = Cursor::new(data);
    let _msg_start_time = cursor.read_u64::<LittleEndian>()?;
    let _msg_end_time = cursor.read_u64::<LittleEndian>()?;
    let chunk_start_offset = cursor.read_u64::<LittleEndian>()?;
    let chunk_length = cursor.read_u64::<LittleEndian>()?;

    // message_index_offsets: Map<uint16, uint64>
    //   serialized as: entries_byte_length(u32) + entries
    //   each entry: channel_id(u16) + offset(u64) = 10 bytes
    let map_bytes = cursor.read_u32::<LittleEndian>()? as usize;
    let mut channel_ids = Vec::with_capacity(map_bytes / 10);
    let map_start = cursor.position() as usize;
    let map_end = map_start + map_bytes;

    if map_end <= data.len() {
        let mut pos = map_start;
        while pos + 10 <= map_end {
            let ch_id = u16::from_le_bytes([data[pos], data[pos + 1]]);
            channel_ids.push(ch_id);
            pos += 10; // skip channel_id(2) + offset(8)
        }
    }

    Ok(ChunkIndexInfo {
        chunk_start_offset,
        chunk_length,
        channel_ids,
    })
}

fn parse_statistics_channel_counts(data: &[u8]) -> Result<HashMap<u16, u64>> {
    if data.len() < 40 {
        return Ok(HashMap::new());
    }
    // Skip to channel_message_counts map:
    //   message_count(8) + schema_count(2) + channel_count(4) +
    //   attachment_count(4) + metadata_count(4) + chunk_count(4) +
    //   message_start_time(8) + message_end_time(8) = 42 bytes
    let mut cursor = Cursor::new(data);
    cursor.set_position(42);

    let map_bytes = cursor.read_u32::<LittleEndian>()? as usize;
    let map_start = cursor.position() as usize;
    let map_end = map_start + map_bytes;

    let mut counts = HashMap::new();
    if map_end <= data.len() {
        let mut pos = map_start;
        while pos + 10 <= map_end {
            let ch_id = u16::from_le_bytes([data[pos], data[pos + 1]]);
            let count = u64::from_le_bytes(data[pos + 2..pos + 10].try_into().unwrap());
            counts.insert(ch_id, count);
            pos += 10;
        }
    }

    Ok(counts)
}

fn read_mcap_string(cursor: &mut Cursor<&[u8]>) -> Result<String> {
    let len = cursor.read_u32::<LittleEndian>()? as usize;
    if len == 0 {
        return Ok(String::new());
    }
    let mut buf = vec![0u8; len];
    cursor.read_exact(&mut buf)?;
    Ok(String::from_utf8_lossy(&buf).to_string())
}

/// Fast message extraction from decompressed chunk records (no cursor overhead).
fn extract_messages_from_records(data: &[u8], channel_filter: &[u16]) -> Vec<RawMessage> {
    let mut messages = Vec::new();
    let mut pos = 0;

    while pos + 9 <= data.len() {
        let opcode = data[pos];
        let content_len = u64::from_le_bytes(
            data[pos + 1..pos + 9]
                .try_into()
                .unwrap_or([0u8; 8]),
        ) as usize;
        pos += 9;

        if pos + content_len > data.len() {
            break;
        }

        // Message record: opcode 0x05
        // Layout: channel_id(2) + sequence(4) + log_time(8) + publish_time(8) + data(rest)
        if opcode == OP_MESSAGE && content_len >= 22 {
            let ch_id = u16::from_le_bytes([data[pos], data[pos + 1]]);

            if channel_filter.contains(&ch_id) {
                let log_time = u64::from_le_bytes(
                    data[pos + 6..pos + 14].try_into().unwrap(),
                );
                let payload = data[pos + 22..pos + content_len].to_vec();

                messages.push(RawMessage {
                    log_time,
                    data: payload,
                });
            }
        }

        pos += content_len;
    }

    messages
}
