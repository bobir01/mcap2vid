use anyhow::Result;
use byteorder::{BigEndian, WriteBytesExt};
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::Path;

/// FTSS atom magic bytes (Frame TimeStamp Sync)
const FTSS_ATOM_TYPE: &[u8; 4] = b"FTSS";
const FTSS_VERSION: u8 = 1;

/// Embed frame timestamps into an MP4 file as a top-level atom
///
/// This appends a custom FTSS atom at the end of the MP4 file.
/// This approach is safe because it doesn't modify the existing moov structure
/// or require updating chunk offsets.
///
/// Format:
/// - 4 bytes: atom size (big-endian)
/// - 4 bytes: 'FTSS' (Frame TimeStamp Sync)
/// - 1 byte: version (1)
/// - 4 bytes: frame count (big-endian)
/// - N * 8 bytes: timestamps as f64 (big-endian)
pub fn embed_timestamps<P: AsRef<Path>>(mp4_path: P, timestamps: &[f64]) -> Result<()> {
    let mp4_path = mp4_path.as_ref();

    // Build the FTSS atom data
    let ftss_data = build_ftss_atom(timestamps)?;

    // Append to the end of the file (simplest and safest approach)
    let mut file = OpenOptions::new().append(true).open(mp4_path)?;

    file.write_all(&ftss_data)?;

    Ok(())
}

/// Build the FTSS atom binary data
fn build_ftss_atom(timestamps: &[f64]) -> Result<Vec<u8>> {
    let frame_count = timestamps.len() as u32;
    let data_size = 1 + 4 + (timestamps.len() * 8); // version + count + timestamps
    let atom_size = 8 + data_size; // header + data

    let mut data = Vec::with_capacity(atom_size);

    // Atom header
    data.write_u32::<BigEndian>(atom_size as u32)?;
    data.write_all(FTSS_ATOM_TYPE)?;

    // Atom data
    data.write_u8(FTSS_VERSION)?;
    data.write_u32::<BigEndian>(frame_count)?;

    for &ts in timestamps {
        data.write_f64::<BigEndian>(ts)?;
    }

    Ok(data)
}

/// Find an atom by type, returning (start_position, size)
fn find_atom(data: &[u8], atom_type: &[u8; 4], start: usize) -> Option<(usize, usize)> {
    let mut pos = start;

    while pos + 8 <= data.len() {
        let size =
            u32::from_be_bytes([data[pos], data[pos + 1], data[pos + 2], data[pos + 3]]) as usize;
        let atype = &data[pos + 4..pos + 8];

        if size == 0 {
            break; // Invalid atom
        }

        if atype == atom_type {
            return Some((pos, size));
        }

        pos += size;
    }

    None
}

/// Read timestamps from an MP4 file's FTSS atom
#[allow(dead_code)]
pub fn read_timestamps<P: AsRef<Path>>(mp4_path: P) -> Result<Vec<f64>> {
    let mut file = File::open(mp4_path.as_ref())?;
    let mut data = Vec::new();
    file.read_to_end(&mut data)?;

    // Find FTSS atom (scan from beginning)
    if let Some((ftss_start, ftss_size)) = find_atom(&data, FTSS_ATOM_TYPE, 0) {
        let content_start = ftss_start + 8; // Skip atom header

        if ftss_size < 13 {
            // Minimum: header(8) + version(1) + count(4)
            anyhow::bail!("FTSS atom too small");
        }

        let version = data[content_start];
        if version != FTSS_VERSION {
            anyhow::bail!("Unsupported FTSS version: {}", version);
        }

        let frame_count = u32::from_be_bytes([
            data[content_start + 1],
            data[content_start + 2],
            data[content_start + 3],
            data[content_start + 4],
        ]) as usize;

        let mut timestamps = Vec::with_capacity(frame_count);
        let ts_start = content_start + 5;

        for i in 0..frame_count {
            let offset = ts_start + i * 8;
            if offset + 8 > data.len() {
                break;
            }
            let ts = f64::from_be_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            timestamps.push(ts);
        }

        return Ok(timestamps);
    }

    anyhow::bail!("No FTSS atom found in MP4 file")
}
