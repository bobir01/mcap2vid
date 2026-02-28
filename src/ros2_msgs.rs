use anyhow::{anyhow, Result};
use byteorder::{LittleEndian, ReadBytesExt};
use std::io::Cursor;

/// ROS2 Time structure
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

impl Time {
    /// Convert to Unix timestamp as f64 (seconds with nanosecond precision)
    pub fn to_unix_timestamp(&self) -> f64 {
        self.sec as f64 + (self.nanosec as f64 / 1_000_000_000.0)
    }
}

/// ROS2 std_msgs/Header
#[derive(Debug, Clone, serde::Serialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

/// ROS2 sensor_msgs/Image
#[derive(Debug, Clone)]
pub struct Image {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: Vec<u8>,
}

/// ROS2 sensor_msgs/CompressedImage
#[derive(Debug, Clone)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    pub data: Vec<u8>,
}

/// Supported image message types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImageMessageType {
    Image,
    CompressedImage,
}

impl ImageMessageType {
    pub fn from_schema_name(name: &str) -> Option<Self> {
        if name.contains("CompressedImage") {
            Some(ImageMessageType::CompressedImage)
        } else if name.contains("Image") {
            Some(ImageMessageType::Image)
        } else {
            None
        }
    }
}

/// Supported metadata message types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MetadataMessageType {
    CameraInfo,
    TFMessage,
}

impl MetadataMessageType {
    pub fn from_schema_name(name: &str) -> Option<Self> {
        if name.contains("CameraInfo") {
            Some(MetadataMessageType::CameraInfo)
        } else if name.contains("TFMessage") {
            Some(MetadataMessageType::TFMessage)
        } else {
            None
        }
    }
}

/// ROS2 sensor_msgs/RegionOfInterest
#[derive(Debug, Clone, serde::Serialize)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}

/// ROS2 sensor_msgs/CameraInfo
#[derive(Debug, Clone, serde::Serialize)]
pub struct CameraInfo {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: String,
    pub d: Vec<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: RegionOfInterest,
}

/// ROS2 geometry_msgs/Vector3
#[derive(Debug, Clone, serde::Serialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// ROS2 geometry_msgs/Quaternion
#[derive(Debug, Clone, serde::Serialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// ROS2 geometry_msgs/Transform
#[derive(Debug, Clone, serde::Serialize)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

/// ROS2 geometry_msgs/TransformStamped
#[derive(Debug, Clone, serde::Serialize)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

/// ROS2 tf2_msgs/TFMessage
#[derive(Debug, Clone, serde::Serialize)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

/// Align cursor position to specified boundary
fn align_to(cursor: &mut Cursor<&[u8]>, alignment: usize) {
    let pos = cursor.position() as usize;
    let remainder = pos % alignment;
    if remainder != 0 {
        cursor.set_position((pos + alignment - remainder) as u64);
    }
}

/// Parse ROS2 CDR-encoded string
fn read_cdr_string(cursor: &mut Cursor<&[u8]>) -> Result<String> {
    // Align to 4 bytes before reading string length
    align_to(cursor, 4);

    let len = cursor.read_u32::<LittleEndian>()? as usize;
    if len == 0 {
        return Ok(String::new());
    }
    let mut buf = vec![0u8; len];
    std::io::Read::read_exact(cursor, &mut buf)?;
    // Remove null terminator if present
    if buf.last() == Some(&0) {
        buf.pop();
    }
    Ok(String::from_utf8_lossy(&buf).to_string())
}

/// Parse ROS2 CDR-encoded byte array
fn read_cdr_byte_array(cursor: &mut Cursor<&[u8]>) -> Result<Vec<u8>> {
    // Align to 4 bytes before reading array length
    align_to(cursor, 4);

    let len = cursor.read_u32::<LittleEndian>()? as usize;
    let mut buf = vec![0u8; len];
    std::io::Read::read_exact(cursor, &mut buf)?;
    Ok(buf)
}

/// Parse ROS2 CDR-encoded Header
fn read_cdr_header(cursor: &mut Cursor<&[u8]>) -> Result<Header> {
    let sec = cursor.read_i32::<LittleEndian>()?;
    let nanosec = cursor.read_u32::<LittleEndian>()?;
    let frame_id = read_cdr_string(cursor)?;
    Ok(Header {
        stamp: Time { sec, nanosec },
        frame_id,
    })
}

/// Parse ROS2 CDR-encoded sensor_msgs/Image
pub fn parse_image(data: &[u8]) -> Result<Image> {
    // Skip CDR encapsulation header (4 bytes: 0x00 0x01 for little-endian, + 2 bytes options)
    if data.len() < 4 {
        return Err(anyhow!("Data too short for CDR encapsulation"));
    }
    let mut cursor = Cursor::new(&data[4..]);

    let header = read_cdr_header(&mut cursor)?;

    // height and width are u32, need 4-byte alignment
    align_to(&mut cursor, 4);
    let height = cursor.read_u32::<LittleEndian>()?;
    let width = cursor.read_u32::<LittleEndian>()?;

    let encoding = read_cdr_string(&mut cursor)?;

    let is_bigendian = cursor.read_u8()?;

    // step is u32, need 4-byte alignment
    align_to(&mut cursor, 4);
    let step = cursor.read_u32::<LittleEndian>()?;

    let img_data = read_cdr_byte_array(&mut cursor)?;

    Ok(Image {
        header,
        height,
        width,
        encoding,
        is_bigendian,
        step,
        data: img_data,
    })
}

/// Parse ROS2 CDR-encoded sensor_msgs/CompressedImage
pub fn parse_compressed_image(data: &[u8]) -> Result<CompressedImage> {
    // Skip CDR encapsulation header (4 bytes)
    if data.len() < 4 {
        return Err(anyhow!("Data too short for CDR encapsulation"));
    }
    let mut cursor = Cursor::new(&data[4..]);

    let header = read_cdr_header(&mut cursor)?;
    let format = read_cdr_string(&mut cursor)?;
    let img_data = read_cdr_byte_array(&mut cursor)?;

    Ok(CompressedImage {
        header,
        format,
        data: img_data,
    })
}

/// Read a fixed-size f64 array from CDR data
fn read_cdr_f64_array<const N: usize>(cursor: &mut Cursor<&[u8]>) -> Result<[f64; N]> {
    align_to(cursor, 8); // f64 requires 8-byte alignment
    let mut arr = [0.0f64; N];
    for i in 0..N {
        arr[i] = cursor.read_f64::<LittleEndian>()?;
    }
    Ok(arr)
}

/// Read a variable-length f64 sequence from CDR data
fn read_cdr_f64_sequence(cursor: &mut Cursor<&[u8]>) -> Result<Vec<f64>> {
    align_to(cursor, 4); // sequence length is u32
    let len = cursor.read_u32::<LittleEndian>()? as usize;
    if len == 0 {
        return Ok(Vec::new());
    }
    align_to(cursor, 8); // f64 requires 8-byte alignment
    let mut vec = Vec::with_capacity(len);
    for _ in 0..len {
        vec.push(cursor.read_f64::<LittleEndian>()?);
    }
    Ok(vec)
}

/// Parse ROS2 CDR-encoded geometry_msgs/Vector3
fn read_cdr_vector3(cursor: &mut Cursor<&[u8]>) -> Result<Vector3> {
    align_to(cursor, 8);
    Ok(Vector3 {
        x: cursor.read_f64::<LittleEndian>()?,
        y: cursor.read_f64::<LittleEndian>()?,
        z: cursor.read_f64::<LittleEndian>()?,
    })
}

/// Parse ROS2 CDR-encoded geometry_msgs/Quaternion
fn read_cdr_quaternion(cursor: &mut Cursor<&[u8]>) -> Result<Quaternion> {
    align_to(cursor, 8);
    Ok(Quaternion {
        x: cursor.read_f64::<LittleEndian>()?,
        y: cursor.read_f64::<LittleEndian>()?,
        z: cursor.read_f64::<LittleEndian>()?,
        w: cursor.read_f64::<LittleEndian>()?,
    })
}

/// Parse ROS2 CDR-encoded geometry_msgs/TransformStamped
fn read_cdr_transform_stamped(cursor: &mut Cursor<&[u8]>) -> Result<TransformStamped> {
    let header = read_cdr_header(cursor)?;
    let child_frame_id = read_cdr_string(cursor)?;
    let translation = read_cdr_vector3(cursor)?;
    let rotation = read_cdr_quaternion(cursor)?;

    Ok(TransformStamped {
        header,
        child_frame_id,
        transform: Transform {
            translation,
            rotation,
        },
    })
}

/// Parse ROS2 CDR-encoded sensor_msgs/CameraInfo
pub fn parse_camera_info(data: &[u8]) -> Result<CameraInfo> {
    if data.len() < 4 {
        return Err(anyhow!("Data too short for CDR encapsulation"));
    }
    let mut cursor = Cursor::new(&data[4..]);

    let header = read_cdr_header(&mut cursor)?;

    align_to(&mut cursor, 4);
    let height = cursor.read_u32::<LittleEndian>()?;
    let width = cursor.read_u32::<LittleEndian>()?;

    let distortion_model = read_cdr_string(&mut cursor)?;

    // D is a variable-length sequence
    let d = read_cdr_f64_sequence(&mut cursor)?;

    // K, R, P are fixed-size arrays
    let k = read_cdr_f64_array::<9>(&mut cursor)?;
    let r = read_cdr_f64_array::<9>(&mut cursor)?;
    let p = read_cdr_f64_array::<12>(&mut cursor)?;

    align_to(&mut cursor, 4);
    let binning_x = cursor.read_u32::<LittleEndian>()?;
    let binning_y = cursor.read_u32::<LittleEndian>()?;

    // RegionOfInterest
    let x_offset = cursor.read_u32::<LittleEndian>()?;
    let y_offset = cursor.read_u32::<LittleEndian>()?;
    let roi_height = cursor.read_u32::<LittleEndian>()?;
    let roi_width = cursor.read_u32::<LittleEndian>()?;
    let do_rectify = cursor.read_u8()? != 0;

    Ok(CameraInfo {
        header,
        height,
        width,
        distortion_model,
        d,
        k,
        r,
        p,
        binning_x,
        binning_y,
        roi: RegionOfInterest {
            x_offset,
            y_offset,
            height: roi_height,
            width: roi_width,
            do_rectify,
        },
    })
}

/// Parse ROS2 CDR-encoded tf2_msgs/TFMessage
pub fn parse_tf_message(data: &[u8]) -> Result<TFMessage> {
    if data.len() < 4 {
        return Err(anyhow!("Data too short for CDR encapsulation"));
    }
    let mut cursor = Cursor::new(&data[4..]);

    // TFMessage contains a sequence of TransformStamped
    align_to(&mut cursor, 4);
    let count = cursor.read_u32::<LittleEndian>()? as usize;

    let mut transforms = Vec::with_capacity(count);
    for _ in 0..count {
        transforms.push(read_cdr_transform_stamped(&mut cursor)?);
    }

    Ok(TFMessage { transforms })
}
