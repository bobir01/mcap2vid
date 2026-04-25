/// Encoding details from an MCAP schema/channel pair.
#[derive(Debug, Clone, Default)]
pub struct TopicSchema {
    pub schema_name: String,
    pub schema_encoding: String,
    pub message_encoding: String,
}

impl TopicSchema {
    pub fn new(
        schema_name: impl Into<String>,
        schema_encoding: impl Into<String>,
        message_encoding: impl Into<String>,
    ) -> Self {
        Self {
            schema_name: schema_name.into(),
            schema_encoding: schema_encoding.into(),
            message_encoding: message_encoding.into(),
        }
    }
}

/// Supported video-like MCAP message types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoMessageKind {
    Ros2Image,
    Ros2CompressedImage,
    FoxgloveCompressedImageProto,
    FoxgloveCompressedVideoProto,
}

impl VideoMessageKind {
    pub fn from_topic_schema(schema: &TopicSchema) -> Option<Self> {
        classify_video_message(
            &schema.schema_name,
            &schema.schema_encoding,
            &schema.message_encoding,
        )
    }

    pub fn display_name(self) -> &'static str {
        match self {
            Self::Ros2Image => "ROS2 Image",
            Self::Ros2CompressedImage => "ROS2 CompressedImage",
            Self::FoxgloveCompressedImageProto => "Foxglove CompressedImage",
            Self::FoxgloveCompressedVideoProto => "Foxglove CompressedVideo",
        }
    }

    pub fn is_exportable_image(self) -> bool {
        !matches!(self, Self::FoxgloveCompressedVideoProto)
    }
}

/// Supported metadata MCAP message types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MetadataMessageKind {
    CameraInfo,
    TFMessage,
}

impl MetadataMessageKind {
    pub fn from_topic_schema(schema: &TopicSchema) -> Option<Self> {
        let schema_name = schema.schema_name.as_str();
        if !is_ros2_cdr(&schema.schema_encoding, &schema.message_encoding) {
            return None;
        }

        match schema_name {
            "sensor_msgs/msg/CameraInfo" | "sensor_msgs/CameraInfo" => Some(Self::CameraInfo),
            "tf2_msgs/msg/TFMessage" | "tf2_msgs/TFMessage" => Some(Self::TFMessage),
            _ => None,
        }
    }
}

fn classify_video_message(
    schema_name: &str,
    schema_encoding: &str,
    message_encoding: &str,
) -> Option<VideoMessageKind> {
    if is_ros2_cdr(schema_encoding, message_encoding) {
        return match schema_name {
            "sensor_msgs/msg/Image" | "sensor_msgs/Image" => Some(VideoMessageKind::Ros2Image),
            "sensor_msgs/msg/CompressedImage" | "sensor_msgs/CompressedImage" => {
                Some(VideoMessageKind::Ros2CompressedImage)
            }
            _ => None,
        };
    }

    if is_protobuf(schema_encoding, message_encoding) {
        return match schema_name {
            "foxglove.CompressedImage" => Some(VideoMessageKind::FoxgloveCompressedImageProto),
            "foxglove.CompressedVideo" => Some(VideoMessageKind::FoxgloveCompressedVideoProto),
            _ => None,
        };
    }

    None
}

fn is_ros2_cdr(schema_encoding: &str, message_encoding: &str) -> bool {
    schema_encoding == "ros2msg" && message_encoding == "cdr"
}

fn is_protobuf(schema_encoding: &str, message_encoding: &str) -> bool {
    schema_encoding == "protobuf" && message_encoding == "protobuf"
}

#[cfg(test)]
mod tests {
    use super::{TopicSchema, VideoMessageKind};

    #[test]
    fn classifies_ros2_image_exactly() {
        let schema = TopicSchema::new("sensor_msgs/msg/Image", "ros2msg", "cdr");
        assert_eq!(
            VideoMessageKind::from_topic_schema(&schema),
            Some(VideoMessageKind::Ros2Image)
        );
    }

    #[test]
    fn classifies_foxglove_video_exactly() {
        let schema = TopicSchema::new("foxglove.CompressedVideo", "protobuf", "protobuf");
        assert_eq!(
            VideoMessageKind::from_topic_schema(&schema),
            Some(VideoMessageKind::FoxgloveCompressedVideoProto)
        );
    }

    #[test]
    fn does_not_treat_foxglove_compressed_video_as_image() {
        let schema = TopicSchema::new("foxglove.CompressedVideo", "ros2msg", "cdr");
        assert_eq!(VideoMessageKind::from_topic_schema(&schema), None);
    }
}
