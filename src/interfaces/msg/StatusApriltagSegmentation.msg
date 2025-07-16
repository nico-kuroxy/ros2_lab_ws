# StatusApriltagSegmentation.msg is a message made to broadcast the current status of the april tag segmentation nodes.
# Core field, shared by every status message.
interfaces/StatusCoreNode core_status # Core bloc of the message, there should always be one.
# Specific field, only for this message.
bool is_apriltag_detected # Whether or not an Apriltag is currently detected in the camera frame.

