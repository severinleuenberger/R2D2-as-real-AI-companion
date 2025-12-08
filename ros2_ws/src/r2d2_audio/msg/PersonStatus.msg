# Person Recognition Status Message
# Published by audio_notification_node to communicate recognition state
# Consumed by: LED controller, STT-LLM-TTS, database logger

# Recognition status state machine
string status              # "red" = recognized | "blue" = lost | "green" = unknown

# Person identity
string person_identity     # "severin" | "unknown" | "no_person"

# Timing information
builtin_interfaces/Time timestamp   # When status changed (ROS 2 time)

# Detection confidence
float32 confidence         # 0.0-1.0 (face detection confidence score)

# Duration tracking
float32 duration_in_state  # How long in current state (seconds)
