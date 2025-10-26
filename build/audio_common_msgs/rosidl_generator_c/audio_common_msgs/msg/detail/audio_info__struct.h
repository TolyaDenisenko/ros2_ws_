// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_common_msgs:msg/AudioInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_common_msgs/msg/audio_info.h"


#ifndef AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_INFO__STRUCT_H_
#define AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'sample_format'
// Member 'coding_format'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AudioInfo in the package audio_common_msgs.
/**
  * This message contains the audio meta data
 */
typedef struct audio_common_msgs__msg__AudioInfo
{
  /// Number of channels
  uint8_t channels;
  /// Sampling rate
  uint32_t sample_rate;
  /// Audio format (e.g. S16LE)
  rosidl_runtime_c__String sample_format;
  /// Amount of audio data per second
  uint32_t bitrate;
  /// Audio coding format (e.g. WAVE, MP3)
  rosidl_runtime_c__String coding_format;
} audio_common_msgs__msg__AudioInfo;

// Struct for a sequence of audio_common_msgs__msg__AudioInfo.
typedef struct audio_common_msgs__msg__AudioInfo__Sequence
{
  audio_common_msgs__msg__AudioInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_common_msgs__msg__AudioInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_INFO__STRUCT_H_
