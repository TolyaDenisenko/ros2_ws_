// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_common_msgs:msg/AudioDataStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_common_msgs/msg/audio_data_stamped.h"


#ifndef AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_DATA_STAMPED__STRUCT_H_
#define AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_DATA_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'audio'
#include "audio_common_msgs/msg/detail/audio_data__struct.h"

/// Struct defined in msg/AudioDataStamped in the package audio_common_msgs.
typedef struct audio_common_msgs__msg__AudioDataStamped
{
  std_msgs__msg__Header header;
  audio_common_msgs__msg__AudioData audio;
} audio_common_msgs__msg__AudioDataStamped;

// Struct for a sequence of audio_common_msgs__msg__AudioDataStamped.
typedef struct audio_common_msgs__msg__AudioDataStamped__Sequence
{
  audio_common_msgs__msg__AudioDataStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_common_msgs__msg__AudioDataStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_DATA_STAMPED__STRUCT_H_
