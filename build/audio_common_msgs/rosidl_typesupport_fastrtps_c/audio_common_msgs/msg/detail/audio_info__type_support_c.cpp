// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from audio_common_msgs:msg/AudioInfo.idl
// generated code does not contain a copyright notice
#include "audio_common_msgs/msg/detail/audio_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "audio_common_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "audio_common_msgs/msg/detail/audio_info__struct.h"
#include "audio_common_msgs/msg/detail/audio_info__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // coding_format, sample_format
#include "rosidl_runtime_c/string_functions.h"  // coding_format, sample_format

// forward declare type support functions


using _AudioInfo__ros_msg_type = audio_common_msgs__msg__AudioInfo;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
bool cdr_serialize_audio_common_msgs__msg__AudioInfo(
  const audio_common_msgs__msg__AudioInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: channels
  {
    cdr << ros_message->channels;
  }

  // Field name: sample_rate
  {
    cdr << ros_message->sample_rate;
  }

  // Field name: sample_format
  {
    const rosidl_runtime_c__String * str = &ros_message->sample_format;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: bitrate
  {
    cdr << ros_message->bitrate;
  }

  // Field name: coding_format
  {
    const rosidl_runtime_c__String * str = &ros_message->coding_format;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
bool cdr_deserialize_audio_common_msgs__msg__AudioInfo(
  eprosima::fastcdr::Cdr & cdr,
  audio_common_msgs__msg__AudioInfo * ros_message)
{
  // Field name: channels
  {
    cdr >> ros_message->channels;
  }

  // Field name: sample_rate
  {
    cdr >> ros_message->sample_rate;
  }

  // Field name: sample_format
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->sample_format.data) {
      rosidl_runtime_c__String__init(&ros_message->sample_format);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->sample_format,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'sample_format'\n");
      return false;
    }
  }

  // Field name: bitrate
  {
    cdr >> ros_message->bitrate;
  }

  // Field name: coding_format
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->coding_format.data) {
      rosidl_runtime_c__String__init(&ros_message->coding_format);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->coding_format,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'coding_format'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
size_t get_serialized_size_audio_common_msgs__msg__AudioInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AudioInfo__ros_msg_type * ros_message = static_cast<const _AudioInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: channels
  {
    size_t item_size = sizeof(ros_message->channels);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sample_rate
  {
    size_t item_size = sizeof(ros_message->sample_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sample_format
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->sample_format.size + 1);

  // Field name: bitrate
  {
    size_t item_size = sizeof(ros_message->bitrate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: coding_format
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->coding_format.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
size_t max_serialized_size_audio_common_msgs__msg__AudioInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: channels
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: sample_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: sample_format
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: bitrate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: coding_format
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = audio_common_msgs__msg__AudioInfo;
    is_plain =
      (
      offsetof(DataType, coding_format) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
bool cdr_serialize_key_audio_common_msgs__msg__AudioInfo(
  const audio_common_msgs__msg__AudioInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: channels
  {
    cdr << ros_message->channels;
  }

  // Field name: sample_rate
  {
    cdr << ros_message->sample_rate;
  }

  // Field name: sample_format
  {
    const rosidl_runtime_c__String * str = &ros_message->sample_format;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: bitrate
  {
    cdr << ros_message->bitrate;
  }

  // Field name: coding_format
  {
    const rosidl_runtime_c__String * str = &ros_message->coding_format;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
size_t get_serialized_size_key_audio_common_msgs__msg__AudioInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AudioInfo__ros_msg_type * ros_message = static_cast<const _AudioInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: channels
  {
    size_t item_size = sizeof(ros_message->channels);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sample_rate
  {
    size_t item_size = sizeof(ros_message->sample_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sample_format
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->sample_format.size + 1);

  // Field name: bitrate
  {
    size_t item_size = sizeof(ros_message->bitrate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: coding_format
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->coding_format.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_audio_common_msgs
size_t max_serialized_size_key_audio_common_msgs__msg__AudioInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: channels
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: sample_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: sample_format
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: bitrate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: coding_format
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = audio_common_msgs__msg__AudioInfo;
    is_plain =
      (
      offsetof(DataType, coding_format) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _AudioInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const audio_common_msgs__msg__AudioInfo * ros_message = static_cast<const audio_common_msgs__msg__AudioInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_audio_common_msgs__msg__AudioInfo(ros_message, cdr);
}

static bool _AudioInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  audio_common_msgs__msg__AudioInfo * ros_message = static_cast<audio_common_msgs__msg__AudioInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_audio_common_msgs__msg__AudioInfo(cdr, ros_message);
}

static uint32_t _AudioInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_audio_common_msgs__msg__AudioInfo(
      untyped_ros_message, 0));
}

static size_t _AudioInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_audio_common_msgs__msg__AudioInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AudioInfo = {
  "audio_common_msgs::msg",
  "AudioInfo",
  _AudioInfo__cdr_serialize,
  _AudioInfo__cdr_deserialize,
  _AudioInfo__get_serialized_size,
  _AudioInfo__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _AudioInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AudioInfo,
  get_message_typesupport_handle_function,
  &audio_common_msgs__msg__AudioInfo__get_type_hash,
  &audio_common_msgs__msg__AudioInfo__get_type_description,
  &audio_common_msgs__msg__AudioInfo__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, audio_common_msgs, msg, AudioInfo)() {
  return &_AudioInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
