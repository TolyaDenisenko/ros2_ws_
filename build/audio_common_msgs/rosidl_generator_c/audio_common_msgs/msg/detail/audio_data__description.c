// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_common_msgs:msg/AudioData.idl
// generated code does not contain a copyright notice

#include "audio_common_msgs/msg/detail/audio_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_common_msgs
const rosidl_type_hash_t *
audio_common_msgs__msg__AudioData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x74, 0x2f, 0xba, 0x15, 0x67, 0xe6, 0x49,
      0xbb, 0xdb, 0xa6, 0xae, 0x03, 0x4a, 0x72, 0xc7,
      0xef, 0x12, 0x9b, 0x97, 0xc7, 0xa4, 0xda, 0x50,
      0xfd, 0x2f, 0x3e, 0x1b, 0xfd, 0x8f, 0xfa, 0x86,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char audio_common_msgs__msg__AudioData__TYPE_NAME[] = "audio_common_msgs/msg/AudioData";

// Define type names, field names, and default values
static char audio_common_msgs__msg__AudioData__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field audio_common_msgs__msg__AudioData__FIELDS[] = {
  {
    {audio_common_msgs__msg__AudioData__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_common_msgs__msg__AudioData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_common_msgs__msg__AudioData__TYPE_NAME, 31, 31},
      {audio_common_msgs__msg__AudioData__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_common_msgs__msg__AudioData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_common_msgs__msg__AudioData__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 13, 13},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_common_msgs__msg__AudioData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_common_msgs__msg__AudioData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
