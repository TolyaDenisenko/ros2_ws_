// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_common_msgs:msg/AudioInfo.idl
// generated code does not contain a copyright notice

#include "audio_common_msgs/msg/detail/audio_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_common_msgs
const rosidl_type_hash_t *
audio_common_msgs__msg__AudioInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb3, 0x49, 0x4d, 0x20, 0x4e, 0x51, 0x3d, 0x2c,
      0xce, 0xef, 0xae, 0x18, 0x88, 0xe5, 0xbe, 0x3d,
      0xa7, 0xc8, 0xc8, 0xca, 0x31, 0xeb, 0x27, 0x7f,
      0x7b, 0x5b, 0x62, 0xf3, 0x64, 0xbc, 0xb1, 0xfb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char audio_common_msgs__msg__AudioInfo__TYPE_NAME[] = "audio_common_msgs/msg/AudioInfo";

// Define type names, field names, and default values
static char audio_common_msgs__msg__AudioInfo__FIELD_NAME__channels[] = "channels";
static char audio_common_msgs__msg__AudioInfo__FIELD_NAME__sample_rate[] = "sample_rate";
static char audio_common_msgs__msg__AudioInfo__FIELD_NAME__sample_format[] = "sample_format";
static char audio_common_msgs__msg__AudioInfo__FIELD_NAME__bitrate[] = "bitrate";
static char audio_common_msgs__msg__AudioInfo__FIELD_NAME__coding_format[] = "coding_format";

static rosidl_runtime_c__type_description__Field audio_common_msgs__msg__AudioInfo__FIELDS[] = {
  {
    {audio_common_msgs__msg__AudioInfo__FIELD_NAME__channels, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_common_msgs__msg__AudioInfo__FIELD_NAME__sample_rate, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_common_msgs__msg__AudioInfo__FIELD_NAME__sample_format, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_common_msgs__msg__AudioInfo__FIELD_NAME__bitrate, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {audio_common_msgs__msg__AudioInfo__FIELD_NAME__coding_format, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_common_msgs__msg__AudioInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_common_msgs__msg__AudioInfo__TYPE_NAME, 31, 31},
      {audio_common_msgs__msg__AudioInfo__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains the audio meta data\n"
  "\n"
  "# Number of channels\n"
  "uint8 channels\n"
  "# Sampling rate [Hz]\n"
  "uint32 sample_rate\n"
  "# Audio format (e.g. S16LE)\n"
  "string sample_format\n"
  "# Amount of audio data per second [bits/s]\n"
  "uint32 bitrate\n"
  "# Audio coding format (e.g. WAVE, MP3)\n"
  "string coding_format";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_common_msgs__msg__AudioInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_common_msgs__msg__AudioInfo__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 288, 288},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_common_msgs__msg__AudioInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_common_msgs__msg__AudioInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
