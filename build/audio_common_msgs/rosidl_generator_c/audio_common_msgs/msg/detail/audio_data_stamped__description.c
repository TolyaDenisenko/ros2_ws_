// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from audio_common_msgs:msg/AudioDataStamped.idl
// generated code does not contain a copyright notice

#include "audio_common_msgs/msg/detail/audio_data_stamped__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_audio_common_msgs
const rosidl_type_hash_t *
audio_common_msgs__msg__AudioDataStamped__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0xc6, 0x79, 0xd5, 0xa8, 0x68, 0xed, 0x68,
      0xe4, 0xd3, 0x01, 0xf1, 0xef, 0x42, 0xab, 0xec,
      0xc8, 0x61, 0x40, 0xaf, 0xb5, 0x7d, 0x25, 0x56,
      0x56, 0xff, 0x2d, 0x9e, 0x47, 0x6e, 0xa9, 0xe3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "audio_common_msgs/msg/detail/audio_data__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t audio_common_msgs__msg__AudioData__EXPECTED_HASH = {1, {
    0xa9, 0x74, 0x2f, 0xba, 0x15, 0x67, 0xe6, 0x49,
    0xbb, 0xdb, 0xa6, 0xae, 0x03, 0x4a, 0x72, 0xc7,
    0xef, 0x12, 0x9b, 0x97, 0xc7, 0xa4, 0xda, 0x50,
    0xfd, 0x2f, 0x3e, 0x1b, 0xfd, 0x8f, 0xfa, 0x86,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char audio_common_msgs__msg__AudioDataStamped__TYPE_NAME[] = "audio_common_msgs/msg/AudioDataStamped";
static char audio_common_msgs__msg__AudioData__TYPE_NAME[] = "audio_common_msgs/msg/AudioData";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char audio_common_msgs__msg__AudioDataStamped__FIELD_NAME__header[] = "header";
static char audio_common_msgs__msg__AudioDataStamped__FIELD_NAME__audio[] = "audio";

static rosidl_runtime_c__type_description__Field audio_common_msgs__msg__AudioDataStamped__FIELDS[] = {
  {
    {audio_common_msgs__msg__AudioDataStamped__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {audio_common_msgs__msg__AudioDataStamped__FIELD_NAME__audio, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {audio_common_msgs__msg__AudioData__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription audio_common_msgs__msg__AudioDataStamped__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {audio_common_msgs__msg__AudioData__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
audio_common_msgs__msg__AudioDataStamped__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {audio_common_msgs__msg__AudioDataStamped__TYPE_NAME, 38, 38},
      {audio_common_msgs__msg__AudioDataStamped__FIELDS, 2, 2},
    },
    {audio_common_msgs__msg__AudioDataStamped__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&audio_common_msgs__msg__AudioData__EXPECTED_HASH, audio_common_msgs__msg__AudioData__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = audio_common_msgs__msg__AudioData__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "audio_common_msgs/AudioData audio";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
audio_common_msgs__msg__AudioDataStamped__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {audio_common_msgs__msg__AudioDataStamped__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 57, 57},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_common_msgs__msg__AudioDataStamped__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *audio_common_msgs__msg__AudioDataStamped__get_individual_type_description_source(NULL),
    sources[1] = *audio_common_msgs__msg__AudioData__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
