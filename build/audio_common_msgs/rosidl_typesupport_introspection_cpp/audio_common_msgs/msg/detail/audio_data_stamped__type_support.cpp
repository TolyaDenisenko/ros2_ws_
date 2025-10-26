// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from audio_common_msgs:msg/AudioDataStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "audio_common_msgs/msg/detail/audio_data_stamped__functions.h"
#include "audio_common_msgs/msg/detail/audio_data_stamped__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace audio_common_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void AudioDataStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) audio_common_msgs::msg::AudioDataStamped(_init);
}

void AudioDataStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<audio_common_msgs::msg::AudioDataStamped *>(message_memory);
  typed_message->~AudioDataStamped();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AudioDataStamped_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_common_msgs::msg::AudioDataStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "audio",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<audio_common_msgs::msg::AudioData>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_common_msgs::msg::AudioDataStamped, audio),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AudioDataStamped_message_members = {
  "audio_common_msgs::msg",  // message namespace
  "AudioDataStamped",  // message name
  2,  // number of fields
  sizeof(audio_common_msgs::msg::AudioDataStamped),
  false,  // has_any_key_member_
  AudioDataStamped_message_member_array,  // message members
  AudioDataStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  AudioDataStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AudioDataStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AudioDataStamped_message_members,
  get_message_typesupport_handle_function,
  &audio_common_msgs__msg__AudioDataStamped__get_type_hash,
  &audio_common_msgs__msg__AudioDataStamped__get_type_description,
  &audio_common_msgs__msg__AudioDataStamped__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace audio_common_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<audio_common_msgs::msg::AudioDataStamped>()
{
  return &::audio_common_msgs::msg::rosidl_typesupport_introspection_cpp::AudioDataStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, audio_common_msgs, msg, AudioDataStamped)() {
  return &::audio_common_msgs::msg::rosidl_typesupport_introspection_cpp::AudioDataStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
