// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from audio_common_msgs:msg/AudioDataStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_common_msgs/msg/audio_data_stamped.hpp"


#ifndef AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_DATA_STAMPED__BUILDER_HPP_
#define AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_DATA_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "audio_common_msgs/msg/detail/audio_data_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace audio_common_msgs
{

namespace msg
{

namespace builder
{

class Init_AudioDataStamped_audio
{
public:
  explicit Init_AudioDataStamped_audio(::audio_common_msgs::msg::AudioDataStamped & msg)
  : msg_(msg)
  {}
  ::audio_common_msgs::msg::AudioDataStamped audio(::audio_common_msgs::msg::AudioDataStamped::_audio_type arg)
  {
    msg_.audio = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_common_msgs::msg::AudioDataStamped msg_;
};

class Init_AudioDataStamped_header
{
public:
  Init_AudioDataStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AudioDataStamped_audio header(::audio_common_msgs::msg::AudioDataStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AudioDataStamped_audio(msg_);
  }

private:
  ::audio_common_msgs::msg::AudioDataStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_common_msgs::msg::AudioDataStamped>()
{
  return audio_common_msgs::msg::builder::Init_AudioDataStamped_header();
}

}  // namespace audio_common_msgs

#endif  // AUDIO_COMMON_MSGS__MSG__DETAIL__AUDIO_DATA_STAMPED__BUILDER_HPP_
