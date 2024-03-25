#pragma once

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <optional>

template <typename T>
class LcmListener {
  using UserCallbackType = std::function<void(const T&)>;

 public:
  LcmListener(std::shared_ptr<lcm::LCM> const& lcm, const std::string& channel, UserCallbackType const& cb)
      : lcm_(lcm), channel_(channel), callback_(cb) {
    lcm_->subscribe(channel, &LcmListener::callback, this);
  }

  LcmListener(std::shared_ptr<lcm::LCM> const& lcm, const std::string& channel) : lcm_(lcm), channel_(channel) {
    lcm_->subscribe(channel, &LcmListener::callback, this);
  }

  void callback(const lcm::ReceiveBuffer* /*buf*/, const std::string& /*channel*/, const T* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_message_ = *message;
    has_latest_message_ = true;
    if (callback_) {
      callback_.value()(latest_message_);
    }
  }

  /**
   * @brief Get the latest message
   * @return A copy of the latest message, so the caller does not need to work about race conditions
   */
  T getLatestMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_message_;
  }

  bool hasLatestMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_latest_message_;
  }

 private:
  std::shared_ptr<lcm::LCM> lcm_;
  std::string channel_;
  T latest_message_{};
  std::optional<UserCallbackType> callback_;
  bool has_latest_message_{false};
  mutable std::mutex mutex_;
};