#pragma once

#include <lcm/lcm-cpp.hpp>
#include <mutex>

template <typename T>
class LcmListener {
 public:
  LcmListener(std::shared_ptr<lcm::LCM> const& lcm, const std::string& channel) : lcm_(lcm), channel_(channel) {
    lcm_->subscribe(channel, &LcmListener::callback, this);
  }

  void callback(const lcm::ReceiveBuffer* /*buf*/, const std::string& /*channel*/, const T* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_message_ = *message;
  }

  /**
   * @brief Get the latest message
   * @return A copy of the latest message, so the caller does not need to work about race conditions
   */
  T getLatestMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_message_;
  }

 private:
  std::shared_ptr<lcm::LCM> lcm_;
  std::string channel_;
  T latest_message_{};
  mutable std::mutex mutex_;
};