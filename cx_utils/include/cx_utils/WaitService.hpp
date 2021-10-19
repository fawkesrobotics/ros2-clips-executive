#ifndef CX_UTILS__WAITSERVICE_HPP_
#define CX_UTILS__WAITSERVICE_HPP_

#include <chrono>
#include <memory>
#include <thread>

namespace cx {

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_future_result(FutureT &future,
                                          WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status ret_status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto remaining_time = end - now;
    if (remaining_time <= std::chrono::seconds(0)) {
      break;
    }
    ret_status = future.wait_for((remaining_time < wait_period) ? remaining_time
                                                                : wait_period);
  } while (rclcpp::ok() && ret_status != std::future_status::ready);
  return ret_status;
}

} // namespace cx
#endif // !CX_UTILS__WAITSERVICE_HPP_