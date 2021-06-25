#ifndef CX_UTILS_LOCK_SHARED_PTR_HPP
#define CX_UTILS_LOCK_SHARED_PTR_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace cx {

template <class T> class LockSharedPtr {
public:
  LockSharedPtr();
  ~LockSharedPtr();
  LockSharedPtr(std::shared_ptr<T> &&ptr);
  LockSharedPtr(const LockSharedPtr<T> &other);
  LockSharedPtr(LockSharedPtr<T> &&other);
  LockSharedPtr<T> &operator=(LockSharedPtr<T> &&other);
  LockSharedPtr<T> &operator=(const LockSharedPtr<T> &other);
  // LockSharedPtr<T> &operator=(T ptr);
  std::shared_ptr<T> operator->() const;
  operator bool() const;
  // Called from instances to lock the mutex for the current scope
  void scopedLock();

private:
  std::shared_ptr<T> obj;
  mutable std::shared_mutex objmutex;
  static int numbers;
};

template <class T> LockSharedPtr<T>::LockSharedPtr() {
  // obj = std::make_shared<T>();
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "Constructor NO param");

  LockSharedPtr::numbers++;
}

template <class T> LockSharedPtr<T>::~LockSharedPtr() {
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "Destroying %i",
              LockSharedPtr::numbers);
  LockSharedPtr::numbers--;
}

template <class T>
LockSharedPtr<T>::LockSharedPtr(std::shared_ptr<T> &&ptr)
    : obj(std::move(ptr)) {
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "Constructor with param");
}

// COPY CONSTRUCTOR
template <class T>
LockSharedPtr<T>::LockSharedPtr(const LockSharedPtr<T> &other) {
  LockSharedPtr::numbers++;

  std::scoped_lock<std::shared_mutex> guard(other.objmutex);
  this->obj = other.obj;
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "copied C");
}

// Move Constructor
template <class T> LockSharedPtr<T>::LockSharedPtr(LockSharedPtr<T> &&other) {
  LockSharedPtr::numbers++;

  std::scoped_lock<std::shared_mutex> guard(other.objmutex);
  obj = std::move(other.obj);
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "moved C");
}

// Move assigment
template <class T>
LockSharedPtr<T> &LockSharedPtr<T>::operator=(LockSharedPtr<T> &&other) {
  LockSharedPtr::numbers++;
  if (this != &other) {
    std::scoped_lock<std::shared_mutex, std::shared_mutex> guard(
        objmutex, other.objmutex);
    obj = std::move(other.obj);
    RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "moved Assig");
  }
  return *this;
}

// Copy Asignment
template <class T>
LockSharedPtr<T> &LockSharedPtr<T>::operator=(const LockSharedPtr<T> &other) {
  LockSharedPtr::numbers++;
  if (this != &other) {
    std::scoped_lock<std::shared_mutex, std::shared_mutex> guard(
        objmutex, other.objmutex);
    obj = other.obj;
    RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "copied Assig");
  }
  return *this;
}
// template <class T> LockSharedPtr<T> &LockSharedPtr<T>::operator=(T ptr) {

// }
/** Dereferencing.
 * Usage:refptr->memberfun()
 */
template <class T> std::shared_ptr<T> LockSharedPtr<T>::operator->() const {
  return obj;
}
template <class T> void LockSharedPtr<T>::scopedLock() {
  std::scoped_lock<std::shared_mutex> guard(objmutex);
}

template <class T> LockSharedPtr<T>::operator bool() const {
  return (obj != nullptr);
}

template <class T> int LockSharedPtr<T>::numbers = 0;

} // namespace cx

#endif // !CX_UTILS_LOCK_SHARED_PTR_HPP
