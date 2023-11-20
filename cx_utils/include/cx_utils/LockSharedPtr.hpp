/***************************************************************************
 *  LockSharedPtr.hpp
 *
 *  Created: 21 June 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef CX_UTILS_LOCK_SHARED_PTR_HPP
#define CX_UTILS_LOCK_SHARED_PTR_HPP

#include <memory>
#include <mutex>
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
  void init_mutex();
  operator bool() const;
  std::mutex *get_mutex_instance();
  std::shared_ptr<T> get_obj();
  // Called from instances to lock the mutex for the current scope

private:
  std::shared_ptr<T> obj;
  mutable std::shared_ptr<std::mutex> objmutex;
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
  this->obj = other.obj;
  this->objmutex = other.objmutex;
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "copied C");
}

// Move Constructor
template <class T> LockSharedPtr<T>::LockSharedPtr(LockSharedPtr<T> &&other) {
  LockSharedPtr::numbers++;
  obj = std::move(other.obj);
  objmutex = std::move(other.objmutex);
  RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "moved C");
}

// Move assigment
template <class T>
LockSharedPtr<T> &LockSharedPtr<T>::operator=(LockSharedPtr<T> &&other) {
  LockSharedPtr::numbers++;
  if (this != &other) {
    obj = std::move(other.obj);
    objmutex = std::move(other.objmutex);
    RCLCPP_INFO(rclcpp::get_logger("SharedPtrC"), "moved Assig");
  }
  return *this;
}

// Copy Asignment
template <class T>
LockSharedPtr<T> &LockSharedPtr<T>::operator=(const LockSharedPtr<T> &other) {
  LockSharedPtr::numbers++;
  if (this != &other) {
    obj = other.obj;
    objmutex = other.objmutex;
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

template <class T> LockSharedPtr<T>::operator bool() const {
  return (obj != nullptr);
}

template <class T>
std::mutex* LockSharedPtr<T>::get_mutex_instance() {
  auto mutex = objmutex.get();
  return mutex;
}

template <class T> std::shared_ptr<T> LockSharedPtr<T>::get_obj() {
  return obj;
}

template <class T> void LockSharedPtr<T>::init_mutex() {
  objmutex = std::make_shared<std::mutex>();
}

template <class T> int LockSharedPtr<T>::numbers = 0;

} // namespace cx

#endif // !CX_UTILS_LOCK_SHARED_PTR_HPP
