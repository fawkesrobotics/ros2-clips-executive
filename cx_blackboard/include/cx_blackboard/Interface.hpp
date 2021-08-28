#ifndef CX_BLACKBOARD__INTERFACE_HPP_
#define CX_BLACKBOARD__INTERFACE_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <string>

namespace cx {

template <class InterfaceMessageType> class Interface {
public:
  Interface();
  Interface(const std::string &type, const std::string &id);
  // constructor from given message
  // Interface(const Interface<InterfaceMessageType> &other);
  // ~Interface();
  void set_writer(const std::string &writer);
  void set_reader(const std::string &reader);

  bool is_writer(const std::string &writer) const;
  bool has_writer() const;
  std::string list_writer();
  std::list<std::string> list_readers();

  void set_owner(const std::string &owner);

  // void set_instance_serial(const Uuid &serial);
  // TODO: MOVE TO PRIVATE
  const std::string type_;
  const std::string id_;
  const std::string uid_;
  std::string owner_;

  std::shared_ptr<InterfaceMessageType> interface_data_ptr_;

private:
  std::string writer_;
  std::list<std::string> readers_;
};

template <class InterfaceMessageType>
Interface<InterfaceMessageType>::Interface() {}

template <class InterfaceMessageType>
Interface<InterfaceMessageType>::Interface(const std::string &type,
                                           const std::string &id)
    : type_{type}, id_{id}, uid_{type + "::" + id},
      interface_data_ptr_{std::make_shared<InterfaceMessageType>()} {
  RCLCPP_INFO(rclcpp::get_logger(type), "Initialising...");
  RCLCPP_INFO(rclcpp::get_logger(type), "Initialised!");
}

template <class InterfaceMessageType>
void Interface<InterfaceMessageType>::set_writer(const std::string &writer) {
  writer_ = writer;
}

template <class InterfaceMessageType>
void Interface<InterfaceMessageType>::set_reader(const std::string &reader) {
  readers_.push_back(reader);
}

template <class InterfaceMessageType>
bool Interface<InterfaceMessageType>::is_writer(
    const std::string &writer) const {
  return writer == writer_;
}

template <class InterfaceMessageType>
bool Interface<InterfaceMessageType>::has_writer() const {
  return writer_ != "";
}

template <class InterfaceMessageType>
std::string Interface<InterfaceMessageType>::list_writer() {
  return writer_;
}

template <class InterfaceMessageType>
std::list<std::string> Interface<InterfaceMessageType>::list_readers() {
  return readers_;
}

} // namespace cx

#endif // !CX_BLACKBOARD__INTERFACE_HPP_