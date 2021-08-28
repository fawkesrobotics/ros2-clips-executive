#ifndef CX_BLACKBOARD__INTERFACEMANAGER_HPP_
#define CX_BLACKBOARD__INTERFACEMANAGER_HPP_

#include <map>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"

#include "cx_utils/NodeThread.hpp"

#include "cx_blackboard/Interface.hpp"
#include "cx_blackboard/InterfaceNode.hpp"
#include "cx_blackboard/InterfaceReader.hpp"
#include "cx_blackboard/InterfaceWriter.hpp"

// Include corresponding messages and services!
#include "cx_msgs/msg/pddl_gen_interface.hpp"
#include "cx_msgs/msg/pddl_gen_interface_messages.hpp"
#include "cx_msgs/srv/pddl_gen_interface_srv.hpp"

namespace cx {

class InterfaceManager {

public:
  using PddlGenInterface =
      std::shared_ptr<cx::InterfaceNode<cx_msgs::srv::PddlGenInterfaceSrv,
                                        cx_msgs::msg::PddlGenInterface>>;

  using PddlGenInterface_Reader = std::shared_ptr<cx::InterfaceReader<
      cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
      cx_msgs::msg::PddlGenInterfaceMessages>>;

  using PddlGenInterface_Writer = std::shared_ptr<cx::InterfaceWriter<
      cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
      cx_msgs::msg::PddlGenInterfaceMessages>>;

  typedef std::variant<PddlGenInterface> Interface_Node_Types_Variant;
  typedef std::optional<PddlGenInterface> Interface_Node_Types_Optional;

  typedef std::variant<PddlGenInterface_Reader> Interface_Reader_Types_Variant;
  typedef std::optional<PddlGenInterface_Reader>
      Interface_Reader_Types_Optional;

  typedef std::variant<PddlGenInterface_Writer> Interface_Writer_Types_Variant;
  typedef std::optional<PddlGenInterface_Writer>
      Interface_Writer_Types_Optional;

  typedef enum {
    INTERFACE_PddlGenInterface = 0,
    INTERFACE_PddlPlannerInterface
  } RegisteredInterfaces;

  InterfaceManager();
  // ~InterfaceManager();

  Interface_Node_Types_Optional cast_interface(const std::string &i_type);

  void generate_interface_node(const std::string &i_name,
                               const std::string &i_type,
                               const std::string &i_id);

  void set_interface_writer(const std::string &i_type,
                            const std::string &i_writer);
  void set_interface_reader(const std::string &i_type,
                            const std::string &i_reader);

  std::map<std::string, Interface_Node_Types_Variant> interfaces_;
  std::list<std::unique_ptr<cx::NodeThread>> threads_list_;

private:
  // Maps the string representation given as input to an interface index, which
  // will be used withen the casts
  std::unordered_map<std::string, RegisteredInterfaces> const interface_map_ = {
      {"PddlGenInterface", RegisteredInterfaces::INTERFACE_PddlGenInterface},
      {"PddlPlannerInterface",
       RegisteredInterfaces::INTERFACE_PddlPlannerInterface}};

private:
};

} // namespace cx

#endif // !CX_BLACKBOARD__INTERFACEMANAGER_HPP_