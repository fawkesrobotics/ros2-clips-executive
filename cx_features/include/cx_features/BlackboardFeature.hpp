#ifndef CX_FEATURES__BLACKBOARDFEATURE_HPP_
#define CX_FEATURES__BLACKBOARDFEATURE_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

#include "cx_blackboard/InterfaceManager.hpp"
#include "cx_blackboard/InterfaceReader.hpp"
#include "cx_blackboard/InterfaceWriter.hpp"

#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosidl_runtime_c/message_initialization.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace cx {

class BlackboardFeature : public ClipsFeature {
public:
  BlackboardFeature();
  ~BlackboardFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool
  clips_context_destroyed(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;

  std::string getFeatureName() const;

private:
  // const std::string clips_feature_name;
  using ReadersMap = std::map<
      std::string,
      std::vector<std::pair<
          std::string, cx::InterfaceManager::Interface_Reader_Types_Variant>>>;

  using WritersMap = std::map<
      std::string,
      std::vector<std::pair<
          std::string, cx::InterfaceManager::Interface_Writer_Types_Variant>>>;

  typedef struct {
    ReadersMap reading;
    WritersMap writing;
  } Interfaces;

  bool cfg_retract_early_{true};
  std::map<std::string, Interfaces> interfaces_;
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;

private:
  bool create_bb_reader(const std::string &env_name, const std::string &i_type,
                        const std::string &i_id, const std::string &reader_id,
                        bool new_interface);

  cx::InterfaceManager::Interface_Reader_Types_Optional
  cast_bb_reader(const std::string &env_name, const std::string &i_type,
                 const std::string &i_id);

  bool clips_assert_interface_type(const std::string &env_name,
                                   const std::string &log_name,
                                   const std::string &i_uid);

  void clips_blackboard_open_interface(const std::string &env_name,
                                       const std::string &i_type,
                                       const std::string &i_id, bool writing);

  // void clips_blackboard_open_interface_writing(const std::string &env_name,
  //                                              const std::string &i_type,
  //                                              const std::string &i_id);

  void clips_blackboard_open_interface_reading(const std::string &env_name,
                                               const std::string &i_type,
                                               const std::string &i_id);

  void clips_blackboard_close_interface(const std::string &env_name,
                                        const std::string &i_type,
                                        const std::string &i_id);

  void clips_blackboard_read(const std::string &env_name);

  CLIPS::Value clips_blackboard_create_msg(const std::string &env_name,
                                           const std::string &i_uid,
                                           const std::string &msg_type);

  void clips_blackboard_enable_time_read(const std::string &env_name);
};
} // namespace cx

#endif // !CX_FEATURES__BLACKBOARDFEATURE_HPP_