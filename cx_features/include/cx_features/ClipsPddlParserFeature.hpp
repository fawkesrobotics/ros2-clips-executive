#ifndef CX_FEATURES__CLIPSPDDLPARSERFEATURE_HPP_
#define CX_FEATURES__CLIPSPDDLPARSERFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include <clips_pddl_parser/clips_pddl_parser.h>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class ClipsPddlParserFeature : public ClipsFeature {
public:
  ClipsPddlParserFeature();
  ~ClipsPddlParserFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool
  clips_context_destroyed(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
  std::mutex mm;
  CLIPS::Environment* maintained_env_;
  std::unique_ptr<clips_pddl_parser::ClipsPddlParser> pddl_parser;

private:
};

} // namespace cx
#endif // !CX_FEATURES__CLIPSPDDLPARSERFEATURE_HPP_