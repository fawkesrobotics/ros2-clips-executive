// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cx_cdb_plugin/cdb_plugin.hpp"
#include <clips_ns/engine.h>
#include <clips_ns/factfun.h>
#include <clips_ns/factmngr.h>
#include <clips_ns/utility.h>
#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {
CDBPlugin::CDBPlugin() {}

CDBPlugin::~CDBPlugin() {}

void CDBPlugin::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
}

bool CDBPlugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Initializing plugin for environment %s",
              context->env_name_.c_str());
  clips::AddAssertFunction(env.get_obj().get(), "cdb_assert_callback", &cdb_assert_callback, 0, this);
  clips::AddBeforeRuleFiresFunction(env.get_obj().get(), "cdb_before_rule_callback",  &cdb_before_rule_callback, 0, this);
  return true;
}

void CDBPlugin::cdb_before_rule_callback(clips::Environment *, clips::Activation *act, void *context){
  CDBPlugin *cdb_plugin = static_cast<CDBPlugin*>(context);
  RCLCPP_INFO(*cdb_plugin->logger_, "Rule %s is about to fire", act->theRule->header.ppForm);
}
void CDBPlugin::cdb_assert_callback(clips::Environment *env, void* fact, void* context) {
  CDBPlugin *cdb_plugin = static_cast<CDBPlugin*>(context);
  clips::Fact *f = static_cast<clips::Fact*>(fact);
  clips::StringBuilder *sb = clips::CreateStringBuilder(env, 1024);
  clips::FactPPForm(f, sb, false);
  RCLCPP_INFO(*cdb_plugin->logger_, "Fact %lld asserted with content: %s", f->factIndex, sb->contents);
}

bool CDBPlugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Destroying plugin for environment %s",
              context->env_name_.c_str());
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBPlugin, cx::ClipsPlugin)
