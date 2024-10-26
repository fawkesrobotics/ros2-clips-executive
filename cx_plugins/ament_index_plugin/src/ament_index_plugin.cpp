// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_ament_index_plugin/ament_index_plugin.hpp"
#include <cx_utils/clips_env_context.hpp>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_packages_with_prefixes.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_search_paths.hpp"
#include "ament_index_cpp/has_resource.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
AmentIndexPlugin::AmentIndexPlugin() {}

AmentIndexPlugin::~AmentIndexPlugin() {}

void AmentIndexPlugin::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
}

bool AmentIndexPlugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_DEBUG(*logger_, "Initializing plugin for environment %s",
               context->env_name_.c_str());
  std::string fun_name;
  fun_name = "ament-index-get-package-prefix";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "s", 1, 1, ";sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        clips::UDFValue package_name;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &package_name);
        std::string prefix = ament_index_cpp::get_package_prefix(
            package_name.lexemeValue->contents);
        out->lexemeValue = clips::CreateString(env, prefix.c_str());
      },
      "ament_index_get_package_prefix", NULL);

  fun_name = "ament-index-get-package-share-directory";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "s", 1, 1, ";sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        clips::UDFValue package_name;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &package_name);
        std::string share_dir = ament_index_cpp::get_package_share_directory(
            package_name.lexemeValue->contents);
        out->lexemeValue = clips::CreateString(env, share_dir.c_str());
      },
      "ament_index_get_package_share_directory", NULL);

  fun_name = "ament-index-get-packages-with-prefixes";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "m", 0, 0, NULL,
      [](clips::Environment *env, clips::UDFContext * /*udfc*/,
         clips::UDFValue *out) {
        auto packages_with_prefixes =
            ament_index_cpp::get_packages_with_prefixes();
        clips::MultifieldBuilder *mb = clips::CreateMultifieldBuilder(
            env, packages_with_prefixes.size() * 2);
        for (const auto &[package, prefix] : packages_with_prefixes) {
          clips::MBAppendString(mb, package.c_str());
          clips::MBAppendString(mb, prefix.c_str());
        }
        out->multifieldValue = clips::MBCreate(mb);
        clips::MBDispose(mb);
      },
      "ament_index_get_packages_with_prefixes", NULL);

  fun_name = "ament-index-get-resource";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "bm", 2, 2, ";sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        clips::UDFValue resource_type, resource_name;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &resource_type);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &resource_name);
        std::string content;
        std::string path;
        if (ament_index_cpp::get_resource(resource_type.lexemeValue->contents,
                                          resource_name.lexemeValue->contents,
                                          content, &path)) {
          out->multifieldValue = clips::StringToMultifield(
              env, std::format("\"{}\" \"{}\"", content, path).c_str());
        } else {
          out->lexemeValue = clips::CreateBoolean(env, false);
        }
      },
      "ament_index_get_resource", NULL);

  fun_name = "ament-index-get-resources";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "m", 1, 1, "sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        clips::UDFValue resource_type;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &resource_type);
        auto resources =
            ament_index_cpp::get_resources(resource_type.lexemeValue->contents);
        clips::MultifieldBuilder *mb =
            clips::CreateMultifieldBuilder(env, resources.size() * 2);
        for (const auto &[package, prefix] : resources) {
          clips::MBAppendString(mb, package.c_str());
          clips::MBAppendString(mb, prefix.c_str());
        }
        out->multifieldValue = clips::MBCreate(mb);
        clips::MBDispose(mb);
      },
      "ament_index_get_resources", NULL);

  fun_name = "ament-index-get-search-paths";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "m", 0, 0, NULL,
      [](clips::Environment *env, clips::UDFContext * /*udfc*/,
         clips::UDFValue *out) {
        auto paths = ament_index_cpp::get_search_paths();
        clips::MultifieldBuilder *mb =
            clips::CreateMultifieldBuilder(env, paths.size());
        for (const auto &path : paths) {
          clips::MBAppendString(mb, path.c_str());
        }
        out->multifieldValue = clips::MBCreate(mb);
        clips::MBDispose(mb);
      },
      "ament_index_get_search_paths", NULL);

  fun_name = "ament-index-has-resource";
  function_names_.emplace(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "b", 2, 2, ";sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        clips::UDFValue resource_type, resource_name;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &resource_type);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &resource_name);
        bool exists =
            ament_index_cpp::has_resource(resource_type.lexemeValue->contents,
                                          resource_name.lexemeValue->contents);
        out->lexemeValue = clips::CreateBoolean(env, exists);
      },
      "ament_index_has_resource", NULL);

  return true;
}

bool AmentIndexPlugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Destroying plugin for environment %s",
              context->env_name_.c_str());
  for (const auto &fun : function_names_) {
    clips::RemoveUDF(env.get_obj().get(), fun.c_str());
  }
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::AmentIndexPlugin, cx::ClipsPlugin)
