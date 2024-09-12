// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  {{name_camel}}.cpp
 *
 *  Automatically Generated: {{ gen_date }}
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
// clang-format off
#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "{{name_snake}}.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace cx {

{{name_camel}}::{{name_camel}}()
    : Node("{{name_snake}}_msg_feature_node") {}
{{name_camel}}::~{{name_camel}}() {}

std::string {{name_camel}}::getFeatureName() const {
  return clips_feature_name;
}

void {{name_camel}}::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;

  spin_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

bool {{name_camel}}::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(get_logger(),
              "Destroying clips context!");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-create-client");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-destroy-client");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-set-field-request");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-set-field-response");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-get-field-request");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-get-field-response");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-create-request");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-create-service");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-destroy-service");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-send-request");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-destroy-request");
  clips::Deftemplate *curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-client");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-service");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-request");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-response");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  envs_.erase(env_name);

  return true;
}

bool {{name_camel}}::clips_context_init(const std::string &env_name,
    LockSharedPtr<clips::Environment> &clips) {
  RCLCPP_INFO(get_logger(),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;
  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-set-field-request", "v", 3, 3, ";e;sy;*",
      [](clips::Environment */*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue request, field, value;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &request);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        clips::UDFNthArgument(udfc, 3, ANY_TYPE_BITS, &value);

        instance->set_field_request(static_cast<{{message_type}}::Request*>(request.externalAddressValue->contents), field.lexemeValue->contents, value, udfc);
      },
      "set_field_request", this);
  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-set-field-response", "v", 3, 3, ";e;sy;*",
      [](clips::Environment */*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue response, field, value;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &response);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        clips::UDFNthArgument(udfc, 3, ANY_TYPE_BITS, &value);

        instance->set_field_response(static_cast<{{message_type}}::Response*>(response.externalAddressValue->contents), field.lexemeValue->contents, value, udfc);
      },
      "set_field_response", this);
  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-get-field-request", "*", 2, 2, ";e;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue request, field;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &request);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        *out = instance->get_field_request(env, static_cast<{{message_type}}::Request*>(request.externalAddressValue->contents), field.lexemeValue->contents);
      },
      "get_field_request", this);
  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-get-field-response", "*", 2, 2, ";e;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue response, field;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &response);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        *out = instance->get_field_response(env, static_cast<{{message_type}}::Response*>(response.externalAddressValue->contents), field.lexemeValue->contents);
      },
      "get_field_response", this);

  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-create-request", "e", 0, 0, "",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        using namespace clips;
        *out = instance->create_request(env);
      },
      "create_request", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-create-service", "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue service_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);

      instance->create_new_service(env, service_name.lexemeValue->contents);
    },
    "create_new_service", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-destroy-service", "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue service_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);

      instance->destroy_service(env, service_name.lexemeValue->contents);
    },
    "destroy_service", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-destroy-request", "v", 1, 1, ";e",
    [](clips::Environment */*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue request;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &request);

      instance->destroy_request(static_cast<{{message_type}}::Request*>(request.externalAddressValue->contents));
    },
    "destroy_request", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-create-client", "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue service_name;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);
        instance->create_new_client(env,service_name.lexemeValue->contents);
    },
    "create_new_client", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-destroy-client", "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue service_name;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);
        instance->destroy_client(env, service_name.lexemeValue->contents);
    },
    "destroy_client", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-send-request", "v", 2, 2, ";e;sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue request, service_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &request);
      clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &service_name);

      instance->send_request(env, static_cast<{{message_type}}::Request*>(request.externalAddressValue->contents), service_name.lexemeValue->contents);
    },
    "send_request", this);


  // add fact templates
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-client \
            (slot service (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-service \
            (slot name (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-response \
            (slot service (type STRING) ) \
            (slot msg-ptr (type EXTERNAL-ADDRESS)) \
            )");


  return true;
}

clips::UDFValue {{name_camel}}::create_request(clips::Environment *env) {
  {{message_type}}::Request *new_req = new {{message_type}}::Request();
  requests_.insert(new_req);
  clips::UDFValue res;
  res.externalAddressValue = clips::CreateCExternalAddress(env, new_req);
  return res;
}

   clips::UDFValue {{name_camel}}::get_field_request(clips::Environment *env,
                               {{message_type}}::Request *req,
                               const std::string &field) {
    clips::UDFValue res;
    size_t full_length = 0;
    clips::MultifieldBuilder *mb;
{%- for slot in request_slots %}
{%- if not slot.array %}
  if (field == "{{ slot.name }}") {
{%- if slot.clips_type == "STRING" %}
    res.lexemeValue = clips::CreateString(env, std::string(req->{{ slot.name }}).c_str());
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
    res.integerValue = clips::CreateInteger(env, req->{{ slot.name }});
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
    res.floatValue = clips::CreateFloat(env, req->{{ slot.name }};
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
    res.lexemeValue = clips::CreateSymbol(env, req->{{ slot.name }} ? "TRUE" : "FALSE");
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    res.externalAddressValue = clips::CreateCExternalAddress(env, &(req->{{ slot.name }}));
{%- endif %}
  }
{%- else %}
  if (field == "{{ slot.name }}") {
  full_length = req->{{ slot.name }}.size();
  mb = clips::CreateMultifieldBuilder(env, full_length);
  for (size_t i = 0; i < full_length; i++) {
{%- if slot.clips_type == "STRING" %}
      clips::MBAppendString(mb,std::string(req->{{ slot.name }}[i]).c_str());
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
      clips::MBAppendInteger(mb,req->{{ slot.name }}[i]);
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
      clips::MBAppendFloat(mb,req->{{ slot.name }}[i]);
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
      clips::MBAppendSymbol(mb,req->{{ slot.name }}[i] ? "TRUE" : "FALSE");
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
      clips::MBAppendCLIPSExternalAddress(mb, clips::CreateCExternalAddress(env, &(req->{{ slot.name }}[i])));
{%- endif %}
    }
  res.multifieldValue = clips::MBCreate(mb);
  clips::MBDispose(mb);
  }
{%- endif -%}
{%- endfor %}
  (void) full_length;
  (void) mb;
  return res;
}

clips::UDFValue {{name_camel}}::get_field_response(clips::Environment *env,
                               {{message_type}}::Response *resp,
                               const std::string &field) {
    clips::UDFValue res;
    size_t full_length = 0;
    clips::MultifieldBuilder *mb;
{%- for slot in response_slots %}
{%- if not slot.array %}
  if (field == "{{ slot.name }}") {
{%- if slot.clips_type == "STRING" %}
    res.lexemeValue = clips::CreateString(env, std::string(resp->{{ slot.name }}).c_str());
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
    res.integerValue = clips::CreateInteger(env, resp->{{ slot.name }});
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
    res.floatValue = clips::CreateFloat(env, resp->{{ slot.name }};
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
    res.lexemeValue = clips::CreateSymbol(env, resp->{{ slot.name }} ? "TRUE" : "FALSE");
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    res.externalAddressValue = clips::CreateCExternalAddress(env, &(resp->{{ slot.name }}));
{%- endif %}
  }
{%- else %}
  if (field == "{{ slot.name }}") {
  full_length = resp->{{ slot.name }}.size();
  mb = clips::CreateMultifieldBuilder(env, full_length);
  for (size_t i = 0; i < full_length; i++) {
{%- if slot.clips_type == "STRING" %}
      clips::MBAppendString(mb,std::string(resp->{{ slot.name }}[i]).c_str());
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
      clips::MBAppendInteger(mb,resp->{{ slot.name }}[i]);
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
      clips::MBAppendFloat(mb,resp->{{ slot.name }}[i]);
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
      clips::MBAppendSymbol(mb,resp->{{ slot.name }}[i] ? "TRUE" : "FALSE");
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
      clips::MBAppendCLIPSExternalAddress(mb, clips::CreateCExternalAddress(env, &(resp->{{ slot.name }}[i])));
{%- endif %}
    }
  res.multifieldValue = clips::MBCreate(mb);
  clips::MBDispose(mb);
  }
{%- endif -%}
{%- endfor %}
  (void) full_length;
  (void) mb;
  return res;
}

void {{name_camel}}::set_field_request({{message_type}}::Request *req,
                                       const std::string &field,
                                       clips::UDFValue value, clips::UDFContext *udfc) {
  clips::Multifield *multi;

{%- for slot in request_slots %}
{%- if not slot.array %}
  if (field == "{{ slot.name }}") {
{%- if slot.clips_type == "STRING" %}
    req->{{ slot.name }} = value.lexemeValue->contents;
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
    req->{{ slot.name }} = value.integerValue->contents;
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
    req->{{ slot.name }} = value.floatValue->contents;
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
    req->{{ slot.name }} = std::string(value.lexemeValue->contents).compare(std::string("TRUE")) ? false : true;
    RCLCPP_INFO(get_logger(), "I SET %s (%b)", value.lexemeValue->contents, req->{{slot.name}});
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    req->{{ slot.name }} = *static_cast<{{slot.type}} *>(value.externalAddressValue->contents);
{%- endif %}
  }
{%- else %}
  if (field == "{{ slot.name }}") {
    multi = value.multifieldValue;
  {%- if slot.clips_type == "STRING" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case STRING_TYPE:
      case SYMBOL_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].lexemeValue->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected STRING/SYMBOL) of %li nth argument of UDF {{name_kebab}}-request-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }

    req->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "INTEGER" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case INTEGER_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].integerType->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected INTEGER) of %li nth argument of UDF {{name_kebab}}-request-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    req->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "FLOAT" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case FLOAT_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].floatType->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected FLOAT) of %li nth argument of UDF {{name_kebab}}-request-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    req->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    std::vector<{{slot.type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case EXTERNAL_ADDRESS_TYPE:
        value_vector_{{slot.name}}.push_back(*static_cast<{{slot.type}}*>((multi->contents[i].externalAddressValue->contents)));
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected FLOAT) of %li nth argument of UDF {{name_kebab}}-request-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    req->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  }
{%- endif -%}
{%- endfor %}
  (void)multi;
  (void)udfc;
}

void {{name_camel}}::set_field_response({{message_type}}::Response *resp,
                                        const std::string &field,
                                        clips::UDFValue value, clips::UDFContext *udfc) {
  clips::Multifield *multi;

{%- for slot in response_slots %}
{%- if not slot.array %}
  if (field == "{{ slot.name }}") {
{%- if slot.clips_type == "STRING" %}
    resp->{{ slot.name }} = value.lexemeValue->contents;
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
    resp->{{ slot.name }} = value.integerValue->contents;
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
    resp->{{ slot.name }} = value.floatValue->contents;
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
    resp->{{ slot.name }} = std::string(value.lexemeValue->contents).compare(std::string("TRUE")) ? false : true;
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    resp->{{ slot.name }} = *static_cast<{{slot.type}} *>(value.externalAddressValue->contents);
{%- endif %}
  }
{%- else %}
  if (field == "{{ slot.name }}") {
    multi = value.multifieldValue;
  {%- if slot.clips_type == "STRING" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case STRING_TYPE:
      case SYMBOL_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].lexemeValue->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected STRING/SYMBOL) of %li nth argument of UDF {{name_kebab}}-response-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }

    resp->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "INTEGER" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case INTEGER_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].integerType->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected INTEGER) of %li nth argument of UDF {{name_kebab}}-response-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    resp->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "FLOAT" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case FLOAT_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].floatType->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected FLOAT) of %li nth argument of UDF {{name_kebab}}-response-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    resp->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    std::vector<{{slot.type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case EXTERNAL_ADDRESS_TYPE:
        value_vector_{{slot.name}}.push_back(*static_cast<{{slot.type}}*>((multi->contents[i].externalAddressValue->contents)));
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected FLOAT) of %li nth argument of UDF {{name_kebab}}-response-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    resp->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  }
{%- endif -%}
{%- endfor %}
  (void)multi;
  (void)udfc;
}

void {{name_camel}}::send_request(clips::Environment *env, {{message_type}}::Request *req, const std::string &service_name) {
  using namespace std::chrono_literals;
  {{message_type}}::Request::SharedPtr req_shared = std::make_shared<{{message_type}}::Request>(*req);
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    return;
  }

   // Handle the result asynchronously to not block clips engine potentially endlessly
  std::thread([this, req_shared, service_name, env_name]() {
  while (!clients_[env_name][service_name]->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "service %s not available, waiting again...", service_name.c_str());
  }
  cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
  auto future = clients_[env_name][service_name]->async_send_request(req_shared);
  auto resp = future.get();
  {{message_type}}::Response *heap_resp = new {{message_type}}::Response();
  *heap_resp = *resp;
   responses_.insert(heap_resp);
    clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-response");
    clips::FBPutSlotString(fact_builder,"service",service_name.c_str());
    clips::FBPutSlotCLIPSExternalAddress(fact_builder,"msg-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), heap_resp));
    clips::FBAssert(fact_builder);
    clips::FBDispose(fact_builder);

  }).detach();
}

void {{name_camel}}::create_new_service(clips::Environment *env, const std::string &service_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    return;
  }
  services_[env_name][service_name] =
      this->create_service<{{message_type}}>(service_name, [this, env_name, service_name](const std::shared_ptr<{{message_type}}::Request> request,
    std::shared_ptr<{{message_type}}::Response> response) {
    this->service_callback(request, response, service_name, env_name);
  });
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-service (name \"" +
                                 service_name + "\"))").c_str());
}

void {{name_camel}}::destroy_service(clips::Environment *env, const std::string &service_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    return;
  }

  auto outer_it = services_.find(env_name);
  if (outer_it != services_.end()) {
      // Check if service_name exists in the inner map
      auto& inner_map = outer_it->second;
      auto inner_it = inner_map.find(service_name);
      if (inner_it != inner_map.end()) {
          // Remove the service_name entry from the inner map
          inner_map.erase(inner_it);
      } else {
          RCLCPP_WARN(this->get_logger(), "Service %s not found in environment %s", service_name.c_str(), env_name.c_str());
      }
  } else {
      RCLCPP_WARN(this->get_logger(), "Environment %s not found", env_name.c_str());
  }
}

void {{name_camel}}::create_new_client(clips::Environment *env,
    const std::string &service_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Creating client for serive %s",
              service_name.c_str());
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(rclcpp::get_logger(clips_feature_name),
                 "Unable to determine environment from raw pointer");
    return;
  }

  auto it = clients_[env_name].find(service_name);

  if (it != clients_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "There already exists a client for service %s", service_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Creating client for service %s", service_name.c_str());
    clients_[env_name][service_name] =
        this->create_client<{{message_type}}>(service_name);
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-client (service \"" + service_name + "\"))").c_str());
  }
}

void {{name_camel}}::destroy_client(clips::Environment *env,
    const std::string &service_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(rclcpp::get_logger(clips_feature_name),
                 "Unable to determine environment from raw pointer");
    return;
  }

  auto it = clients_[env_name].find(service_name);

  if (it != clients_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Destroying client for service %s", service_name.c_str());
    clients_[env_name].erase(service_name);
  }

  clips::Eval(env, ("(do-for-all-facts ((?f {{name_kebab}}-client)) (eq (str-cat ?f:service) (str-cat " + service_name + "))  (retract ?f))").c_str(), NULL);
}

void {{name_camel}}::service_callback(const std::shared_ptr<{{message_type}}::Request> request,
                        std::shared_ptr<{{message_type}}::Response> response,
    std::string service_name,
    std::string env_name) {
  cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  // call a user-defined function
  clips::FunctionCallBuilder *fcb = clips::CreateFunctionCallBuilder(clips.get_obj().get(),3);
  clips::FCBAppendString(fcb, service_name.c_str());
  clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), request.get()));
  clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), response.get()));
  clips::FCBCall(fcb,"{{name_kebab}}-service-callback",NULL);
  clips::FCBDispose(fcb);
}

void {{name_camel}}::destroy_request({{message_type}}::Request *req) {
  auto it = requests_.find(req);
  if (it != requests_.end()) {
      // If found, delete the element
      requests_.erase(it);
      delete req;
  }
}


} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::{{name_camel}}, cx::ClipsFeature)
