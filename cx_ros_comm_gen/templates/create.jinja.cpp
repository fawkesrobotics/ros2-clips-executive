// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

// clang-format off
{%- if template_part == "definition" %}
clips::UDFValue {{name_camel}}::{{template_type|snake_case}}_create(clips::Environment *env) {
  std::shared_ptr<{{message_type}}::{{template_type|camel_case}}> ptr = std::make_shared<{{message_type}}::{{template_type|camel_case}}>();
  {{template_type|snake_case}}s_[ptr.get()] = ptr;
  clips::UDFValue res;
  res.externalAddressValue = clips::CreateCExternalAddress(env, ptr.get());
  return res;
}
{%- endif -%}

{%- if template_part == "registration" %}
  function_names_.insert("{{name_kebab}}-{{template_type|kebab_case}}-create");
  clips::AddUDF(
      env.get_obj().get(), "{{name_kebab}}-{{template_type|kebab_case}}-create", "e", 0, 0, "",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        using namespace clips;
        *out = instance->{{template_type|snake_case}}_create(env);
      },
      "{{template_type|snake_case}}_create", this);
{%- endif -%}

{%- if template_part == "declaration" %}
  clips::UDFValue {{template_type|snake_case}}_create(clips::Environment *env);
{%- endif -%}
