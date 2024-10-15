// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

// clang-format off
{%- if template_part == "definition" %}
void {{name_camel}}::{{template_type|snake_case}}_{{template_call_fun|snake_case}}(void *{{template_type|snake_case}}_raw, clips::UDFContext *udfc) {
  auto {{template_type|snake_case}} = {{template_type|snake_case}}s_.at({{template_type|snake_case}}_raw);
  if(!{{template_type|snake_case}}) {
    RCLCPP_ERROR(get_logger(), "{{template_type|snake_case}}_{{template_call_fun|snake_case}}: Invalid pointer to {{template_type|camel_case}}");
    clips::UDFThrowError(udfc);
    return;
  }
  {{template_type|snake_case}}->{{template_call_fun|snake_case}}();
}
{%- endif -%}

{%- if template_part == "registration" %}
  function_names_.insert("{{name_kebab}}-{{template_type|kebab_case}}-{{template_call_fun|kebab_case}}");
  clips::AddUDF(
    env.get_obj().get(),"{{name_kebab}}-{{template_type|kebab_case}}-{{template_call_fun|kebab_case}}", "v", 1, 1, ";e",
    [](clips::Environment * /*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue {{template_type|snake_case}};
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &{{template_type|snake_case}});
       try {
        instance->{{template_type|snake_case}}_{{template_call_fun|snake_case}}({{template_type|snake_case}}.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown {{template_type|snake_case}} %s", e.what());
       }
    },
    "{{template_type|snake_case}}_{{template_call_fun|snake_case}}", this);
{%- endif -%}

{%- if template_part == "declaration" %}
  void {{template_type|snake_case}}_{{template_call_fun|snake_case}}(void *raw, clips::UDFContext *udfc);
{%- endif -%}
