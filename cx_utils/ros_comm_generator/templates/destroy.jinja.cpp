// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

// clang-format off
{%- if template_part == "definition" %}
void {{name_camel}}::{{template_type|snake_case}}_destroy({{message_type}}::{{template_type|camel_case}} *g) {
  auto it = {{template_type|snake_case}}s_.find(g);
  if (it != {{template_type|snake_case}}s_.end()) {
      {{template_type|snake_case}}s_.erase(it);
  }
}
{%- endif -%}

{%- if template_part == "registration" %}
  function_names_.insert("{{name_kebab}}-{{template_type|kebab_case}}-destroy");
  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-{{template_type|kebab_case}}-destroy", "v", 1, 1, ";e",
    [](clips::Environment */*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue {{template_type|snake_case}};
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &{{template_type|snake_case}});

      instance->{{template_type|snake_case}}_destroy(static_cast<{{message_type}}::{{template_type|camel_case}}*>({{template_type|snake_case}}.externalAddressValue->contents));
    },
    "destroy_{{template_type|snake_case}}", this);
{%- endif -%}

{%- if template_part == "declaration" %}
  void {{template_type|snake_case}}_destroy({{message_type}}::{{template_type|camel_case}} *{{template_type|snake_case}});
{%- endif -%}
