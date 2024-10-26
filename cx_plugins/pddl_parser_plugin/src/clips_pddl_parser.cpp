
// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  clips_pddl_parser.cpp - Parse PDDL files to a CLIPS environment
 *
 *  Created: Tue Apr 16 13:51:14 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 *             2021  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cx_pddl_parser_plugin/clips_pddl_parser.h>
#include <cx_pddl_parser_plugin/effect_visitor.h>
#include <cx_pddl_parser_plugin/precondition_visitor.h>
#include <pddl_parser/pddl_exception.h>
#include <pddl_parser/pddl_parser.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>

using namespace std;
using namespace pddl_parser;

namespace clips_pddl_parser {

/** @class ClipsPddlParser <clips_pddl_parser/communicator.h>
 * PDDL parser CLIPS integration class.
 * This class provides functionality to parse a PDDL domain to a given CLIPS
 * environment. It stores the relevant domain information to dedicated facts.
 */

/** Constructor.
 * @param env CLIPS environment to which to provide the PDDL parsing
 * functionality
 * @param env_mutex mutex to lock when operating on the CLIPS environment.
 */
ClipsPddlParser::ClipsPddlParser(clips::Environment *env, std::mutex &env_mutex)
    : clips_(env), clips_mutex_(env_mutex) {
  setup_clips();
}

/** Destructor. */
ClipsPddlParser::~ClipsPddlParser() {
  {
    for (auto f : functions_) {
      clips::RemoveUDF(clips_, f.c_str());
    }
    functions_.clear();
  }
}

/** Setup CLIPS environment.
 */
void ClipsPddlParser::setup_clips() {
  clips::AddUDF(
      clips_, "parse-pddl-domain", "v", 1, 1, ";sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsPddlParser *instance =
            static_cast<ClipsPddlParser *>(udfc->context);
        clips::UDFValue domain_file;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &domain_file);
        instance->parse_domain(domain_file.lexemeValue->contents);
      },
      "parse_domain", this);
  functions_.push_back("parse-pddl-domain");
  clips::AddUDF(
      clips_, "parse-pddl-formula", "v", 2, 2, ";sy;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsPddlParser *instance =
            static_cast<ClipsPddlParser *>(udfc->context);
        clips::UDFValue formula;
        clips::UDFValue id;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &formula);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &id);
        instance->parse_formula(formula.lexemeValue->contents,
                                id.lexemeValue->contents);
      },
      "parse_pddl_formula", this);
}
/** CLIPS function to parse a PDDL domain.
 * This parses the given domain and asserts domain facts for all parts of the
 * domain.
 * @param domain_file The path of the domain file to parse.
 */
void ClipsPddlParser::parse_domain(std::string domain_file) {
  Domain domain;
  {
    Domain domain;
    try {
      ifstream df(domain_file);
      stringstream buffer;
      buffer << df.rdbuf();
      domain = PddlParser::parseDomain(buffer.str());
    } catch (PddlParserException &e) {
      SPDLOG_WARN(std::string("CLIPS_PDDL_Parser: Failed to parse domain:") +
                  e.what());
      return;
    }
    for (const auto &temp : {"pddl-formula", "pddl-predicate", "domain-effect",
                             "domain-object-type", "domain-predicate",
                             "domain-operator-parameter", "domain-operator"}) {
      clips::Deftemplate *templ = clips::FindDeftemplate(clips_, temp);
      if (!templ) {
        SPDLOG_WARN(std::string("CLIPS_PDDL_Parser: Did not get template ") +
                    temp + ", did you load pddl_domain.clp?");
      }

      string super_type = "";
      for (auto &type : domain.types) {
        string super_type = "";
        if (!type.second.empty()) {
          super_type = "(super-type " + type.second + ")";
        }
        clips::AssertString(clips_, ("(domain-object-type "
                                     "(name " +
                                     type.first + ")" + super_type + ")")
                                        .c_str());
      }

      for (auto &predicate : domain.predicates) {
        string param_string = "";
        string type_string = "";
        for (auto &param : predicate.second) {
          param_string += " " + param.first;
          type_string += " " + param.second;
        }
        clips::AssertString(clips_, ("(domain-predicate"
                                     " (name " +
                                     predicate.first +
                                     ")"
                                     " (param-names " +
                                     param_string +
                                     ")"
                                     " (param-types " +
                                     type_string +
                                     ")"
                                     ")")
                                        .c_str());
      }

      for (auto &action : domain.actions) {
        string params_string = "(param-names";
        for (auto &param_pair : action.action_params) {
          string param_name = param_pair.first;
          string param_type = param_pair.second;
          params_string += " " + param_name;
          clips::AssertString(clips_, ("(domain-operator-parameter"
                                       " (name " +
                                       param_name +
                                       ")"
                                       " (operator " +
                                       action.name +
                                       ")"
                                       " (type " +
                                       param_type +
                                       ")"
                                       ")")
                                          .c_str());
        }
        params_string += ")";
        clips::AssertString(clips_, ("(domain-operator (name " + action.name +
                                     ")" + params_string + ")")
                                        .c_str());
        vector<string> precondition_facts = boost::apply_visitor(
            PreconditionToCLIPSFactVisitor(action.name, 1, true),
            action.precondition.expression);
        for (auto &fact : precondition_facts) {
          clips::AssertString(clips_, fact.c_str());
        }
        vector<string> effect_facts =
            boost::apply_visitor(EffectToCLIPSFactVisitor(action.name, true),
                                 action.effect.expression);
        for (auto &fact : effect_facts) {
          clips::AssertString(clips_, fact.c_str());
        }
      }
    }
  }
}

/** CLIPS function to parse a PDDL formula.
 * This parses the given domain and asserts domain facts for all parts of the
 * domain.
 * @param pddl_formula The formula as string
 * @param output_id The id that the generated CLIPS fact should have
 */
void ClipsPddlParser::parse_formula(std::string pddl_formula,
                                    std::string output_id) {
  Expression formula;
  try {
    formula = PddlParser::parseFormula(pddl_formula);
  } catch (PddlParserException &e) {
    SPDLOG_ERROR("PDDLCLIPS: Failed to parse formula: %s", e.what());
  }
  vector<string> formula_facts = boost::apply_visitor(
      PreconditionToCLIPSFactVisitor(output_id, 1, true), formula.expression);
  for (const auto &fact : formula_facts) {
    clips::AssertString(clips_, fact.c_str());
  }
}

} // end namespace clips_pddl_parser
