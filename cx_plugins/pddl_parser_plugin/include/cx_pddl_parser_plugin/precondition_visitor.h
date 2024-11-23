// Copyright (c) 2024 Carologistics
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Library General Public License for more details.
//
// Read the full text in the LICENSE.GPL file in the main directory.

/***************************************************************************
 *  precondition_visitor.h - A static visitor to translate a precondition
 *
 *  Created: Mon 16 Oct 2017 18:33:34 CEST 18:33
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 *  Copyright  2024  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

#ifndef _PLUGINS_CLIPS_PDDL_PARSER_PRECONDITION_VISITOR_H_
#define _PLUGINS_CLIPS_PDDL_PARSER_PRECONDITION_VISITOR_H_

#include <pddl_parser/pddl_exception.h>
#include <pddl_parser/pddl_parser.h>

#include <boost/variant/variant.hpp>
#include <string>
#include <vector>

namespace clips_pddl_parser {

class PreconditionToCLIPSFactVisitor
    : public boost::static_visitor<std::vector<std::string>> {
public:
  PreconditionToCLIPSFactVisitor(const std::string &parent, int sub_counter,
                                 bool is_main = false);
  std::vector<std::string> operator()(pddl_parser::Atom &a) const;
  std::vector<std::string> operator()(pddl_parser::Predicate &p) const;
  std::vector<std::string> operator()(pddl_parser::QuantifiedFormula &q) const;

private:
  std::string parent_;
  uint sub_counter_;
  bool is_main_;
};

} // end namespace clips_pddl_parser

#endif /* !PLUGINS_CLIPS_PDDL_PARSER_PRECONDITION_VISITOR_H__ */
