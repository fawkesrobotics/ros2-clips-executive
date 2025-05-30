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
 *  clips_pddl_parser.h - PDDL parser for CLIPS
 *
 *  Created: Tue Apr 16 13:41:13 2013
 *  Copyright  2013-2014  Tim Niemueller [www.niemueller.de]
 *             2021       Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 *             2024       Tarik Viehmann<viehmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

#ifndef _CLIPS_PDDL_PARSER_H_
#define _CLIPS_PDDL_PARSER_H_
#include <clips_ns/clips.h>
#undef LHS // to avoid clash with boost
#undef RHS // to avoid clash with boost
#include <list>
#include <mutex>
#include <string>

namespace clips_pddl_parser {

class ClipsPddlParser {
public:
  ClipsPddlParser(clips::Environment *env, std::mutex &env_mutex);
  ~ClipsPddlParser();

private:
  void setup_clips();
  void parse_domain(std::string domain_file);
  void parse_formula(std::string formula, std::string output_id);

private:
  clips::Environment *clips_;
  std::mutex &clips_mutex_;

  std::list<std::string> functions_;
};

} // end namespace clips_pddl_parser

#endif
