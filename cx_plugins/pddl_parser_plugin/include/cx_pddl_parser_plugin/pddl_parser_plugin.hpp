// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  pddl_parser_plugin.hpp
 *
 *  Created: 15 September 2021
 *  Copyright  2021  Ivaylo Doychev
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

#ifndef CX_PLUGINS__CLIPSPDDLPARSERPLUGIN_HPP_
#define CX_PLUGINS__CLIPSPDDLPARSERPLUGIN_HPP_

#include <map>
#include <memory>
#include <string>

#include <clips_pddl_parser/clips_pddl_parser.h>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class PddlParserPlugin : public ClipsPlugin {
public:
  PddlParserPlugin();
  ~PddlParserPlugin();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::map<std::string, std::unique_ptr<clips_pddl_parser::ClipsPddlParser>>
      pddl_parsers_;

private:
};

} // namespace cx
#endif // !CX_PLUGINS__CLIPSPDDLPARSERPLUGIN_HPP_
