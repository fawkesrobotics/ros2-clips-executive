// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  PddlParserFeature.hpp
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

#ifndef CX_FEATURES__CLIPSPDDLPARSERFEATURE_HPP_
#define CX_FEATURES__CLIPSPDDLPARSERFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include <clips_pddl_parser/clips_pddl_parser.h>

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class PddlParserFeature : public ClipsFeature {
public:
  PddlParserFeature();
  ~PddlParserFeature();

  void initialize(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<clips::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;
  std::unique_ptr<clips_pddl_parser::ClipsPddlParser> pddl_parser;

private:
};

} // namespace cx
#endif // !CX_FEATURES__CLIPSPDDLPARSERFEATURE_HPP_
