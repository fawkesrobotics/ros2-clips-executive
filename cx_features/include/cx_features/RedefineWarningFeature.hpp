/***************************************************************************
 *  RedefineWarningFeature.hpp
 *
 *  Created: 30 June 2021
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

#ifndef CX_FEATURES__REDEFINEWARNINGFEATURE_HPP_
#define CX_FEATURES__REDEFINEWARNINGFEATURE_HPP_

#include <clipsmm.h>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class RedefineWarningFeature : public ClipsFeature {
public:
  RedefineWarningFeature();
  ~RedefineWarningFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
};

} // namespace cx

#endif // !CX_FEATURES__REDEFINEWARNINGFEATURE_HPP_