// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ProtobufFeature.hpp
 *
 *  Created: October 13th, 2023
 *  Copyright  2023 Daniel Swoboda
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

#ifndef CX_FEATURES__CLIPSPROTOBUFFEATURE_HPP_
#define CX_FEATURES__CLIPSPROTOBUFFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class ProtobufFeature : public ClipsFeature {
public:
  ProtobufFeature();
  ~ProtobufFeature();

  void initialize() override;

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;
  std::mutex mm;
  std::vector<std::string> path_;
  clips::Environment *maintained_env_;
  std::unordered_map<std::string,
                     std::unique_ptr<protobuf_clips::ClipsProtobufCommunicator>>
      protobuf_communicator_;

private:
};

} // namespace cx
#endif // !CX_FEATURES__CLIPSPROTOBUFFEATURE_HPP_
