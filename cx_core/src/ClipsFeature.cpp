#include <clipsmm.h>
#include <string>

#include "cx_core/ClipsFeature.hpp"
namespace cx {

ClipsFeature::ClipsFeature(const std::string &feature_name)
    : clips_feature_name{feature_name} {}
// Empty Destructor as delcared virtual
ClipsFeature::~ClipsFeature() {}
} // namespace cx