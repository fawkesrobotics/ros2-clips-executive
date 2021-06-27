#include <clipsmm.h>
#include <string>

#include "cx_core/ClipsFeature.hpp"
namespace cx {

ClipsFeature::ClipsFeature() {}
// Empty Destructor as delcared virtual
ClipsFeature::~ClipsFeature() {}

std::string ClipsFeature::getFeatureName() const { return clips_feature_name; }

void ClipsFeature::initialise(const std::string &feature_name) {}

} // namespace cx