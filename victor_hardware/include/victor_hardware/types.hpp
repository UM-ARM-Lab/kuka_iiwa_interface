#pragma once

#include <memory>
#include <vector>
#include <lcm/lcm-cpp.hpp>

namespace victor_hardware {

using LCMPtr = std::shared_ptr<lcm::LCM>;
using LCMPtrs = std::vector<LCMPtr>;

}  // namespace victor_hardware