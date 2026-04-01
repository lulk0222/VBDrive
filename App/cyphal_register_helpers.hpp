#pragma once

#include <cmath>
#include <utility>

#include <cyphal/node/registers_utils.hpp>

namespace vbdrive::cyphal_register_helpers {

inline bool try_extract_finite_real32(const uavcan_register_Value_1_0& value, float& out) {
    if (value._tag_ == REGISTER_EMPTY_TAG) {
        return false;
    }
    if (!parse_register_real32(value, out)) {
        return false;
    }
    return std::isfinite(out);
}

template <typename SaveCallback>
inline bool handle_config_save_request(const uavcan_register_Value_1_0& value, SaveCallback&& on_save) {
    if (value._tag_ == REGISTER_EMPTY_TAG) {
        return false;
    }

    bool should_save = false;
    if (!parse_register_bit(value, should_save) || !should_save) {
        return false;
    }

    std::forward<SaveCallback>(on_save)();
    return true;
}

}  // namespace vbdrive::cyphal_register_helpers
