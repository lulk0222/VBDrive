#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

#include "App/cyphal_register_helpers.hpp"

namespace {

int failures = 0;

void expect_true(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_false(bool condition, const char* message) {
    expect_true(!condition, message);
}

void expect_eq_u32(std::uint32_t lhs, std::uint32_t rhs, const char* message) {
    if (lhs != rhs) {
        std::cerr << "FAIL: " << message << " expected=" << rhs << " actual=" << lhs << "\n";
        failures++;
    }
}

void test_try_extract_finite_real32_rejects_empty() {
    uavcan_register_Value_1_0 value = {};
    float out = 0.0F;
    expect_false(vbdrive::cyphal_register_helpers::try_extract_finite_real32(value, out),
                 "empty register value must not be treated as write");
}

void test_try_extract_finite_real32_accepts_real32() {
    uavcan_register_Value_1_0 value = {};
    value._tag_ = REGISTER_REAL32_TAG;
    value.real32.value.count = 1;
    value.real32.value.elements[0] = 1.25F;

    float out = 0.0F;
    expect_true(vbdrive::cyphal_register_helpers::try_extract_finite_real32(value, out),
                "finite real32 should be accepted");
    expect_true(std::fabs(out - 1.25F) < 1.0e-6F,
                "accepted real32 value should be returned unchanged");
}

void test_try_extract_finite_real32_rejects_nan() {
    uavcan_register_Value_1_0 value = {};
    value._tag_ = REGISTER_REAL32_TAG;
    value.real32.value.count = 1;
    value.real32.value.elements[0] = std::numeric_limits<float>::quiet_NaN();

    float out = 0.0F;
    expect_false(vbdrive::cyphal_register_helpers::try_extract_finite_real32(value, out),
                 "NaN real32 must be rejected");
}

void test_handle_config_save_request_triggers_on_true() {
    uavcan_register_Value_1_0 value = {};
    value._tag_ = REGISTER_BIT_TAG;
    value.bit.value.count = 1;
    value.bit.value.bitpacked[0] = 1;

    std::uint32_t invocations = 0;
    const bool triggered = vbdrive::cyphal_register_helpers::handle_config_save_request(
        value,
        [&invocations]() { invocations++; }
    );

    expect_true(triggered, "save trigger should report true on bit=true");
    expect_eq_u32(invocations, 1U, "save callback should be invoked exactly once on bit=true");
}

void test_handle_config_save_request_ignores_false_or_non_bit() {
    std::uint32_t invocations = 0;

    uavcan_register_Value_1_0 false_bit = {};
    false_bit._tag_ = REGISTER_BIT_TAG;
    false_bit.bit.value.count = 1;
    false_bit.bit.value.bitpacked[0] = 0;

    const bool triggered_false = vbdrive::cyphal_register_helpers::handle_config_save_request(
        false_bit,
        [&invocations]() { invocations++; }
    );

    uavcan_register_Value_1_0 wrong_tag = {};
    wrong_tag._tag_ = REGISTER_REAL32_TAG;
    wrong_tag.real32.value.count = 1;
    wrong_tag.real32.value.elements[0] = 1.0F;

    const bool triggered_wrong_tag = vbdrive::cyphal_register_helpers::handle_config_save_request(
        wrong_tag,
        [&invocations]() { invocations++; }
    );

    expect_false(triggered_false, "save trigger should be false on bit=false");
    expect_false(triggered_wrong_tag, "save trigger should be false on non-bit values");
    expect_eq_u32(invocations, 0U, "save callback should not be called for false/non-bit");
}

}  // namespace

int main() {
    test_try_extract_finite_real32_rejects_empty();
    test_try_extract_finite_real32_accepts_real32();
    test_try_extract_finite_real32_rejects_nan();
    test_handle_config_save_request_triggers_on_true();
    test_handle_config_save_request_ignores_false_or_non_bit();

    if (failures != 0) {
        std::cerr << "Total failures: " << failures << "\n";
        return 1;
    }

    std::cout << "All cyphal register helper tests passed\n";
    return 0;
}
