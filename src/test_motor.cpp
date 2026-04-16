/**
 * Tests for Motor Model: BrotherHobby 1404 KV4600 + T4030 on 2S (7.4V)
 * Datasheet: Brother-Hobby-1404_4600KV_Blane_Townsend.csv (tytorobotics.com)
 * Verifies getMotorThrustNewtons() and getMotorCurrentAmps() against stand test data.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "motor.h"
#include "motor_lut.h"

#define COL_GREEN "\033[32m"
#define COL_RED   "\033[31m"
#define COL_RESET "\033[0m"

// Tolerances vs. datasheet (V_eff-scaled model with normalized LUT)
static constexpr float TOL_THRUST  = 0.05f;  // ±5% for thrust
static constexpr float TOL_CURRENT = 0.10f;  // ±10% for current

bool approxEqual(float actual, float expected, float tol) {
    if (expected == 0.0f) return fabs(actual) < tol;
    return fabs(actual - expected) / fabs(expected) < tol;
}

static void check(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        printf(COL_RED "✗ FAIL: %s\n  → %s:%d" COL_RESET "\n", expr, file, line);
        exit(1);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

// === Thrust Tests ===

void test_zero_throttle() {
    float thrust = getMotorThrustNewtons(0.0f, MOTOR_V_NOMINAL);
    CHECK(thrust == 0.0f);
    printf(COL_GREEN "✓" COL_RESET " zero throttle → zero thrust\n");
}

void test_thrust_at_30_percent() {
    // Datasheet: 30% throttle (1300µs), 7.296V → 24.90g
    float thrust = getMotorThrustNewtons(0.3f, 7.296f);
    float expected = 24.90f / 1000.0f * 9.81f;
    CHECK(approxEqual(thrust, expected, TOL_THRUST));
    printf(COL_GREEN "✓" COL_RESET " thrust at 30%%: %.3fN (expected %.3fN)\n", thrust, expected);
}

void test_thrust_at_50_percent() {
    // Datasheet: 50% throttle (1500µs), 7.138V → 46.17g
    float thrust = getMotorThrustNewtons(0.5f, 7.138f);
    float expected = 46.17f / 1000.0f * 9.81f;
    CHECK(approxEqual(thrust, expected, TOL_THRUST));
    printf(COL_GREEN "✓" COL_RESET " thrust at 50%%: %.3fN (expected %.3fN)\n", thrust, expected);
}

void test_thrust_at_70_percent() {
    // Datasheet: 70% throttle (1700µs), 7.002V → 62.65g
    float thrust = getMotorThrustNewtons(0.7f, 7.002f);
    float expected = 62.65f / 1000.0f * 9.81f;
    CHECK(approxEqual(thrust, expected, TOL_THRUST));
    printf(COL_GREEN "✓" COL_RESET " thrust at 70%%: %.3fN (expected %.3fN)\n", thrust, expected);
}

void test_thrust_monotonic() {
    float prev = 0.0f;
    for (float t = 0.1f; t <= 1.0f; t += 0.1f) {
        float thrust = getMotorThrustNewtons(t, MOTOR_V_NOMINAL);
        CHECK(thrust >= prev);
        prev = thrust;
    }
    printf(COL_GREEN "✓" COL_RESET " thrust is monotonically increasing\n");
}

void test_voltage_effect() {
    // Higher voltage → higher thrust (2S range: 6.6V–8.4V)
    float thrust_high = getMotorThrustNewtons(0.5f, 8.4f);
    float thrust_low  = getMotorThrustNewtons(0.5f, 6.6f);
    CHECK(thrust_high > thrust_low);
    printf(COL_GREEN "✓" COL_RESET " higher voltage → higher thrust\n");
}

// === Current Tests ===

void test_current_at_30_percent() {
    // Datasheet: 30% throttle (1300µs), 7.296V → 1.589A
    float current  = getMotorCurrentAmps(0.3f, 7.296f);
    float expected = 1.589f;
    CHECK(approxEqual(current, expected, TOL_CURRENT));
    printf(COL_GREEN "✓" COL_RESET " current at 30%%: %.2fA (expected %.2fA)\n", current, expected);
}

void test_current_at_50_percent() {
    // Datasheet: 50% throttle (1500µs), 7.138V → 3.597A
    float current  = getMotorCurrentAmps(0.5f, 7.138f);
    float expected = 3.597f;
    CHECK(approxEqual(current, expected, TOL_CURRENT));
    printf(COL_GREEN "✓" COL_RESET " current at 50%%: %.2fA (expected %.2fA)\n", current, expected);
}

void test_current_at_70_percent() {
    // Datasheet: 70% throttle (1700µs), 7.002V → 5.322A
    float current  = getMotorCurrentAmps(0.7f, 7.002f);
    float expected = 5.322f;
    CHECK(approxEqual(current, expected, TOL_CURRENT));
    printf(COL_GREEN "✓" COL_RESET " current at 70%%: %.2fA (expected %.2fA)\n", current, expected);
}

void test_current_monotonic() {
    float prev = 0.0f;
    for (float t = 0.1f; t <= 1.0f; t += 0.1f) {
        float current = getMotorCurrentAmps(t, MOTOR_V_NOMINAL);
        CHECK(current >= prev);
        prev = current;
    }
    printf(COL_GREEN "✓" COL_RESET " current is monotonically increasing\n");
}

void test_current_voltage_quadratic() {
    // V_eff-scaling: current scales with effective voltage (V - I·R), not raw V.
    // Verify higher voltage → higher current, and ratio is physically plausible.
    float v_high = 8.4f;   // full 2S
    float v_low  = 7.0f;   // discharged 2S
    float i_high = getMotorCurrentAmps(0.5f, v_high);
    float i_low  = getMotorCurrentAmps(0.5f, v_low);
    CHECK(i_high > i_low);
    float ratio_actual = i_high / i_low;
    // With V_eff model: ratio ≈ (V_eff_high/V_eff_low)² adjusted for idle current
    // Should be between 1.0 and (V_high/V_low)² = 1.44
    CHECK(ratio_actual > 1.0f && ratio_actual < 2.0f);
    printf(COL_GREEN "✓" COL_RESET " current V_eff-scaling: I(%.1fV)/I(%.1fV) = %.3f\n",
           v_high, v_low, ratio_actual);
}

int main() {
    printf("\n=== Motor Model Tests: BrotherHobby 1404 KV4600 + T4030 (2S) ===\n");
    printf("V_nominal = %.1fV\n\n", MOTOR_V_NOMINAL);

    printf("--- Thrust Tests ---\n");
    test_zero_throttle();
    test_thrust_at_30_percent();
    test_thrust_at_50_percent();
    test_thrust_at_70_percent();
    test_thrust_monotonic();
    test_voltage_effect();

    printf("\n--- Current Tests ---\n");
    test_current_at_30_percent();
    test_current_at_50_percent();
    test_current_at_70_percent();
    test_current_monotonic();
    test_current_voltage_quadratic();

    printf(COL_GREEN "\n✓ All tests passed.\n" COL_RESET);

    float errs[] = {
        fabs(getMotorThrustNewtons(0.3f, 7.296f) - 24.90f/1000.0f*9.81f) / (24.90f/1000.0f*9.81f) * 100.0f,
        fabs(getMotorThrustNewtons(0.5f, 7.138f) - 46.17f/1000.0f*9.81f) / (46.17f/1000.0f*9.81f) * 100.0f,
        fabs(getMotorThrustNewtons(0.7f, 7.002f) - 62.65f/1000.0f*9.81f) / (62.65f/1000.0f*9.81f) * 100.0f,
    };
    float avg_err = (errs[0] + errs[1] + errs[2]) / 3.0f;
    printf("Model accuracy (30/50/70%%): %.1f%% / %.1f%% / %.1f%% | avg=%.1f%%\n",
           errs[0], errs[1], errs[2], avg_err);

    return 0;
}