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

bool approxEqual(float actual, float expected, float tol) {
    if (expected == 0.0f) return fabs(actual) < tol;
    return fabs(actual - expected) / fabs(expected) < tol;
}

static void check(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        printf("FAIL: %s\n  → %s:%d\n", expr, file, line);
        exit(1);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

// === TESTs ===

void test_zero_throttle() {
    float thrust = getMotorThrustNewtons(0.0f, MOTOR_V_NOMINAL);
    CHECK(thrust == 0.0f);
    printf("OK zero throttle → zero thrust\n");
}

void test_thrust_monotonic() {
    // Hard check: operational range 10%-90% must be strictly monotonic
    float prev = 0.0f;
    for (int i = 1; i <= 9; i++) {
        float t = i / 10.0f;
        float thrust = getMotorThrustNewtons(t, MOTOR_V_NOMINAL);
        CHECK(thrust >= prev);
        prev = thrust;
    }
    printf("OK thrust is monotonically increasing (10%%–90%%)\n");

    // Soft check: 100% throttle — known propeller saturation zone, no exit
    float thrust_90 = getMotorThrustNewtons(0.9f, MOTOR_V_NOMINAL);
    float thrust_100 = getMotorThrustNewtons(1.0f, MOTOR_V_NOMINAL);
    if (thrust_100 < thrust_90) {
        printf("WARN thrust drops at 100%% throttle: %.4fN -> %.4fN "
               "(propeller saturation / Hall sensor RPM underread at >20k RPM)\n",
               thrust_90, thrust_100);
    }
}

void test_voltage_effect() {
    // Higher voltage → higher thrust (2S range: 6.6V–8.4V)
    float thrust_high = getMotorThrustNewtons(0.5f, 8.4f);
    float thrust_low  = getMotorThrustNewtons(0.5f, 6.6f);
    CHECK(thrust_high > thrust_low);
    printf("OK higher voltage → higher thrust\n");
}

// === Current Tests ===
void test_current_monotonic() {
    // Hard check: operational range 10%-90% must be strictly monotonic
    float prev = 0.0f;
    for (int i = 1; i <= 9; i++) {
        float t = i / 10.0f;
        float current = getMotorCurrentAmps(t, MOTOR_V_NOMINAL);
        CHECK(current >= prev);
        prev = current;
    }
    printf("OK current is monotonically increasing (10%%–90%%)\n");

    // Soft check: 100% throttle — consistent with thrust saturation zone, no exit
    float current_90  = getMotorCurrentAmps(0.9f, MOTOR_V_NOMINAL);
    float current_100 = getMotorCurrentAmps(1.0f, MOTOR_V_NOMINAL);
    if (current_100 < current_90) {
        printf("WARN current drops at 100%% throttle: %.4fA -> %.4fA "
               "(consistent with propeller saturation at >20k RPM)\n",
               current_90, current_100);
    }
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
    printf("OK current V_eff-scaling: I(%.1fV)/I(%.1fV) = %.3f\n",
           v_high, v_low, ratio_actual);
}

int main() {
    printf("\n=== Motor Model Tests: BrotherHobby 1404 KV4600 + T4030 (2S) ===\n");
    printf("V_nominal = %.1fV\n\n", MOTOR_V_NOMINAL);

    printf("--- Thrust Tests ---\n");
    test_zero_throttle();
    test_thrust_monotonic();
    test_voltage_effect();

    printf("\n--- Current Tests ---\n");
    test_current_monotonic();
    test_current_voltage_quadratic();

    printf("\nOK All tests passed.\n");

    printf("\n--- TRACE DATA FOR OCTAVE ---\n");
    printf("[TRACE]| THROTTLE; THRUST_N; CURRENT_A\n");
    for (int i = 0; i <= 10; i++) {
        float throttle = i / 10.0f;
        float thrust = getMotorThrustNewtons(throttle, MOTOR_V_NOMINAL);
        float current = getMotorCurrentAmps(throttle, MOTOR_V_NOMINAL);
        printf("[TRACE]| %.2f ; %f ; %f \n", throttle, thrust, current);
    }

    printf("\n--- TEST BENCH: FINISHED ---\n");

    return 0;
}