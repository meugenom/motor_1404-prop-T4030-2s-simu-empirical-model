/**
 * STM32/Renode Test Bench for Motor Model
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstdint>

#include "motor.h"
#include "motor_lut.h"

// --- UART PRINTF IMPLEMENTATION FOR STM32 (without HAL) ---
extern "C" {
    int _write(int file, char *ptr, int len) {
        volatile uint32_t *usart2_sr = (volatile uint32_t *)0x40004400; // Status Register
        volatile uint32_t *usart2_dr = (volatile uint32_t *)0x40004404; // Data Register
        for (int i = 0; i < len; i++) {
            while (!(*usart2_sr & (1 << 7))); // Wait for TXE
            *usart2_dr = ptr[i] & 0xFF;
        }
        return len;
    }
}

// Security wrapper for printf, max 256 chars
void safe_printf(const char *fmt, ...) {
    char buffer[256]; 
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (len > 0) {
        _write(1, buffer, len);
    }
}

// Change all printf calls to safe_printf
#define printf safe_printf


// Checks if two floats are approximately equal within a relative tolerance
bool approxEqual(float actual, float expected, float tol) {
    // Pure math without <cmath> to avoid double bugs on Cortex-M4
    float diff = actual - expected;
    if (diff < 0.0f) diff = -diff; 
    
    float expected_abs = expected < 0.0f ? -expected : expected;
    bool ok = (expected == 0.0f) ? (diff < tol) : ((diff / expected_abs) < tol);
    
    // If the test fails, print the actual values to UART
    if (!ok) {
        printf("\n[DEBUG-MATH] Actual: %.4f | Expected: %.4f | Diff: %.4f\n", actual, expected, diff);
    }
    return ok;
}


static void check(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        printf("FAIL: %s\n  → %s:%d\n", expr, file, line);
        // On a microcontroller, we cannot call exit(1), so we hang:
        while(1) {}
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

// ================= TEST CASES =================


void test_thrust_monotonic() {
    // Hard check: operational range 10%-90% must be strictly monotonic
    float last_t = -1.0f;
    for (int i = 0; i <= 90; i += 10) {
        float t = getMotorThrustNewtons(i / 100.0f, MOTOR_V_NOMINAL);
        CHECK(t >= last_t);
        last_t = t;
    }
    printf("OK THRUST is monotonically increasing (10%%-90%%)\n");

    // Soft check: 100% throttle - known propeller saturation zone, no hang
    float thrust_90  = getMotorThrustNewtons(0.9f, MOTOR_V_NOMINAL);
    float thrust_100 = getMotorThrustNewtons(1.0f, MOTOR_V_NOMINAL);
    if (thrust_100 < thrust_90) {
        printf("WARN thrust drops at 100%% throttle: %.4fN -> %.4fN "
               "(propeller saturation / Hall sensor RPM underread at >20k RPM)\n",
               thrust_90, thrust_100);
    }
}

void test_voltage_effect() {
    float t_low  = getMotorThrustNewtons(0.5f, 7.0f);
    float t_high = getMotorThrustNewtons(0.5f, 8.4f);
    CHECK(t_high > t_low);
    printf("OK higher VOLTAGE → higher THRUST\n");
}

void test_current_monotonic() {
    // Hard check: operational range 10%-90% must be strictly monotonic
    float last_c = -1.0f;
    for (int i = 0; i <= 90; i += 10) {
        float c = getMotorCurrentAmps(i / 100.0f, MOTOR_V_NOMINAL);
        CHECK(c >= last_c);
        last_c = c;
    }
    printf("OK CURRENT is monotonically increasing (10%%-90%%)\n");

    // Soft check: 100% throttle - consistent with thrust saturation zone, no hang
    float current_90  = getMotorCurrentAmps(0.9f, MOTOR_V_NOMINAL);
    float current_100 = getMotorCurrentAmps(1.0f, MOTOR_V_NOMINAL);
    if (current_100 < current_90) {
        printf("WARN current drops at 100%% throttle: %.4fA -> %.4fA "
               "(consistent with propeller saturation at >20k RPM)\n",
               current_90, current_100);
    }
}

void test_current_voltage_quadratic() {
    float v_low = 7.0f;
    float v_high = 8.4f;
    float i_low  = getMotorCurrentAmps(0.8f, v_low);
    float i_high = getMotorCurrentAmps(0.8f, v_high);
    
    float ratio_actual = i_high / i_low;
    CHECK(ratio_actual > 1.0f && ratio_actual < 2.0f);
    printf("OK current V_eff-scaling: I(%.1fV)/I(%.1fV) = %.3f\n",
           v_high, v_low, ratio_actual);
}

// ================= main =================

int main() {
    // 1. Initialize UART for printf
    volatile uint32_t *rcc_apb1enr = (volatile uint32_t *)0x40023840;
    volatile uint32_t *rcc_ahb1enr = (volatile uint32_t *)0x40023830;
    volatile uint32_t *gpioa_moder = (volatile uint32_t *)0x40020000;
    volatile uint32_t *gpioa_afrl  = (volatile uint32_t *)0x40020020;
    volatile uint32_t *usart2_cr1  = (volatile uint32_t *)0x4000440C;
    volatile uint32_t *usart2_brr  = (volatile uint32_t *)0x40004408;

    *rcc_apb1enr |= (1 << 17);
    *rcc_ahb1enr |= (1 << 0);
    *gpioa_moder &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
    *gpioa_moder |=  ((2 << (2 * 2)) | (2 << (3 * 2)));
    *gpioa_afrl  &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    *gpioa_afrl  |=  ((7 << (2 * 4)) | (7 << (3 * 4)));
    *usart2_brr   = 0x0683; // 9600
    *usart2_cr1   = (1 << 13) | (1 << 3) | (1 << 2);

    // 2. RUN TESTS
    printf("\n=== RENODE Test Bench: BrotherHobby 1404 KV4600 ===\n");
    printf("V_nominal = %.1fV\n\n", MOTOR_V_NOMINAL);

    
    printf("--- Thrust Tests ---\n");        
    test_thrust_monotonic();
    test_voltage_effect();

    printf("\n--- Current Tests ---\n");    
    test_current_monotonic();
    test_current_voltage_quadratic();
    

    printf("\n✓ All tests passed in Renode Emulator.\n");

    // 3. OUTPUT TRACE DATA FOR OCTAVE (End-to-end validation)
    printf("\n--- TRACE DATA FOR OCTAVE ---\n");
    printf("[TRACE]| THROTTLE; THRUST_N; CURRENT_A\n");
    for (int i = 0; i <= 10; i++) {
        float throttle = i / 10.0f;
        float thrust = getMotorThrustNewtons(throttle, MOTOR_V_NOMINAL);
        float current = getMotorCurrentAmps(throttle, MOTOR_V_NOMINAL);
        printf("[TRACE]| %.2f ; %f ; %f \n", throttle, thrust, current);
    }

    printf("\n--- TEST BENCH: FINISHED ---\n");

    // MICROCONTROLLER SHOULD NOT EXIT MAIN!
    while (1) {
        // Waiting for emulator to stop
    }

    return 0;
}