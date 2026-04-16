#include "motor.h"
#include <algorithm>
#include "motor_lut.h"

namespace {
    /**
     * O(1)-Interpolation in einer gleichmäßigen Tabelle (GAS: 0.0 .. 1.0)
     * @param t - current value Gas (0.0. ... 1.0)
     * @param x_tab - values table X (MOTOR_TAB_GAS)
     * @param y_tab - values table Y (f-e- MOTOR_TAB_SHUB)
     */
    float interpolate(float t, const float* x_tab, const float* y_tab) {
        float idx_f = t * (MOTOR_TAB_SIZE - 1);
        int lo = static_cast<int>(idx_f);

        if (lo >= MOTOR_TAB_SIZE - 1) return y_tab[MOTOR_TAB_SIZE - 1];
        if (lo < 0) return y_tab[0];

        // calculation interpolation koeffizient
        float x0 = x_tab[lo];
        float x1 = x_tab[lo + 1];
        float alpha = (t - x0) / (x1 - x0);

        // 3. Lineal Interpolation Y
        return y_tab[lo] + alpha * (y_tab[lo + 1] - y_tab[lo]);
    }
}

// Schubkraft in Newton: Tabelle Gas→Schub[N] @ V_eff_nom, skaliert mit V_eff^2
// Two Newton-Raphson iterations resolve the algebraic loop I(V_eff) <-> V_eff(I)
float getMotorThrustNewtons(float throttle, float voltage) {
    float t = std::max(0.0f, std::min(1.0f, throttle));
    if (t <= 0.0f) return 0.0f;

    float i_nom = interpolate(t, MOTOR_TAB_GAS, MOTOR_TAB_STROM);
    float thrust_nom = interpolate(t, MOTOR_TAB_GAS, MOTOR_TAB_SCHUB_N);

    float v_eff_nom = MOTOR_V_NOMINAL - i_nom * MOTOR_R_INTERNAL;
    if (v_eff_nom <= 0.0f) return 0.0f;

    // Zeroth-order V_eff estimate (uses i_nom for IR drop)
    float v_eff = voltage - i_nom * MOTOR_R_INTERNAL;
    if (v_eff <= 0.0f) return 0.0f;

    // Two Newton-Raphson iterations: estimate I_actual from V_eff ratio, refine V_eff
    for (int nr = 0; nr < 2; nr++) {
        float ratio = v_eff / v_eff_nom;
        float i_est = SYSTEM_STANDBY_CURRENT + (i_nom - SYSTEM_STANDBY_CURRENT) * (ratio * ratio);
        v_eff = voltage - i_est * MOTOR_R_INTERNAL;
        if (v_eff <= 0.0f) return 0.0f;
    }

    float v_scale = v_eff / v_eff_nom;
    return std::max(0.0f, thrust_nom * (v_scale * v_scale));
}

// Strom in Ampere: skaliert mit V_eff^2, Idle-Strom spannungsunabhängig
// MOTOR_I_IDLE (0.11A) is the ESC/system standby current at 0 RPM,
// not the motor's mechanical idle current (0.45A at spinning idle).
float getMotorCurrentAmps(float throttle, float voltage) {
    float t = std::max(0.0f, std::min(1.0f, throttle));
    if (t <= 0.0f) return SYSTEM_STANDBY_CURRENT;  // system standby — motor not spinning

    float i_nom = interpolate(t, MOTOR_TAB_GAS, MOTOR_TAB_STROM);
    float v_eff_nom = MOTOR_V_NOMINAL - i_nom * MOTOR_R_INTERNAL;
    if (v_eff_nom <= 0.0f) return SYSTEM_STANDBY_CURRENT;

    float v_eff = voltage - i_nom * MOTOR_R_INTERNAL;
    if (v_eff <= 0.0f) return SYSTEM_STANDBY_CURRENT;

    // Two Newton-Raphson iterations
    for (int nr = 0; nr < 2; nr++) {
        float ratio = v_eff / v_eff_nom;
        float i_est = SYSTEM_STANDBY_CURRENT + (i_nom - SYSTEM_STANDBY_CURRENT) * (ratio * ratio);
        v_eff = voltage - i_est * MOTOR_R_INTERNAL;
        if (v_eff <= 0.0f) return SYSTEM_STANDBY_CURRENT;
    }

    float v_scale = v_eff / v_eff_nom;
    float load_current = i_nom - SYSTEM_STANDBY_CURRENT;
    float scaled_current = SYSTEM_STANDBY_CURRENT + (load_current * v_scale * v_scale);

    return std::max(SYSTEM_STANDBY_CURRENT, scaled_current);
}