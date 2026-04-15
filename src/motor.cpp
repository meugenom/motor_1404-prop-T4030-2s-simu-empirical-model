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

        // calc interpolation koeff
        float x0 = x_tab[lo];
        float x1 = x_tab[lo + 1];
        float alpha = (t - x0) / (x1 - x0);

        // 3. Lineal Interpolation Y
        return y_tab[lo] + alpha * (y_tab[lo + 1] - y_tab[lo]);
    }
}

// Schubkraft in Newton: Tabelle Gas→Schub[g] @ 16V, skaliert mit V²
float getMotorThrustNewtons(float throttle, float voltage) {
    // Clamp Gas auf [0.0, 1.0]
    float t = std::max(0.0f, std::min(1.0f, throttle));
    if (t <= 0.0f) return 0.0f;

    // Skalierung bei Spannung: Thrust proportional zu V^2 
    float v_scale = voltage / MOTOR_V_NOMINAL;
    float thrust_nom = interpolate(t, MOTOR_TAB_GAS, MOTOR_TAB_SCHUB_N);
    
    return std::max(0.0f, thrust_nom * v_scale * v_scale);
}

float getMotorCurrentAmps(float throttle, float voltage) {
    float t = std::max(0.0f, std::min(1.0f, throttle));
    
    // Skalierung bei Spannung: 
    // Laststrom proportional zu V^2, aber Leerlaufstrom (I_IDLE) ist normalerweise konstant oder linear.
    float v_scale = voltage / MOTOR_V_NOMINAL;
    float current_nom = interpolate(t, MOTOR_TAB_GAS, MOTOR_TAB_STROM);

    // Richtige physikalische Modellierung: 
    // Nur die Differenz zwischen dem aktuellen Strom und dem Leerlaufstrom wird quadratisch skaliert.
    float load_current = current_nom - MOTOR_I_IDLE;
    float scaled_current = MOTOR_I_IDLE + (load_current * v_scale * v_scale);

    return std::max(MOTOR_I_IDLE, scaled_current);
}