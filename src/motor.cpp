/**
 * motor.cpp — Motormodell-Implementierung
 * T-Motor F1404 KV4600 + GF3016 Propeller
 *
 * Pipeline: motor_model.m (Octave) → motor_lut.h (auto-generiert) → motor.cpp
 *
 * Beide öffentlichen Funktionen interpolieren direkt in vorberechneten
 * 101-Punkte-Tabellen (Gas 0..1, Schritt 0.01).
 * Da die Tabelle gleichmäßig ist, erfolgt der Zugriff in O(1).
 *
 * Spannungsskalierung:
 *   Schub   ∝ V²  (F ∝ RPM² ∝ V² per KV-Gesetz)
 *   Strom   ∝ V²  (M_prop ∝ ω² ∝ V²  →  I = M/k_T ∝ V²)
 *
 * Quelle: https://n-factory.de/T-Motor-F1404-4600KV-Ultra-Light-Motor
 */

#include "motor.h"
#include <algorithm>
#include "motor_lut.h"

namespace {
    // O(1)-Interpolation in einer gleichmäßigen Tabelle (GAS: 0.0 .. 1.0)
    float tabInterp(float t, const float* values) {
        float idx_f = t * (MOTOR_TAB_SIZE - 1);
        int lo = static_cast<int>(idx_f);
        if (lo >= MOTOR_TAB_SIZE - 1) return values[MOTOR_TAB_SIZE - 1];
        float alpha = idx_f - lo;
        return values[lo] + alpha * (values[lo + 1] - values[lo]);
    }
}

// Schubkraft in Newton: Tabelle Gas→Schub[g] @ 16V, skaliert mit V²
float getMotorThrustNewtons(float throttle, float voltage) {
    if (throttle <= 0.0f) return 0.0f;
    float t = std::min(1.0f, std::max(0.0f, throttle));

    float v_scale = voltage / MOTOR_V_NOMINAL;
    float thrust_N = tabInterp(t, MOTOR_TAB_SCHUB_N) * v_scale * v_scale;
    return std::max(0.0f, thrust_N);
}

// Strom in Ampere: Tabelle Gas→Strom[A] @ 16V, skaliert mit V²
// Herleitung: M_prop ∝ ω²  →  I = M_prop/k_T ∝ ω² ∝ V²
float getMotorCurrentAmps(float throttle, float voltage) {
    float t = std::min(1.0f, std::max(0.0f, throttle));

    float v_scale = voltage / MOTOR_V_NOMINAL;
    float strom_nom = tabInterp(t, MOTOR_TAB_STROM);
    return std::max(MOTOR_I_IDLE, strom_nom * v_scale * v_scale);
}