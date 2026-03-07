#ifndef MOTOR_H
#define MOTOR_H

/**
 * Berechnet den Schub eines Motors in Newton.
 * @param throttle Gas von 0.0 bis 1.0
 * @param voltage Batteriespannung in Volt
 * @return Schubkraft in Newton (N)
 */
float getMotorThrustNewtons(float throttle, float voltage);


/**
 * Berechnet den Strom eines Motors in Ampere.
 * @param throttle Gas von 0.0 bis 1.0
 * @param voltage Batteriespannung in Volt
 * @return Strom in Ampere (A)
 */
float getMotorCurrentAmps(float throttle, float voltage);

#endif