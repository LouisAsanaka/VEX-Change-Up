#pragma once

/**
 * Constrains the angle to [-pi, pi) radians.
 * 
 * @param radians angle in radians
 * @return angle in [-pi, pi) 
 */
double constrainAnglePi(double radians);

/**
 * Constrains the angle to [0, 2pi] radians.
 * 
 * @param radians angle in radians
 * @return angle in [0, 2pi]
 */
double constrainAngle2Pi(double radians);

/**
 * Constrains the angle to [-180, 180) degrees.
 * 
 * @param degrees angle in degrees
 * @return angle in [-180, 180) 
 */
double constrainAngle180(double degrees);

/**
 * Constrains the angle to [0, 360] degrees.
 * 
 * @param degrees angle in degrees
 * @return angle in [0, 360]
 */
double constrainAngle360(double degrees);
