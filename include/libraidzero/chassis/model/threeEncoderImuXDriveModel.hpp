/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "libraidzero/device/betterIMU.hpp"
#include "okapi/api/chassis/model/threeEncoderXDriveModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/units/QAngle.hpp"

using namespace okapi::literals;

class ThreeEncoderImuXDriveModel : public okapi::ThreeEncoderXDriveModel {
public:
    /**
     * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
     * +100%, the robot should move forward in a straight line.
     *
     * @param itopLeftMotor The top left motor.
     * @param itopRightMotor The top right motor.
     * @param ibottomRightMotor The bottom right motor.
     * @param ibottomLeftMotor The bottom left motor.
     * @param ileftEnc The left side encoder.
     * @param irightEnc The right side encoder.
     * @param imiddleEnc The middle encoder.
     * @param iimu The IMU.
     */
    ThreeEncoderImuXDriveModel(
        std::shared_ptr<okapi::AbstractMotor> itopLeftMotor,
        std::shared_ptr<okapi::AbstractMotor> itopRightMotor,
        std::shared_ptr<okapi::AbstractMotor> ibottomRightMotor,
        std::shared_ptr<okapi::AbstractMotor> ibottomLeftMotor,
        std::shared_ptr<okapi::ContinuousRotarySensor> ileftEnc,
        std::shared_ptr<okapi::ContinuousRotarySensor> irightEnc,
        std::shared_ptr<okapi::ContinuousRotarySensor> imiddleEnc,
        std::shared_ptr<BetterIMU> iimu,
        double imaxVelocity,
        double imaxVoltage
    );
  
    /**
     * Read the sensors.
     *
     * @return sensor readings in the format {left, right, middle, heading}
     */
    std::valarray<std::int32_t> getSensorVals() const override;

    double getHeading() const;
  
    /**
     * Reset the sensors to their zero point.
     */
    void resetSensors() override;

    void resetImu(okapi::QAngle iangle = 0_deg);
protected:
    std::shared_ptr<BetterIMU> imu;
};