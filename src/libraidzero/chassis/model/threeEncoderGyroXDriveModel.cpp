#include "libraidzero/chassis/model/threeEncoderGyroXDriveModel.hpp"

ThreeEncoderGyroXDriveModel::ThreeEncoderGyroXDriveModel(std::shared_ptr<okapi::AbstractMotor> itopLeftMotor,
                                                         std::shared_ptr<okapi::AbstractMotor> itopRightMotor,
                                                         std::shared_ptr<okapi::AbstractMotor> ibottomRightMotor,
                                                         std::shared_ptr<okapi::AbstractMotor> ibottomLeftMotor,
                                                         std::shared_ptr<okapi::ContinuousRotarySensor> ileftEnc,
                                                         std::shared_ptr<okapi::ContinuousRotarySensor> irightEnc,
                                                         std::shared_ptr<okapi::ContinuousRotarySensor> imiddleEnc,
                                                         std::shared_ptr<pros::Imu> igyro,
                                                         const double imaxVelocity,
                                                         const double imaxVoltage)
    : ThreeEncoderXDriveModel(std::move(itopLeftMotor),
                              std::move(itopRightMotor),
                              std::move(ibottomRightMotor),
                              std::move(ibottomLeftMotor),
                              std::move(ileftEnc),
                              std::move(irightEnc),
                              std::move(imiddleEnc),
                              imaxVelocity,
                              imaxVoltage),
      gyro(std::move(igyro)) {
}

std::valarray<std::int32_t> ThreeEncoderGyroXDriveModel::getSensorVals() const {
    // Return the gyro heading last so this is compatible with ThreeEncoderXDriveModel::getSensorVals()
    return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                       static_cast<std::int32_t>(rightSensor->get()),
                                       static_cast<std::int32_t>(middleSensor->get()),
                                       static_cast<std::int32_t>(gyro->get_rotation() - headingOffset)};
}

void ThreeEncoderGyroXDriveModel::resetSensors() {
    ThreeEncoderXDriveModel::resetSensors();
    headingOffset = gyro->get_rotation();
}