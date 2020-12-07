#include "libraidzero/chassis/model/threeEncoderImuXDriveModel.hpp"
#include "okapi/api/units/QAngle.hpp"

ThreeEncoderImuXDriveModel::ThreeEncoderImuXDriveModel(
    std::shared_ptr<okapi::AbstractMotor> itopLeftMotor,
    std::shared_ptr<okapi::AbstractMotor> itopRightMotor,
    std::shared_ptr<okapi::AbstractMotor> ibottomRightMotor,
    std::shared_ptr<okapi::AbstractMotor> ibottomLeftMotor,
    std::shared_ptr<okapi::ContinuousRotarySensor> ileftEnc,
    std::shared_ptr<okapi::ContinuousRotarySensor> irightEnc,
    std::shared_ptr<okapi::ContinuousRotarySensor> imiddleEnc,
    std::shared_ptr<BetterIMU> iimu,
    const double imaxVelocity,
    const double imaxVoltage
) : ThreeEncoderXDriveModel(
        std::move(itopLeftMotor),
        std::move(itopRightMotor),
        std::move(ibottomRightMotor),
        std::move(ibottomLeftMotor),
        std::move(ileftEnc),
        std::move(irightEnc),
        std::move(imiddleEnc),
        imaxVelocity,
        imaxVoltage
    ),
    imu{std::move(iimu)} {
}

std::valarray<std::int32_t> ThreeEncoderImuXDriveModel::getSensorVals() const {
    return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                       static_cast<std::int32_t>(rightSensor->get()),
                                       static_cast<std::int32_t>(middleSensor->get())};
}

double ThreeEncoderImuXDriveModel::getHeading() const {
    return imu->getRawAngle();
}

void ThreeEncoderImuXDriveModel::resetSensors() {
    ThreeEncoderXDriveModel::resetSensors();
    resetImu();
}

void ThreeEncoderImuXDriveModel::resetImu(okapi::QAngle angle) {
    imu->reset(angle);
}