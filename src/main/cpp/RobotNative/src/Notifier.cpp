#include "Notifier.h"

#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
namespace controls = ctre::phoenix6::controls;
using controls::VelocityTorqueCurrentFOC;

#include <units/angular_velocity.h>
using units::angular_velocity::revolutions_per_minute_t;

void NotifierRun(NotifierData* data) {
    double currentControlRequest = data->launcherData.launcherFlywheelQueueReader.read();
    auto speed = revolutions_per_minute_t { currentControlRequest };

    VelocityTorqueCurrentFOC controlRequest = VelocityTorqueCurrentFOC { speed };
    data->launcherData.flywheelMotor.SetControl(controlRequest);
}

void NotifierHandle::startNotifier() {
    this->notifier.StartPeriodic(this->periodMs);
}

void NotifierHandle::stopNotifier() {
    this->notifier.Stop();
}