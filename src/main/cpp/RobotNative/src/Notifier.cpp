#include "Notifier.h"

#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
namespace controls = ctre::phoenix6::controls;
using controls::DutyCycleOut;

#include <units/angular_velocity.h>
using units::angular_velocity::revolutions_per_minute_t;

#include <iostream>

#include <frc/DataLogManager.h>

#include "TBHController.h"

void NotifierRun(NotifierData* data) {
    double currentControlRequest = data->launcherData.launcherFlywheelQueueReader.read();

    DutyCycleOut controlRequest = DutyCycleOut{currentControlRequest};
    controlRequest.EnableFOC = true;

    data->launcherData.flywheelMotor.SetControl(controlRequest);
}

void NotifierHandle::startNotifier() {
    this->notifier.StartPeriodic(this->periodMs);
    frc::DataLogManager::Start();
}

void NotifierHandle::stopNotifier() {
    this->notifier.Stop();
    this->data->launcherData.flywheelMotor.StopMotor();
    frc::DataLogManager::GetLog().Flush();
}