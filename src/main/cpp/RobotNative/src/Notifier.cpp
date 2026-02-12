#include "Notifier.h"

#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
namespace controls = ctre::phoenix6::controls;
using controls::DutyCycleOut;

#include <units/angular_velocity.h>
using units::angular_velocity::revolutions_per_minute_t;

#include <frc/DataLogManager.h>
#include "subsystems/LauncherFlywheelSubsystem.h"

#include <wpi/DataLog.h>

NotifierData::NotifierData(const Constants &constants, SPSCQueue<double> &launcherFlywheelQueue) :
    launcherFlywheelSubsystem {
        constants.launcherConstants,
        launcherFlywheelQueue
    },
    scheduler(
        CommandScheduler::GetInstance()
    )
{}

NotifierHandle::NotifierHandle(const Constants &constants, hertz_t frequency):
        launcherFlywheelQueue(SPSCQueue<double>()),
        data(new NotifierData(constants, launcherFlywheelQueue)),
        notifier(NotifierRun, data),
        periodMs(frequency)
{}

void NotifierRun(NotifierData* data) {
    data->scheduler.Run();
}

void NotifierHandle::startNotifier() {
    this->notifier.StartPeriodic(this->periodMs);
    frc::DataLogManager::Start();
}

void NotifierHandle::stopNotifier() {
    this->notifier.Stop();
    this->data->launcherFlywheelSubsystem.flywheelMotor.StopMotor();
    frc::DataLogManager::GetLog().Flush();
}

void NotifierHandle::submitLauncherControlRequest(double controlRequest) {
    this->launcherFlywheelQueue.write(controlRequest);
}