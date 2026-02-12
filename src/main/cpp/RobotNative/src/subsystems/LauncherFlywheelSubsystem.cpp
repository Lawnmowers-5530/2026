//
// Created by siryellsalot on 2/11/26.
//

#include "subsystems/LauncherFlywheelSubsystem.h"

#include <frc2/command/CommandScheduler.h>

#include <frc2/command/sysid/SysIdRoutine.h>
using frc2::sysid::ramp_rate_t, frc2::sysid::Mechanism;

LauncherFlywheelSubsystem::LauncherFlywheelSubsystem(const LauncherConstants& constants, SPSCQueue<double>& launcherFlywheelQueue):
        flywheelMotor(constants.canId),
        flywheelVelocitySignal(flywheelMotor.GetVelocity()),
        flywheelController(coulombs_per_rad_t { constants.kI } ),
        launcherFlywheelQueueReader(launcherFlywheelQueue.CreateReader()),
        flywheelControlRequest( TorqueCurrentFOC { ampere_t { 0 } } )
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    SysIdConfig sysIdConfig = SysIdConfig {
        ramp_rate_t { 1_V / 1_s },
        7_V,
        std::nullopt,
        nullptr
    };

    Mechanism sysIdMechanism = Mechanism(
        [this] (volt_t voltage) {
            this->SetVoltage(voltage);
        },
        [this] (SysIdRoutineLog* log) {
            log->Motor("Launcher Flywheel")
                .voltage(this->GetAppliedVoltage())
                .velocity(this->GetEncoderVelocity())
                .position(this->GetEncoderPosition());
        },
        this,
        "LauncherFlywheelSubsystem"
    );
}

void LauncherFlywheelSubsystem::SubmitControlRequest(radians_per_second_t request) {
    auto currentVelocity = flywheelVelocitySignal.Refresh().GetValue().convert<radians_per_second>();
    auto output = flywheelController.Calculate(currentVelocity);
    flywheelControlRequest.WithOutput(output);
    flywheelMotor.SetControl(flywheelControlRequest);
}

void LauncherFlywheelSubsystem::Periodic() {
    auto controlRequest = launcherFlywheelQueueReader.read();
    SubmitControlRequest(radians_per_second_t { controlRequest });
}
