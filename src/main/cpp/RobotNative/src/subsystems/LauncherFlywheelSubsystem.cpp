//
// Created by siryellsalot on
// 2/11/26.
//

#include "subsystems/LauncherFlywheelSubsystem.h"

#include <frc2/command/CommandScheduler.h>

#include <frc/sysid/SysIdRoutineLog.h>
#include <frc2/command/sysid/SysIdRoutine.h>
using frc::sysid::SysIdRoutineLog;
using frc2::sysid::ramp_rate_t, frc2::sysid::Mechanism;

LauncherFlywheelSubsystem::LauncherFlywheelSubsystem(
    const LauncherConstants &constants,
    SPSCQueue<double> &launcherFlywheelQueue ) :
    flywheelMotor( constants.canId ),
    flywheelVoltageSignal( flywheelMotor.GetMotorVoltage() ),
    flywheelVelocitySignal( flywheelMotor.GetVelocity() ),
    flywheelPositionSignal( flywheelMotor.GetPosition() ),
    flywheelController( webers_per_rad_t{ constants.kI } ),
    launcherFlywheelQueueReader( launcherFlywheelQueue.CreateReader() ),
    flywheelControlRequest( VoltageOut{ volt_t{ 0 } } ),
    sysIdRoutine(
        SysIdConfig{ ramp_rate_t{ 1_V / 1_s }, 7_V, std::nullopt, nullptr },
        Mechanism( [this]( volt_t voltage ) { this->SetVoltage( voltage ); },
            [this]( SysIdRoutineLog *log ) {
                log->Motor( "Launcher Flywheel" )
                    .voltage( this->GetAppliedVoltage() )
                    .velocity( this->GetVelocity() )
                    .position( this->GetPosition() );
            },
            this,
            "LauncherFlywheel Subsystem" ) ) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem( this );
}

void LauncherFlywheelSubsystem::SubmitControlRequest(
    radians_per_second_t request ) {
    auto currentVelocity = flywheelVelocitySignal.Refresh()
                               .GetValue()
                               .convert<radians_per_second>();
    flywheelControlRequest.WithEnableFOC( true );
    flywheelControlRequest.WithOutput(
        flywheelController.Calculate( currentVelocity ) );
}

void LauncherFlywheelSubsystem::Periodic() {
    flywheelMotor.SetControl( flywheelControlRequest );
}

void LauncherFlywheelSubsystem::SetVoltage( volt_t voltage ) {
    flywheelControlRequest.WithOutput( voltage );
}

volt_t LauncherFlywheelSubsystem::GetAppliedVoltage() {
    return flywheelVoltageSignal.Refresh().GetValue();
}

radians_per_second_t LauncherFlywheelSubsystem::GetVelocity() {
    return flywheelVelocitySignal.Refresh()
        .GetValue()
        .convert<radians_per_second>();
}

radian_t LauncherFlywheelSubsystem::GetPosition() {
    return flywheelPositionSignal.Refresh().GetValue().convert<radian>();
}
