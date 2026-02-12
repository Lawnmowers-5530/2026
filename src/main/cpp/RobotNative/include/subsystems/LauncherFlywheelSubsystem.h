//
// Created by siryellsalot on 2/11/26.
//

#ifndef INC_2026_LAUNCHERFLYWHEEL_H
#define INC_2026_LAUNCHERFLYWHEEL_H
#include <ctre/phoenix6/TalonFX.hpp>
namespace hardware = ctre::phoenix6::hardware;
using hardware::TalonFX;
#include <ctre/phoenix6/StatusSignal.hpp>
using ctre::phoenix6::StatusSignal;
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
using ctre::phoenix6::controls::TorqueCurrentFOC;

#include <units/angular_velocity.h>
using namespace units;

#include "TBHController.h"
#include "Sync.h"
#include "Globals.h"

#include <frc2/command/Subsystem.h>
using frc2::Subsystem;

#include <frc2/command/sysid/SysIdRoutine.h>
using SysIdConfig = frc2::sysid::Config;
using frc2::sysid::SysIdRoutine;

struct LauncherFlywheelSubsystem: Subsystem {
    TalonFX flywheelMotor;
    StatusSignal<turns_per_second_t> flywheelVelocitySignal;
    TBHController flywheelController;
    SPSCQueue<double>::SPSCReader launcherFlywheelQueueReader;
    TorqueCurrentFOC flywheelControlRequest;
    SysIdRoutine sysIdRoutine;

    LauncherFlywheelSubsystem(const LauncherConstants& constants, SPSCQueue<double>& launcherFlywheelQueue);

    void SubmitControlRequest(radians_per_second_t request);

    void Periodic() override;
};

#endif //INC_2026_LAUNCHERFLYWHEEL_H