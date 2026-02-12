//
// Created by siryellsalot on 2/6/26.
//

#ifndef INC_2026_NOTIFIER_H
#define INC_2026_NOTIFIER_H

#include <frc/Notifier.h>
using frc::Notifier;

#include <ctre/phoenix6/TalonFX.hpp>
namespace hardware = ctre::phoenix6::hardware;
using hardware::TalonFX;
#include <ctre/phoenix6/StatusSignal.hpp>
using ctre::phoenix6::StatusSignal;

#include "Sync.h"
#include "Globals.h"
#include "TBHController.h"

#include "subsystems/LauncherFlywheelSubsystem.h"

#include <units/frequency.h>
using units::frequency::hertz_t;

#include <frc2/command/CommandScheduler.h>
using frc2::CommandScheduler;

#include <frc2/command/button/CommandXboxController.h>
using frc2::CommandXboxController;

struct NotifierData {
    LauncherFlywheelSubsystem launcherFlywheelSubsystem;
    CommandScheduler& scheduler;
    CommandXboxController controller;

    NotifierData(const Constants& constants, SPSCQueue<double>& launcherFlywheelQueue);
};

void NotifierRun(const NotifierData*);

struct NotifierHandle {
    SPSCQueue<double> launcherFlywheelQueue;
    NotifierData* data;
    Notifier notifier;
    hertz_t periodMs;

    NotifierHandle(const Constants& constants, hertz_t frequency);

    void submitLauncherControlRequest(double controlRequest);
    void startNotifier();
    void stopNotifier();
};

#endif //INC_2026_NOTIFIER_H