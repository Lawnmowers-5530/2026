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

#include "Sync.h"

#include "Globals.h"

#include <units/frequency.h>
using units::frequency::hertz_t;

struct NotifierData {
    struct LauncherData {
        TalonFX flywheelMotor;
        SPSCQueue<double>::SPSCReader launcherFlywheelQueueReader;
    } launcherData;
    NotifierData(const Constants& constants, SPSCQueue<double>& launcherFlywheelQueue):
        launcherData(
            TalonFX(constants.launcherConstants.canId),
            launcherFlywheelQueue.CreateReader()
        )
    {}
};

void NotifierRun(NotifierData*);

struct NotifierHandle {
    SPSCQueue<double> launcherFlywheelQueue;
    NotifierData* data;
    Notifier notifier;
    hertz_t periodMs;

public:
    NotifierHandle(const Constants& constants, hertz_t frequency):
        launcherFlywheelQueue(SPSCQueue<double>()),
        data(new NotifierData(constants, launcherFlywheelQueue)),
        notifier(NotifierRun, data),
        periodMs(frequency) {

    }

    void submitLauncherControlRequest(double controlRequest) {
        launcherFlywheelQueue.write(controlRequest);
    }

    void startNotifier();
    void stopNotifier();
};



#endif //INC_2026_NOTIFIER_H