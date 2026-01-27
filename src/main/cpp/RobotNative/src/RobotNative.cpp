//
// Created by siryellsalot on 1/26/26.
//

#include "RobotNative.h"
#include "jni/frc_robot_subsystems_RobotNative_Native.h"

#include <frc/Notifier.h>

using frc::Notifier;

struct NotifierData {

};

struct Handle {
    Notifier notifier;
    NotifierData* notifierData;
};



JNIEXPORT jlong JNICALL
Java_frc_robot_subsystems_RobotNative_00024Native_initialize(
    JNIEnv* env,
    jclass _class,
    jobject _this
) {


    {
        jclass launcherConstantsClass = env->FindClass("frc/robot/constants/LauncherConstants");
        if (launcherConstantsClass == nullptr) {
            if (env->ExceptionCheck()) {
                env->ExceptionDescribe();
                env->ExceptionClear();
            }
            return reinterpret_cast<jlong>(nullptr);
        }
    }

}

