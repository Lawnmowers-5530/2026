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

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* pjvm, void* reserved) {
    static auto jvm{std::make_unique<jni::JvmRef<jni::kDefaultJvm>>(pjvm)};
    return JNI_VERSION_10;
}

JNIEXPORT jlong JNICALL
Java_frc_robot_subsystems_RobotNative_00024Native_initialize(
    JNIEnv* env,
    jclass _class,
    jobject initInfoObj
) {
    {
    }
}

