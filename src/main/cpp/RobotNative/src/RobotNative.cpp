//
// Created by siryellsalot on 1/26/26.
//

#include "RobotNative.h"
#include "jni/frc_robot_subsystems_RobotNative_Native.h"
#include <third-party/jni-bind/jni_bind_release.h>

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


static constexpr jni::Class Class {
    "java/lang/Class",
    jni::Method { "getName", jni::Return<jstring>{} }
};

static constexpr jni::Class InitInfoClass {
    "frc/robot/subsystems/RobotNative$InitInfo",
    jni::Field { "launcherConstants", Class}
};

JNIEXPORT jlong JNICALL
Java_frc_robot_subsystems_RobotNative_00024Native_initialize(
    JNIEnv* env,
    jclass _class,
    jobject initInfoObj
) {
    {
        //jni::LocalObject<InitInfoClass> initInfo { initInfoObj };
        //auto launcherConstantsField = initInfo.Access<"launcherConstants">().Get();
    }
}

