//
// Created by siryellsalot on 1/26/26.
//
#include "RobotNative.h"
#include "jni/frc_robot_subsystems_RobotNative_Native.h"
#include "Globals.h"
#include "Notifier.h"

#include <frc/Notifier.h>
#include <jni/jni.hpp>
#include <jni/class.hpp>

using frc::Notifier;

struct Handle {
    Notifier notifier;
    NotifierData *notifierData;
};

class InitInfoJNI {
public:
    static constexpr auto Name() { return "frc/robot/subsystems/RobotNative$InitInfo"; }

    class Fields {
        Fields(JNIEnv& env) :
            self(jni::Class<InitInfoJNI>::Singleton(env)),
            launcherConstantsField(jni::Class<InitInfoJNI>::Singleton(env).GetStaticField<jni::Object<jni::ClassTag>>(env, "launcherConstants")) {};

    public:
        const jni::Class<InitInfoJNI>& self;
        jni::StaticField<InitInfoJNI, jni::Object<jni::ClassTag>> launcherConstantsField;

        static Fields& getInstance(JNIEnv& env) {
            static Fields instance(env);
            return instance;
        }

        static auto getLauncherConstants(JNIEnv& env) {
            auto instance = getInstance(env);
            return instance.self.Get(env, instance.launcherConstantsField);
        }
    };
};

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *jvm, void *reserved) {
    return JNI_VERSION_10;
}

JNIEXPORT jlong JNICALL
Java_frc_robot_subsystems_RobotNative_00024Native_initialize(
    JNIEnv *_env,
    jclass _class,
    jni::jclass &initInfoObj
) {
    auto& env = *_env;
    {
        auto launcherConstantsObj = InitInfoJNI::Fields::getLauncherConstants(env);
        auto& launcherConstantsClass = jni::GetObjectClass(env, *launcherConstantsObj);
        launcherConstants.init(env, launcherConstantsClass, *launcherConstantsObj.get()); // declared in notifier.h
    }

}

