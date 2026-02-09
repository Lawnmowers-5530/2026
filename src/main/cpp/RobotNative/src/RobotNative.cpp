//
// Created by siryellsalot on 1/26/26.
//
#include <jni.h>

#include "RobotNative.h"
#include "jni/JNI_Onload.h"
#include "Globals.h"
#include "Notifier.h"
#include "Sync.h"


#include <frc/Notifier.h>
#include <jni/jni.hpp>
#include <jni/class.hpp>
#include <memory>

class ConstantsJNI {
public:
    static constexpr auto Name() { return "frc/robot/subsystems/RobotNative$InitInfo"; }

    class Fields {
        Fields(JNIEnv& env, jni::Object<ConstantsJNI>& initInfoObj):
            self(jni::Class<ConstantsJNI>::Singleton(env)),
            launcherConstantsObj(initInfoObj),
            launcherConstantsField(self.GetField<jni::Object<jni::ClassTag>>(env, "launcherConstants")) {};

    public:
        const jni::Class<ConstantsJNI>& self;
        jni::Object<ConstantsJNI>& launcherConstantsObj;
        jni::Field<ConstantsJNI, jni::Object<jni::ClassTag>> launcherConstantsField;

        static Fields& getInstance(JNIEnv& env, jni::Object<ConstantsJNI>& initInfoObj) {
            static Fields instance(env, initInfoObj);
            return instance;
        }

        static auto getLauncherConstants(JNIEnv& env, jni::Object<ConstantsJNI>& initInfoObj) {
            auto instance = getInstance(env, initInfoObj);
            return initInfoObj.Get(env, instance.launcherConstantsField);
        }
    };
};

JNICALL jlong Initialize (
    JNIEnv* _env,
    jni::jclass*,
    jni::jobject* _constantsObj
) {
    auto& env = *_env;
    auto& cls = jni::Class<ConstantsJNI>::Singleton(env);
    auto constantsObj = jni::Object<ConstantsJNI>(_constantsObj);

    auto launcherConstantsObj = ConstantsJNI::Fields::getLauncherConstants(env, constantsObj);
    auto& launcherConstantsClass = jni::GetObjectClass(env, *launcherConstantsObj);
    Constants constants = {
         LauncherConstants{env, launcherConstantsClass, *launcherConstantsObj}
    };
    auto handle = std::make_unique<NotifierHandle>(constants, 200_Hz);

    return reinterpret_cast<jlong>(handle.release());
}

JNICALL void Destroy (JNIEnv* _env, jni::jclass*, jlong _handle) {
    auto* handle = reinterpret_cast<NotifierHandle*>(_handle);
    delete handle;
}

JNICALL void StartNotifier (JNIEnv* _env, jni::jclass*) {

}

JNICALL void StopNotifier (JNIEnv* _env, jni::jclass*) {

}

JNICALL void SubmitLauncherControlRequest (JNIEnv* _env, jni::jclass*, jlong _handle, jdouble rpm) {
    auto* handle = reinterpret_cast<NotifierHandle*>(_handle);
    handle->submitLauncherControlRequest(rpm);
}

void RegisterNatives(JNIEnv& env) {
    jni::RegisterNatives(
        env,
        jni::FindClass(env, "frc/Robot/subsystems/RobotNative$Native"),
        jni::MakeNativeMethod<decltype(&Initialize), &Initialize> ("initialize", "(Lfrc/robot/subsystems/RobotNative/InitInfo;)J"),
        jni::MakeNativeMethod<decltype(&Destroy), &Destroy> ("destroy", "(J)V"),
        jni::MakeNativeMethod<decltype(&StartNotifier), &StartNotifier> ("startNotifier", "()V"),
        jni::MakeNativeMethod<decltype(&StopNotifier), &StopNotifier> ("stopNotifier", "()V")
    );
}

extern "C" {
    JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* jvm, void*) {
        JNIEnv& env = jni::GetEnv(*jvm);
        RegisterNatives(env );
        return JNI_VERSION_10;
    }
};


