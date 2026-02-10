//
// Created by siryellsalot on 2/6/26.
//

#ifndef INC_2026_GLOBALS_H
#define INC_2026_GLOBALS_H
#include <jni/jni.hpp>

struct LauncherConstants {
    int canId;
    double kF;
    double kI;

    LauncherConstants(JNIEnv &env, jni::jclass &launcherConstantsObj) {
        auto &canIdFieldID = jni::GetStaticFieldID(env, launcherConstantsObj, "canId", "I");
        auto &kFFieldID = jni::GetStaticFieldID(env, launcherConstantsObj, "kI", "D");
        auto &kIFieldID = jni::GetStaticFieldID(env, launcherConstantsObj, "kA", "D");

        this->canId = jni::GetStaticField<int>(env, launcherConstantsObj, canIdFieldID);
        this->kI = jni::GetStaticField<double>(env, launcherConstantsObj,kIFieldID);
        this->kF = jni::GetStaticField<double>(env, launcherConstantsObj, kFFieldID);
    }
};


struct Constants {
    LauncherConstants launcherConstants;
};
#endif //INC_2026_GLOBALS_H