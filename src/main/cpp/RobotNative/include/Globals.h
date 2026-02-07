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

    LauncherConstants(JNIEnv &env, jni::jclass &cls, jni::jobject &launcherConstantsObj) {
        auto &canIdFieldID = jni::GetFieldID(env, cls, "canId", "I");
        auto &kFFieldID = jni::GetFieldID(env, cls, "kF", "D");
        auto &kIFieldID = jni::GetFieldID(env, cls, "kI", "D");

        this->canId = jni::GetField<int>(env, &launcherConstantsObj, canIdFieldID);
        this->kF = jni::GetField<double>(env, &launcherConstantsObj, kFFieldID);
        this->kI = jni::GetField<double>(env, &launcherConstantsObj, kIFieldID);
    }
};


struct Constants {
    LauncherConstants launcherConstants;
};
#endif //INC_2026_GLOBALS_H