//
// Created by siryellsalot on 2/6/26.
//

#ifndef INC_2026_GLOBALS_H
#define INC_2026_GLOBALS_H
#include <jni/jni.hpp>

static struct LauncherConstants {
    int canId;
    double kF;
    double kI;

    void init(JNIEnv &env, jni::jclass &cls, jni::jobject &launcherConstantsObj) {
        auto &canIdFieldID = jni::GetFieldID(env, cls, "canId", "I");
        auto &kFFieldID = jni::GetFieldID(env, cls, "kF", "D");
        auto &kIFieldID = jni::GetFieldID(env, cls, "kI", "D");

        canId = jni::GetField<int>(env, &launcherConstantsObj, canIdFieldID);
        kF = jni::GetField<double>(env, &launcherConstantsObj, kFFieldID);
        kI = jni::GetField<double>(env, &launcherConstantsObj, kIFieldID);
    }
} launcherConstants;

#endif //INC_2026_GLOBALS_H