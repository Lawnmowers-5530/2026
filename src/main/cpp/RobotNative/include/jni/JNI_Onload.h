//
// Created by siryellsalot on 2/9/26.
//

#ifndef INC_2026_JNI_ONLOAD_H
#define INC_2026_JNI_ONLOAD_H

#include <jni.h>
extern "C" {
    JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*);
};

#endif //INC_2026_JNI_ONLOAD_H