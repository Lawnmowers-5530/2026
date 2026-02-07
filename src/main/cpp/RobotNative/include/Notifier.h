//
// Created by siryellsalot on 2/6/26.
//

#ifndef INC_2026_NOTIFIER_H
#define INC_2026_NOTIFIER_H
#include <frc/Notifier.h>
using frc::Notifier;

#include "Globals.h"

struct NotifierData {

};

void notifierRun(NotifierData*);

class NotiferHandle {
    NotifierData* data;
    Notifier notifier;

public:
    NotiferHandle():
    data(new NotifierData()), notifier(notifierRun, data) {
        
    }
};



#endif //INC_2026_NOTIFIER_H