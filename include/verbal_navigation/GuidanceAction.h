#ifndef ACTION_H
#define ACTION_H

enum GuidanceActions { LEAD = 'L', INSTRUCT = 'I', TRANSITION = 'T'};

class GuidanceAction {
    virtual void perform() = 0;
};

#endif