#ifndef VALVESTATES_H
#define VALVESTATES_H

// defines the possible valve states accross all valves

enum class ValveState
{
    Closed,             // 0
    Open,               // 1
    FireCommanded,      // 2
    OpenCommanded,      // 3
    CloseCommanded,     // 4
    OpenProcess,        // 5
    CloseProcess,       // 6
    BangingOpen,          // 7
    BangingClosed,         // 8
    ValveState_SIZE,    // 9
};

#endif