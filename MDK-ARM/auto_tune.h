#ifndef AUTO_TUNE_H
#define AUTO_TUNE_H

#include "motor_control.h"
#include <stdlib.h> // for abs()

// ????
#define TUNE_STEP_AMPLITUDE   1000    // ?????????? (?1/50?)
#define TUNE_WAIT_CYCLES      500     // ???????????(??1ms????,?500ms)
#define TUNE_MAX_KP           300     // ???? KP
#define TUNE_MAX_KD           600     // ???? KD

typedef enum {
    TUNE_IDLE = 0,
    TUNE_INIT,
    TUNE_MOVE_POSITIVE,
    TUNE_WAIT_STABLE_POS,
    TUNE_MOVE_NEGATIVE,
    TUNE_WAIT_STABLE_NEG,
    TUNE_EVALUATE,
    TUNE_COMPLETE,
    TUNE_FAILED
} Tune_State;

typedef struct {
    Tune_State state;
    int32_t timer;
    int32_t start_pos;
    int32_t max_error;      // ????????
    int32_t overshoot;      // ?????
    int32_t last_kp;        // ??????KP
    bool    is_oscillating; // ????
} AutoTune_Typedef;

void Motor_AutoTune_Start(void);
void Motor_AutoTune_Loop(void); // ?????????? (?? 1ms ? 5ms)

#endif