#include "auto_tune.h"

AutoTune_Typedef tuner;

// ??????????
static void Set_Safe_Baseline(void) {
    Control_DCE_SetKP(50);  // ?????
    Control_DCE_SetKI(0);   // ????,????
    Control_DCE_SetKV(0);   // ??????
    Control_DCE_SetKD(80);  // ???????
}

// ??????
void Motor_AutoTune_Start(void) {
    tuner.state = TUNE_INIT;
    tuner.timer = 0;
    tuner.is_oscillating = false;
    
    // ???????
    Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);
    // ?????
    Motor_Control_Write_Goal_Disable(0);
    Motor_Control_Write_Goal_Brake(0);
}

// ????? (?? main while(1) ??????)
void Motor_AutoTune_Loop(void) {
    // ?????????????,??????
    if(motor_control.state == Control_State_Stall || motor_control.state == Control_State_Overload) {
        tuner.state = TUNE_FAILED;
    }

    switch (tuner.state) {
        case TUNE_IDLE:
        case TUNE_COMPLETE:
        case TUNE_FAILED:
            return; // ?????

        case TUNE_INIT:
            // 1. ?????
            Set_Safe_Baseline();
            // 2. ??????????
            tuner.start_pos = motor_control.real_location;
            // 3. ???????,????
            motor_control.goal_location = tuner.start_pos;
            motor_control.soft_location = tuner.start_pos;
            
            tuner.state = TUNE_MOVE_POSITIVE;
            tuner.timer = 0;
            break;

        // --- ?????? ---
        case TUNE_MOVE_POSITIVE:
            // ????????
            Motor_Control_Write_Goal_Location(tuner.start_pos + TUNE_STEP_AMPLITUDE);
            // ????PID?????,???????“???”????“???”
            // ??XDrive???Tracker????????? Tracker ???????????
            
            tuner.max_error = 0;
            tuner.timer = 0;
            tuner.state = TUNE_WAIT_STABLE_POS;
            break;

        case TUNE_WAIT_STABLE_POS:
            tuner.timer++;
            
            // ?????????????
            if (abs(dce.p_error) > tuner.max_error) {
                tuner.max_error = abs(dce.p_error);
            }

            // ???????:???????0?,?????????
            if (tuner.timer > (TUNE_WAIT_CYCLES / 2)) {
                // ??????????
                if (abs(dce.p_error) > 50 && abs(dce.v_error) > 50) { 
                    tuner.is_oscillating = true;
                }
            }

            if (tuner.timer >= TUNE_WAIT_CYCLES) {
                tuner.state = TUNE_MOVE_NEGATIVE; // ???:???
            }
            break;

        // --- ?????? ---
        case TUNE_MOVE_NEGATIVE:
            Motor_Control_Write_Goal_Location(tuner.start_pos); // ????
            tuner.timer = 0;
            tuner.state = TUNE_WAIT_STABLE_NEG;
            break;

        case TUNE_WAIT_STABLE_NEG:
            tuner.timer++;
            // ??????(??)
            if (tuner.timer >= TUNE_WAIT_CYCLES) {
                tuner.state = TUNE_EVALUATE; // ????,????
            }
            break;

        // --- ??:??????? ---
        case TUNE_EVALUATE:
            // ??:
            // 1. ???? -> ?? KD ? ?? KP
            // 2. ?????????(????)? -> ?? KP
            // 3. ???? -> ??

            if (tuner.is_oscillating) {
                // ????,?????? KD ??
                if (dce.kd < TUNE_MAX_KD) {
                    uint16_t new_kd = dce.kd + 20;
                    Control_DCE_SetKD(new_kd);
                    // ??????,????
                    tuner.is_oscillating = false;
                    tuner.state = TUNE_MOVE_POSITIVE; // ??
                } else {
                    // KD ????????,?? KP ???,????
                    uint16_t safe_kp = (dce.kp > 20) ? (dce.kp - 20) : dce.kp;
                    Control_DCE_SetKP(safe_kp);
                    tuner.state = TUNE_COMPLETE; // ???????,??
                }
            } else {
                // ????
                if (dce.kp < TUNE_MAX_KP) {
                    // ???????,??????“?”??
                    // ??????? KP
                    tuner.last_kp = dce.kp; 
                    
                    // ?? KP
                    Control_DCE_SetKP(dce.kp + 20);
                    
                    // ????????? KD,?????
                    // ????:KD ?????? sqrt(KP) ??,?????????
                    if (dce.kd < TUNE_MAX_KD) {
                        Control_DCE_SetKD(dce.kd + 10);
                    }
                    
                    tuner.state = TUNE_MOVE_POSITIVE; // ?????????
                } else {
                    // ??????
                    tuner.state = TUNE_COMPLETE;
                }
            }
            break;
    }
}