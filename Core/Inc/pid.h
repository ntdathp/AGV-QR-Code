
#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

#define SAMPLING_TIME 0.01
#define MINUTE 60
#define SECOND 1

// control PID Structure
typedef struct
{
    float dKp, dKi, dKd;
    float dErrorTerm;
    float dIntergral;
    float result;
} PID_CONTROL_t;

extern PID_CONTROL_t tPID_1, tPID_2;

extern void PIDReset(PID_CONTROL_t *PID_Ctrl);
extern void PIDInit(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd);
extern void PIDTunningSet(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd);
extern float PIDCompute(PID_CONTROL_t *PID_Ctrl, float dCmdValue, float dActValue, float dTs);

#endif /* INC_PID_H_ */
