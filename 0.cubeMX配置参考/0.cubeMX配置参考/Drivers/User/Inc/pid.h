#ifndef PID_H
#define PID_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Generic position PID  (angle loop, speed loop)
 * ----------------------------------------------------------------------- */
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float int_limit;   /* integral absolute clamp */
    float out_limit;   /* output absolute clamp   */

    /* diagnostic readback — safe to log via VOFA */
    float p_term;
    float i_term;
    float d_term;
    float out;
} pid_t;

void  PID_Init(pid_t *pid, float kp, float ki, float kd,
               float int_limit, float out_limit);

/* Returns output.  dt_s must be > 0. */
float PID_Compute(pid_t *pid, float target, float feedback, float dt_s);

void  PID_Reset(pid_t *pid);

typedef struct {
    pid_t forward;
    pid_t reverse;
    int8_t active_dir;
} current_pid_t;

void  CurrentPID_Init(current_pid_t *pid,
                      float kp_fwd, float ki_fwd, float kd_fwd,
                      float kp_rev, float ki_rev, float kd_rev,
                      float int_limit, float out_limit);

float CurrentPID_Compute(current_pid_t *pid,
                         float target, float feedback, float dt_s);

void  CurrentPID_Reset(current_pid_t *pid);

typedef enum {
    PID_CHAIN_SPEED = 0,
    PID_CHAIN_ANGLE = 1
} pid_chain_mode_t;

typedef struct {
    pid_t angle;
    pid_t speed;
    current_pid_t current;

    float angle_target;
    float speed_target;
    float current_target;

    float angle_feedback;
    float speed_feedback;
    float current_feedback;

    float angle_output;
    float speed_output;
    float current_output;
} cascade_pid_t;

void  CascadePID_Init(cascade_pid_t *pid,
                      float angle_kp, float angle_ki, float angle_kd,
                      float angle_int_limit, float angle_out_limit,
                      float speed_kp, float speed_ki, float speed_kd,
                      float speed_int_limit, float speed_out_limit,
                      float current_kp_fwd, float current_ki_fwd, float current_kd_fwd,
                      float current_kp_rev, float current_ki_rev, float current_kd_rev,
                      float current_int_limit, float current_out_limit);

float CascadePID_Compute(cascade_pid_t *pid, pid_chain_mode_t mode,
                         float target,
                         float angle_feedback, float speed_feedback,
                         float current_feedback, float dt_s);

void  CascadePID_Reset(cascade_pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
