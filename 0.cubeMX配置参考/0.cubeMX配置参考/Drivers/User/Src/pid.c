#include "pid.h"

static float pid_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

void PID_Init(pid_t *pid, float kp, float ki, float kd, float int_limit, float out_limit)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->int_limit = (int_limit < 0.0f) ? -int_limit : int_limit;
    pid->out_limit = (out_limit < 0.0f) ? -out_limit : out_limit;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->out = 0.0f;
}

float PID_Compute(pid_t *pid, float target, float feedback, float dt_s)
{
    float error;
    float derivative = 0.0f;

    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    error = target - feedback;

    pid->integral += error * dt_s;
    pid->integral = pid_clampf(pid->integral, -pid->int_limit, pid->int_limit);
    derivative = (error - pid->prev_error) / dt_s;

    pid->p_term = pid->kp * error;
    pid->i_term = pid->ki * pid->integral;
    pid->d_term = pid->kd * derivative;
    pid->out = pid->p_term + pid->i_term + pid->d_term;
    pid->out = pid_clampf(pid->out, -pid->out_limit, pid->out_limit);
    pid->prev_error = error;

    return pid->out;
}

void PID_Reset(pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->out = 0.0f;
}

void CurrentPID_Init(current_pid_t *pid,
                     float kp_fwd, float ki_fwd, float kd_fwd,
                     float kp_rev, float ki_rev, float kd_rev,
                     float int_limit, float out_limit)
{
    if (pid == NULL)
    {
        return;
    }

    PID_Init(&pid->forward, kp_fwd, ki_fwd, kd_fwd, int_limit, out_limit);
    PID_Init(&pid->reverse, kp_rev, ki_rev, kd_rev, int_limit, out_limit);
    pid->active_dir = 0;
}

float CurrentPID_Compute(current_pid_t *pid, float target, float feedback, float dt_s)
{
    pid_t *controller;
    int8_t dir;

    if (pid == NULL)
    {
        return 0.0f;
    }

    if (target > 0.0f)
    {
        controller = &pid->forward;
        dir = 1;
    }
    else if (target < 0.0f)
    {
        controller = &pid->reverse;
        dir = -1;
    }
    else
    {
        CurrentPID_Reset(pid);
        return 0.0f;
    }

    if (pid->active_dir != dir)
    {
        PID_Reset(controller);
        pid->active_dir = dir;
    }

    return PID_Compute(controller, target, feedback, dt_s);
}

void CurrentPID_Reset(current_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_Reset(&pid->forward);
    PID_Reset(&pid->reverse);
    pid->active_dir = 0;
}

void CascadePID_Init(cascade_pid_t *pid,
                     float angle_kp, float angle_ki, float angle_kd,
                     float angle_int_limit, float angle_out_limit,
                     float speed_kp, float speed_ki, float speed_kd,
                     float speed_int_limit, float speed_out_limit,
                     float current_kp_fwd, float current_ki_fwd, float current_kd_fwd,
                     float current_kp_rev, float current_ki_rev, float current_kd_rev,
                     float current_int_limit, float current_out_limit)
{
    if (pid == NULL)
    {
        return;
    }

    PID_Init(&pid->angle, angle_kp, angle_ki, angle_kd, angle_int_limit, angle_out_limit);
    PID_Init(&pid->speed, speed_kp, speed_ki, speed_kd, speed_int_limit, speed_out_limit);
    CurrentPID_Init(&pid->current,
                    current_kp_fwd, current_ki_fwd, current_kd_fwd,
                    current_kp_rev, current_ki_rev, current_kd_rev,
                    current_int_limit, current_out_limit);

    pid->angle_target = 0.0f;
    pid->speed_target = 0.0f;
    pid->current_target = 0.0f;
    pid->angle_feedback = 0.0f;
    pid->speed_feedback = 0.0f;
    pid->current_feedback = 0.0f;
    pid->angle_output = 0.0f;
    pid->speed_output = 0.0f;
    pid->current_output = 0.0f;
}

float CascadePID_Compute(cascade_pid_t *pid, pid_chain_mode_t mode,
                         float target,
                         float angle_feedback, float speed_feedback,
                         float current_feedback, float dt_s)
{
    if ((pid == NULL) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    pid->angle_target = 0.0f;
    pid->speed_target = 0.0f;
    pid->current_target = 0.0f;
    pid->angle_feedback = angle_feedback;
    pid->speed_feedback = speed_feedback;
    pid->current_feedback = current_feedback;

    if (mode == PID_CHAIN_ANGLE)
    {
        pid->angle_target = target;
        pid->angle_output = PID_Compute(&pid->angle, pid->angle_target, pid->angle_feedback, dt_s);
        pid->speed_target = pid->angle_output;
        pid->speed_output = PID_Compute(&pid->speed, pid->speed_target, pid->speed_feedback, dt_s);
    }
    else
    {
        pid->speed_target = target;
        pid->angle_output = 0.0f;
        pid->speed_output = PID_Compute(&pid->speed, pid->speed_target, pid->speed_feedback, dt_s);
    }

    pid->current_target = pid->speed_output;
    pid->current_output = CurrentPID_Compute(&pid->current,
                                             pid->current_target,
                                             pid->current_feedback,
                                             dt_s);
    return pid->current_output;
}

void CascadePID_Reset(cascade_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_Reset(&pid->angle);
    PID_Reset(&pid->speed);
    CurrentPID_Reset(&pid->current);
    pid->angle_target = 0.0f;
    pid->speed_target = 0.0f;
    pid->current_target = 0.0f;
    pid->angle_feedback = 0.0f;
    pid->speed_feedback = 0.0f;
    pid->current_feedback = 0.0f;
    pid->angle_output = 0.0f;
    pid->speed_output = 0.0f;
    pid->current_output = 0.0f;
}
