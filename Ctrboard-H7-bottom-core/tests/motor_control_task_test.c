#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "motor_control_task.h"
#include "stubs/tim.h"

TIM_HandleTypeDef htim1 = { .autoreload = 1000U, .ccr1 = 0U, .ccr3 = 0U };
TIM_HandleTypeDef htim2 = { .autoreload = 1000U, .ccr1 = 0U, .ccr3 = 0U };

static int16_t g_current_a = 0;
static int16_t g_current_b = 0;

void CANTask_GetCurrent(int16_t *current_a, int16_t *current_b)
{
    if (current_a != NULL)
    {
        *current_a = g_current_a;
    }
    if (current_b != NULL)
    {
        *current_b = g_current_b;
    }
}

static void test_motor_direction_and_state_update(void)
{
    motor_control_state_t state;

    MotorControlTask_Init();
    MotorControlTask_ReleaseBrake();
    MotorControlTask_SetPIDMode(PID_MODE_DISABLED);

    MotorControlTask_SetTarget(500, -250);
    g_current_a = 123;
    g_current_b = -321;
    MotorControlTask_Process(NULL);

    assert(htim1.ccr1 == 18U);
    assert(htim1.ccr3 == 0U);
    assert(htim2.ccr1 == 0U);
    assert(htim2.ccr3 == 18U);

    MotorControlTask_GetState(&state);
    assert(state.current_a == 123);
    assert(state.current_b == -321);
    assert(state.target_rpm_a == 500);
    assert(state.target_rpm_b == -250);
}

static void test_brake_and_release(void)
{
    motor_control_state_t state;

    MotorControlTask_Init();
    MotorControlTask_SetPIDMode(PID_MODE_DISABLED);
    MotorControlTask_SetTarget(600, 600);
    MotorControlTask_Process(NULL);
    assert(htim1.ccr1 > 0U);

    MotorControlTask_Brake();
    assert(htim1.ccr1 == 0U);
    assert(htim1.ccr3 == 0U);
    assert(htim2.ccr1 == 0U);
    assert(htim2.ccr3 == 0U);

    MotorControlTask_Process(NULL);
    MotorControlTask_GetState(&state);
    assert(state.brake_active);
    assert(state.target_rpm_a == 0);
    assert(state.target_rpm_b == 0);

    MotorControlTask_ReleaseBrake();
    MotorControlTask_SetTarget(200, 0);
    MotorControlTask_Process(NULL);
    assert(htim1.ccr1 > 0U);
}

static void test_max_duty_limit(void)
{
    int i;

    MotorControlTask_Init();
    MotorControlTask_ReleaseBrake();
    MotorControlTask_SetPIDMode(PID_MODE_DISABLED);
    MotorControlTask_SetMaxDuty(0.5f);
    MotorControlTask_SetTarget(1000, 1000);

    for (i = 0; i < 80; ++i)
    {
        MotorControlTask_Process(NULL);
    }

    assert(htim1.ccr1 <= 500U);
    assert(htim2.ccr1 <= 500U);
}

int main(void)
{
    test_motor_direction_and_state_update();
    test_brake_and_release();
    test_max_duty_limit();

    puts("motor_control_task_test: PASS");
    return 0;
}
