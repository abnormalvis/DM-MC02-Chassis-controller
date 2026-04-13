#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_tasks.h"
#include "cmsis_os2.h"
#include "comm_dispatcher.h"
#include "protocol_adapter.h"

static uint32_t g_fake_tick = 0U;
static int16_t g_current_a = 0;
static int16_t g_current_b = 0;

static uint32_t g_comm_init_calls = 0U;
static uint32_t g_can_init_calls = 0U;
static uint32_t g_safety_estop_calls = 0U;
static uint32_t g_motor_target_calls = 0U;
static int16_t g_last_target_a = 0;
static int16_t g_last_target_b = 0;

typedef struct
{
    const char *name;
    osPriority_t priority;
    uint32_t stack_size;
    osThreadFunc_t func;
} thread_record_t;

static thread_record_t g_threads[16];
static uint32_t g_thread_count = 0U;

static void reset_fakes(void)
{
    g_fake_tick = 0U;
    g_current_a = 0;
    g_current_b = 0;
    g_comm_init_calls = 0U;
    g_can_init_calls = 0U;
    g_safety_estop_calls = 0U;
    g_motor_target_calls = 0U;
    g_last_target_a = 0;
    g_last_target_b = 0;
    g_thread_count = 0U;
    memset(g_threads, 0, sizeof(g_threads));
}

uint32_t HAL_GetTick(void)
{
    return g_fake_tick;
}

osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr)
{
    (void)argument;

    assert(g_thread_count < (uint32_t)(sizeof(g_threads) / sizeof(g_threads[0])));
    g_threads[g_thread_count].name = attr != NULL ? attr->name : NULL;
    g_threads[g_thread_count].priority = attr != NULL ? attr->priority : 0;
    g_threads[g_thread_count].stack_size = attr != NULL ? attr->stack_size : 0U;
    g_threads[g_thread_count].func = func;
    g_thread_count++;

    return (osThreadId_t)(uintptr_t)g_thread_count;
}

void osDelay(uint32_t ms)
{
    (void)ms;
}

void comm_dispatcher_init_queues(void)
{
    g_comm_init_calls++;
}

void CommDispatcherTask(void *argument)
{
    (void)argument;
}

void CANTask_Init(void)
{
    g_can_init_calls++;
}

void CANTask_Process(void *argument)
{
    (void)argument;
}

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

void MotorControlTask_Init(void) {}
void MotorControlTask_Process(void *argument) { (void)argument; }
void MotorControlTask_SetTarget(int16_t rpm_a, int16_t rpm_b)
{
    g_last_target_a = rpm_a;
    g_last_target_b = rpm_b;
    g_motor_target_calls++;
}
void CRSFTask_Init(void) {}
void CRSFTask_Process(void *argument) { (void)argument; }
void USBTask_Init(void) {}
void USBTask_Process(void *argument) { (void)argument; }
void RS485Task_Init(void) {}
void RS485Task_Process(void *argument) { (void)argument; }
void IMUTask_Init(void) {}
void IMUTask_Process(void *argument) { (void)argument; }
void SafetyTask_Init(void) {}
void SafetyTask_Process(void *argument) { (void)argument; }
void LEDTask_Init(void) {}
void LEDTask_Process(void *argument) { (void)argument; }
void BuzzerTask_Init(void) {}
void BuzzerTask_Process(void *argument) { (void)argument; }

void SafetyTask_TriggerEStop(void)
{
    g_safety_estop_calls++;
}

static int has_thread_named(const char *name)
{
    uint32_t i;
    for (i = 0U; i < g_thread_count; ++i)
    {
        if (g_threads[i].name != NULL && strcmp(g_threads[i].name, name) == 0)
        {
            return 1;
        }
    }
    return 0;
}

static void test_app_tasks_create_layout(void)
{
    reset_fakes();

    AppTasks_Create();

    assert(g_comm_init_calls == 1U);
    assert(g_can_init_calls == 1U);
    assert(g_thread_count == 10U);

    assert(has_thread_named("CommDispatch"));
    assert(has_thread_named("MotorControl"));
    assert(has_thread_named("CRSFTask"));
    assert(has_thread_named("CANTask"));
    assert(has_thread_named("USBTask"));
    assert(has_thread_named("RS485Task"));
    assert(has_thread_named("IMUTask"));
    assert(has_thread_named("SafetyTask"));
    assert(has_thread_named("LEDTask"));
    assert(has_thread_named("BuzzerTask"));
}

static void test_control_mode_fault_and_estop_path(void)
{
    system_state_t state;

    reset_fakes();
    AppTasks_Create();

    g_fake_tick = 123U;
    AppTasks_SetControlMode(MODE_OFFBOARD);
    AppTasks_GetSystemState(&state);
    assert(state.mode == MODE_OFFBOARD);
    assert(state.last_update_tick == 123U);

    g_fake_tick = 200U;
    AppTasks_SetFault(0x04U);
    AppTasks_GetSystemState(&state);
    assert((state.fault_bits & 0x04U) != 0U);
    assert(state.last_update_tick == 200U);

    g_fake_tick = 220U;
    AppTasks_ClearFault(0x04U);
    AppTasks_GetSystemState(&state);
    assert((state.fault_bits & 0x04U) == 0U);
    assert(state.last_update_tick == 220U);

    g_fake_tick = 300U;
    AppTasks_TriggerEStop();
    AppTasks_GetSystemState(&state);
    assert(state.mode == MODE_BRAKE_PROTECT);
    assert(state.last_update_tick == 300U);
    assert(g_safety_estop_calls == 1U);
}

static void test_motor_state_bridge(void)
{
    motor_state_t mstate;

    reset_fakes();
    AppTasks_Create();

    g_current_a = 111;
    g_current_b = -222;

    AppTasks_SetControlMode(MODE_IDLE);
    AppTasks_GetMotorState(&mstate);
    assert(mstate.current_a == 111);
    assert(mstate.current_b == -222);
    assert(mstate.brake_active == 0U);

    AppTasks_TriggerEStop();
    AppTasks_GetMotorState(&mstate);
    assert(mstate.brake_active == 1U);
}

static void test_dispatcher_control_linkage(void)
{
    chassis_ctrl_cmd_t cmd;
    chassis_cmd_t *offboard;
    system_state_t state;

    reset_fakes();
    AppTasks_Create();

    memset(&cmd, 0, sizeof(cmd));
    cmd.vx_mps = 1.0f;
    cmd.wz_rps = 0.0f;
    cmd.mode = (uint8_t)CTRL_MODE_OFFBOARD;
    cmd.source = (uint8_t)OFFBOARD_SRC_USB;
    cmd.ts_ms = 456U;
    cmd.valid = 1U;
    cmd.estop = 0U;

    g_fake_tick = 500U;
    comm_dispatcher_on_ctrl_cmd(&cmd);

    AppTasks_GetSystemState(&state);
    assert(state.mode == MODE_OFFBOARD);
    assert(state.usb_connected == 1U);
    assert(state.last_update_tick == 500U);

    offboard = AppTasks_GetOffboardCommand();
    assert(offboard->vx_mps == 1.0f);
    assert(offboard->wz_rps == 0.0f);
    assert(offboard->timestamp == 456U);
    assert(offboard->valid == 1U);

    assert(g_motor_target_calls == 1U);
    assert(g_last_target_a != 0 || g_last_target_b != 0);
}

static void test_dispatcher_estop_path(void)
{
    chassis_ctrl_cmd_t cmd;
    system_state_t state;

    reset_fakes();
    AppTasks_Create();

    memset(&cmd, 0, sizeof(cmd));
    cmd.vx_mps = 1.0f;
    cmd.wz_rps = 0.0f;
    cmd.mode = (uint8_t)CTRL_MODE_OFFBOARD;
    cmd.source = (uint8_t)OFFBOARD_SRC_USB;
    cmd.ts_ms = 789U;
    cmd.valid = 1U;
    cmd.estop = 1U;

    g_fake_tick = 600U;
    comm_dispatcher_on_ctrl_cmd(&cmd);

    assert(g_safety_estop_calls == 1U);
    AppTasks_GetSystemState(&state);
    assert(state.mode == MODE_BRAKE_PROTECT);
}

int main(void)
{
    test_app_tasks_create_layout();
    test_control_mode_fault_and_estop_path();
    test_motor_state_bridge();
    test_dispatcher_control_linkage();
    test_dispatcher_estop_path();

    puts("app_tasks_chain_test: PASS");
    return 0;
}
