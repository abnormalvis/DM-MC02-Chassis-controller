#ifndef TEST_STUB_COMM_DISPATCHER_H
#define TEST_STUB_COMM_DISPATCHER_H

void comm_dispatcher_init_queues(void);
void CommDispatcherTask(void *argument);
void comm_dispatcher_on_ctrl_cmd(const void *cmd);

#endif
