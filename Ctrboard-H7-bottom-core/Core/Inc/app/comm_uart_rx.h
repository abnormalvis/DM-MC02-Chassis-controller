#ifndef COMM_UART_RX_H
#define COMM_UART_RX_H

#include <stdint.h>

#include "comm_dispatcher.h"

void comm_uart_rx_init(void);
uint32_t comm_uart_rx_get_last_tick(comm_link_t link);
uint32_t comm_uart_rx_get_drop_count(comm_link_t link);

#endif /* COMM_UART_RX_H */