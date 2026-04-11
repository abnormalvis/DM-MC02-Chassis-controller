#ifndef COMM_USB_RX_H
#define COMM_USB_RX_H

#include <stdbool.h>
#include <stdint.h>

void comm_usb_rx_mark_connected(void);
bool comm_usb_rx_submit(const uint8_t *data, uint16_t len);
uint32_t comm_usb_rx_get_last_tick(void);
uint32_t comm_usb_rx_get_drop_count(void);

#endif /* COMM_USB_RX_H */