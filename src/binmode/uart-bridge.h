#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

extern const char uart_bridge_name[];
void uart_bridge_setup(void);
void uart_bridge_setup_message(void);
void uart_bridge_service(void);
void uart_bridge_cleanup(void);

#endif
