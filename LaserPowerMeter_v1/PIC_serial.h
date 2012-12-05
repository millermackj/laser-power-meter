/* PIC_serial.h
 Serial UART support for dsPIC33
 */

#ifndef _PIC_SERIAL_H
#define _PIC_SERIAL_H

// oscillator and instruction cycle frequencies for buadrate generator
#define OSC_FREQ 79227500                 // 79.2275 Mhz
#define FCY (OSC_FREQ / 2)

#define UART_ENABLE U1MODEbits.UARTEN // enable bit for UART1

#define TX_REG U1TXREG          // transmit buffer for UART1
#define TX_FULL U1STAbits.UTXBF // transmit buffer flag is up if buffer's full
#define TX_EMPTY U1STAbits.TRMT // Transmit Shift Register Empty bit

#define RX_REG  U1RXREG             // receive buffer for UART1
#define RX_AVAIL U1STAbits.URXDA    // receive buffer data available bit
#define RX_INTFLAG IFS0bits.U1RXIF  // receive interrupt flag
#define TX_INTFLAG IFS0bits.U1TXIF  // transmit interrupt flag
#define RX_LOCK IEC0bits.U1RXIE     // enable bit for recieve interrupts
#define TX_LOCK IEC0bits.U1TXIE     // enable bit for transmit interrupts

/* interrupt declaration for faster context switching
 * if the interrupt doesn't access any const variables*/

void serial_begin(unsigned long int baud);
void serialTest(void);
void serial_write(const char * buffer, int size);
int serial_available(void); // returns number of unread input buffer
int serial_remaining(void); // returns length of unsent character buffer
void serial_read(char* buffer, int size);
void serial_bufWrite(const char *buffer, int size);
void serial_ping(void);
int serial_gotNewline(void);

#endif
