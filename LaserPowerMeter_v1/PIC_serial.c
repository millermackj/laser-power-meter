#include "PIC_serial.h"
#include <p33FJ64MC202.h>
#include "support.h"

char inputBuffer[BUFFER_SIZE];  // create user input buffer
int inputHead;                  // index to head of input buffer
int inputTail;                  // index to tail of input buffer

char outputBuffer[BUFFER_SIZE];  // create user output buffer
int outputHead;                  // index to head of output buffer
int outputTail;                  // index to tail of output buffer
char newByte;

volatile int bytesRemaining;    // number of unsent bytes left in output buffer
volatile int bytesAvailable;    // number of unread bytes in input buffer
int rx_lock;                    // lock out recieve interrupt routine
int tx_lock;

int gotNewline;

// lock out transmit interrupt routine

extern LED_struct LED;
extern int LED_ON;
extern int LED_OFF;
extern int LED_BLINK;

extern int wait_flag;

void serial_begin(unsigned long int baud){
  inputHead = 0;          // index to head of input buffer
  inputTail = 0;          // index to tail of input buffer
  bytesAvailable = 0;     // reset byte count
  outputHead = 0;
  outputTail = 0;
  bytesRemaining = 0;
  rx_lock = 0;            // recieve interrupt begins unlocked
  tx_lock = 0;            // unlock transmit interrupt
  gotNewline = 0;
  // initialize UART for serial communication
  U1MODE = 0;                     // clear mode register
  U1BRG = (unsigned int)((FCY/(16 * baud)) - 1);// Baudrate generator, 85@79.xxx=115200
  U1MODEbits.UEN = 0;             // use just RX and TX pins, not CTS/RTS/BCLK
  U1STA = 0;                      // clear status register
  U1STAbits.URXISEL = 0;          // fire RX interrupt each received byte
  U1STAbits.UTXISEL0 = 0;         // fire TX interrupt when h/w buffer's empty
  U1STAbits.UTXISEL1 = 0;
  U1MODEbits.UARTEN = 1;          // enable the UART RX
  U1STAbits.UTXEN = 1;            // enable the UART TX
  //U1MODEbits.LPBACK = 1;        // enable loopback mode
  IEC0bits.U1TXIE = 1;            // enable transmit interrupts
  IFS0bits.U1TXIF = 0;            // clear transmit interrupt flag
  IEC0bits.U1RXIE = 1;            // enable recieve interrupts
  IFS0bits.U1RXIF = 0;            // clear recieve interrupt flag
}

// UART1 Recieve interrupt service routine
void _ISRFASTER _U1RXInterrupt(void){
  if(!rx_lock){ // check for interrupt lock
    while(RX_AVAIL){
    // copy contents of recieve register into user buffer
    // circular buffer: the index loops back to the front when it gets to end
      inputBuffer[inputHead] = RX_REG;         // grab byte from hardware buffer
      if(inputBuffer[inputHead] == '\n')
        gotNewline = 1;
      inputHead = (inputHead+1) % BUFFER_SIZE; // increment head index
      bytesAvailable++; // increment number of bytes available in user buffer
    }
  }
  IFS0bits.U1RXIF = 0; // reset interrupt flag if not locked
}
//}

// TX Interrupt transmits data to the serial port during system idle time
void _ISRFASTER _U1TXInterrupt(void) {
  if(tx_lock == 0){  // check for interrupt lock
    while(!TX_FULL && bytesRemaining > 0){
      TX_REG = outputBuffer[outputTail];         // grab byte from user buffer
      outputTail = (outputTail+1) % BUFFER_SIZE; // increment tail index
      bytesRemaining--; // decrement number of bytes remaining in user buffer
    }
  }
  TX_INTFLAG = 0;
}

void serial_ping(void){
  // throw transmit interrupt flag 
  TX_INTFLAG = 1;
}

void serialTest(void){
      while (TX_FULL) // wait for transmit buffer to make space for another char
      ;
      TX_REG = 0x55; // send 'U' (square wave) to serial
}

/* writes an array to serial port immediately*/
void serial_write(const char *buffer, int size) {
  tx_lock = 1; // lock transmit interrupt
  // purge output buffer first
  while(bytesRemaining){
    while(TX_FULL) // wait for slot to open in buffer
      ;
    TX_REG = outputBuffer[outputTail];
    outputTail = (outputTail+1) % BUFFER_SIZE; // increment tail index
    bytesRemaining--; // decrement number of bytes remaining in user buffer
  }
  while (size--) { // keep talking until we've nothing left to say
    while (TX_FULL) // wait for transmit buffer to make space for another char
      ;
    TX_REG = *buffer; // push next char onto transmit buffer
    buffer++;
  }
  tx_lock = 0; // unlock transmit interrupt
}

/*take a char array and copies it to output buffer to be delivered later*/
void serial_bufWrite(const char *buffer, int size){
  while (size && *buffer != '\0') { // copy entire string
    outputBuffer[outputHead] = *buffer; // push next char onto transmit buffer
    outputHead = (outputHead+1) % BUFFER_SIZE;
    buffer++;         // increment outgoing string buffer
    bytesRemaining++; // increment unsent byte count
    size--;
  }
}

/*returns number of unread bytes available in user buffer*/
int serial_available(void){
  return bytesAvailable;
}

int serial_remaining(void){
  return bytesRemaining;
}

/* read size bytes from inputBuffer into provided char buffer. Actual allocated
 * size of buffer must be at least size+1 for terminating null character*/
void serial_read(char* buffer, int size){
  // throw recieve interrupt flag to purge RX hardware buffer
  rx_lock = 1;   // lockout interrupt service routine.
  // copy inputBuffer into buffer
  if(bytesAvailable){
    while(size && bytesAvailable){
      *buffer = inputBuffer[inputTail];
      buffer++;
      inputTail = (inputTail+1) % BUFFER_SIZE; // increment tail index
      bytesAvailable--;
      size--;
    }
    *buffer = '\0'; // add terminating character to string
    gotNewline = 0;
  }
  rx_lock = 0;    // unlock receive interrupt routine
  RX_INTFLAG = 1; // reset interrupt flag
}
int serial_gotNewline(void){
  return gotNewline;
}

