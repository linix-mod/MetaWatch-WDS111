/** 
 * @file  hal_bt.c
 ***************************************************************************/
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"

#include "Messages.h"
#include "MessageQueues.h"
#include "BufferPool.h"

#include <msp430x54x.h>
#include <portmacro.h>
#include <hal_lpm.h>
#include "hal_compat.h"

#include <btstack/hal_uart_dma.h>
#include <hal_digital_v2_defs.h>
#include <hal_clock_control.h>

extern void hal_cpu_set_uart_needed_during_sleep(uint8_t enabled);

void dummy_handler(void){};

// rx state
static uint16_t  bytes_to_read = 0;
static uint8_t * rx_buffer_ptr = 0;

// tx state
static uint16_t  bytes_to_write = 0;
static uint8_t * tx_buffer_ptr = 0;

// handlers
static void (*rx_done_handler)(void) = dummy_handler;
static void (*tx_done_handler)(void) = dummy_handler;
static void (*cts_irq_handler)(void) = dummy_handler;

//static unsigned int config;

/**
 * @brief  Deinits the serial communications peripheral and GPIO ports
 *         to communicate with the PAN BT .. assuming 16 Mhz CPU
 *
 * @param  none
 *
 * @return none
 */
void hal_uart_dma_deinit(void)
{
  // Stop ACLK
  P11DIR |= BIT0;
  P11OUT &= ~BIT0;
  P11SEL &= ~BIT0;

  // Configure Bluetooth SHUTDOWN pin (active low)
  BT_RST_SEL &= ~BT_RST_PIN;
  BT_RST_PDIR |= BT_RST_PIN;
  BT_RST_POUT &= ~BT_RST_PIN; // Low - stop chip
}

/**
 * @brief  Initializes the serial communications peripheral and GPIO ports 
 *         to communicate with the PAN BT .. assuming 16 Mhz CPU
 * 
 * @param  none
 * 
 * @return none
 */
void hal_uart_dma_init(void)
{
  // Stop ACLK
  P11DIR |= BIT0;
  P11OUT &= ~BIT0;
  P11SEL &= ~BIT0;

  // Configure port pins as UART I/O
  BT_COMM_SEL |= BT_TX_PIN | BT_RX_PIN;
  BT_COMM_PDIR |= BT_TX_PIN;
  BT_COMM_PDIR &= ~BT_RX_PIN;

  // Configure RS232 control lines
  BT_CTRL_SEL &= ~(BT_CTS_PIN | BT_RTS_PIN);
  BT_CTRL_PREN &= ~BT_CTS_PIN; // no pull resistor for CTS pin
  BT_CTRL_PDIR |= BT_RTS_PIN;
  BT_CTRL_PDIR &= ~BT_CTS_PIN;
  BT_CTRL_POUT |= BT_RTS_PIN; // High - not ready to receive

  // Configure Bluetooth SHUTDOWN pin (active low)
  BT_RST_SEL &= ~BT_RST_PIN;
  BT_RST_PDIR |= BT_RST_PIN;
  BT_RST_POUT &= ~BT_RST_PIN; // Low - stop chip

  // Wait for proper reset propagation
  vTaskDelay(1000 / portTICK_RATE_MS);

  // Wait until chip signals reset state by setting CTS pin to high
  //while (!(BT_CTRL_PIN & BT_CTS_PIN)) ;

  // Start bluetooth chip
  BT_RST_POUT |= BT_RST_PIN;

  // Enable ACLK to provide 32 kHz clock to Bluetooth module
  P11SEL |= BIT0;
  P11DIR |= BIT0;

  // Wait for proper reset propagation
  vTaskDelay(1000 / portTICK_RATE_MS);

  // Wait until CTS is low
  while ((BT_CTRL_PIN & BT_CTS_PIN)) ;

  // Configure UART
  EnableSmClkUser(BT_UART_USER);
  BT_UART_CTL1 |= UCSWRST;              //Reset State
  BT_UART_CTL0 = UCMODE_0;
  BT_UART_CTL0 &= ~UC7BIT;              // 8bit char
  BT_UART_CTL1 |= UCSSEL__SMCLK;
  BT_UART_CTL1 &= ~UCSWRST;             // Run UART
  hal_uart_dma_set_baud(115200);

/*
  config = UCSCTL0;
  config = UCSCTL1;
  config = UCSCTL2;
  config = UCSCTL3;
  config = UCSCTL4;
  config = UCSCTL5;
  config = UCSCTL6;
  config = UCSCTL7;
  config = UCSCTL8;
  config = BT_UART_CTL0;
  config = BT_UART_CTL1;
  config = BT_UART_BR0;
  config = BT_UART_BR1;
  config = BT_UART_MCTL;
  config = UCA1IRRCTL;
  config = UCA1IRTCTL;
  config = UCA1ABCTL;
*/
}

static const struct {
  uint32_t baud;
  unsigned short selector;
  unsigned char modulation;
} SupportedBauds[] = {
  // 16,777,216 SMCLK
  { 2000000, 8, 3 },
  { 115200, 145, 5 },
};

int hal_uart_dma_set_baud(uint32_t baud)
{
  int i, find;

  find = -1;
  for (i = 0; i < (sizeof(SupportedBauds) / sizeof(SupportedBauds[0])); i++)
  {
    if (SupportedBauds[i].baud == baud)
    {
      find = i;
      break;
    }
  }
  if (find == -1)
    return -1;

  BT_UART_CTL1 |= UCSWRST;  // Reset UART

  BT_UART_BR0 = (unsigned char)(SupportedBauds[find].selector & 0xFF);
  BT_UART_BR1 = (unsigned char)((SupportedBauds[find].selector & 0xFF00) >> 8);
  BT_UART_MCTL = SupportedBauds[find].modulation << 1;

  BT_UART_CTL1 &= ~UCSWRST; // Resume UART

  return 0;
}

void hal_uart_dma_set_block_received( void (*the_block_handler)(void)){
  if (the_block_handler)
    rx_done_handler = the_block_handler;
  else
    rx_done_handler = dummy_handler;
}

void hal_uart_dma_set_block_sent( void (*the_block_handler)(void)){
  if (the_block_handler)
    tx_done_handler = the_block_handler;
  else
    tx_done_handler = dummy_handler;
}

void hal_uart_dma_set_csr_irq_handler( void (*the_cts_irq_handler)(void))
{
  BT_CTRL_IE &= ~BT_CTS_PIN;
  if (the_cts_irq_handler)
  {
    cts_irq_handler = the_cts_irq_handler;
    BT_CTRL_IFG = 0;
    BT_CTRL_IV = 0;
    BT_CTRL_IES &= ~BT_CTS_PIN; // IRQ on rising edge
    BT_CTRL_IE |= BT_CTS_PIN; // Enable interrupt
  }
  else
  {
    cts_irq_handler = dummy_handler;
  }
}

//static uint8_t sent_data[256];
//static unsigned int sent_data_pos = 0;

void hal_uart_dma_send_block(const uint8_t * data, uint16_t len){
    
  // printf("hal_uart_dma_send_block, size %u\n\r", len);
  EnableSmClkUser(BT_UART_USER);

  BT_UART_IE &= ~UCTXIE ;  // disable TX interrupts

  tx_buffer_ptr = (uint8_t *) data;
  bytes_to_write = len;
  //sent_data_pos = 0;

  BT_UART_IE |= UCTXIE;    // enable TX interrupts
}

static inline void hal_uart_dma_enable_rx(void){
  //P1OUT &= ~BIT4;  // = 0 - RTS low -> ok
  BT_CTRL_POUT &= ~BT_RTS_PIN;
}

static inline void hal_uart_dma_disable_rx(void){
  //P1OUT |= BIT4;  // = 1 - RTS high -> stop
  BT_CTRL_POUT |= BT_RTS_PIN;
}

// int used to indicate a request for more new data
void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len){

  EnableSmClkUser(BT_UART_USER);

  BT_UART_IE &= ~UCRXIE ;  // disable RX interrupts

  rx_buffer_ptr = buffer;
  bytes_to_read = len;

  BT_UART_IE |= UCRXIE;    // enable RX interrupts

  hal_uart_dma_enable_rx();     // enable receive

}

void hal_uart_dma_set_sleep(uint8_t sleep){
  if (sleep)
    DisableSmClkUser(BT_UART_USER);
  else
    EnableSmClkUser(BT_UART_USER);
}

// block-wise "DMA" RX/TX UART driver
#pragma vector=BT_UART_VECTOR
__interrupt
void usbRxTxISR(void){ 

  // find reason
  switch (BT_UART_IV){

    case 2: // RXIFG
        if (bytes_to_read == 0) {
          hal_uart_dma_disable_rx();
          BT_UART_IE &= ~UCRXIE ;  // disable RX interrupts
          return;
        }
        *rx_buffer_ptr = BT_UART_RXBUF;
        ++rx_buffer_ptr;
        --bytes_to_read;
        if (bytes_to_read > 0) {
          return;
        }
        hal_uart_dma_disable_rx();
        BT_UART_IE &= ~UCRXIE ; // disable RX interrupts
    
        (*rx_done_handler)();

        // force exit low power mode
        EXIT_LPM_ISR();
        
        break;

    case 4: // TXIFG
        if (bytes_to_write == 0){
          BT_UART_IE &= ~UCTXIE ;  // disable TX interrupts
          return;
        }
        BT_UART_TXBUF = *tx_buffer_ptr;
        /*sent_data[sent_data_pos] = *tx_buffer_ptr;
        if (sent_data_pos<255)
          sent_data_pos++;*/
        ++tx_buffer_ptr;
        --bytes_to_write;

        if (bytes_to_write > 0) {
          return;
        }

        BT_UART_IE &= ~UCTXIE ;  // disable TX interrupts

        (*tx_done_handler)();

        // force exit low power mode
        EXIT_LPM_ISR();

        break;

    default:
        break;
  }
}


// CTS ISR

extern void ehcill_handle(uint8_t action);
#define EHCILL_CTS_SIGNAL      0x034

#pragma vector=BT_CTRL_VECTOR
__interrupt
void ctsISR(void){ 
  BT_CTRL_IV = 0;
  (*cts_irq_handler)();
}
