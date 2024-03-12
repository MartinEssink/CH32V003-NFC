#include "ch32v003fun.h"
#include <stdio.h>
#include <string.h>

#define NFC_CMD_WUPA  0x52
#define NFC_CMD_REQA  0x26
#define NFC_CMD_CL1   0x93
#define NFC_CMD_CL2   0x95
#define NFC_CMD_READ  0x30
#define NFC_CMD_WRITE   0xA2

#define NFC_CASCADE_TAG 0x88
#define NFC_NVB_NONE  0x20
#define NFC_NVB_ALL   0x70
#define NFC_SAK_NC    0x04 //Select acknowledge uid not complete
#define NFC_SAK_C   0x00 //Select acknowledge uid complete, Type 2 (PICC not compliant to ISO/IEC 14443-4)

#define NFC_ACK     0xA
#define NFC_NAK_CRC   0x1
#define NFC_NAK     0x0

#define NFC_CLOCK_MULTIPLIER  2 // Multiple of 13.56MHz
#define NFC_RX_BIT_DELAY    (64*NFC_CLOCK_MULTIPLIER)
#define NFC_TX_DELAY_HIGH   (916*NFC_CLOCK_MULTIPLIER) // 1236-5*64
#define NFC_TX_DELAY_LOW    (852*NFC_CLOCK_MULTIPLIER) // 1172-5*64

#define NFC_BUFFER_LENGTH     32 // Must be a multiple of 2

uint32_t nfc_bit_time;
uint8_t buffer[NFC_BUFFER_LENGTH];

struct nfc_data_t {
  uint8_t ATQA[2];
  // We store the cascase tag right before the tag storage.
  // This way we don't need to copy it to the buffer for transmission.
  uint8_t CT;
  uint8_t storage[128];
};

struct nfc_data_t nfc_data = {
  {0x44, 0x00},
  NFC_CASCADE_TAG,
  {
    0x11, 0x22, 0x33, 0x88, // UID0-2, BCC0 (XOR CT, UID0-2)
    0x44, 0x55, 0x66, 0x77, // UID3-7
    0x00, 0x00, 0x00, 0x00, // BCC1 (XOR UID3-7), INT, Lock0, Lock1
    0xE1, 0x10, 0x0F, 0x00, // CC: Magic number, Version, Size, RW Access. (NFCForum-TS-Type-2-Tag_1.1)

    0x03,//Type NDEF
    0x19,//Length

    0xD1, 0x01, 0x15, 0x54, 
    0x02, 0x65, 0x6E,

    0x48, 0x69, 0x2C, 0x20, 0x49, 0x20, 0x61, 0x6D, 0x20, 0x43, 0x48, 0x33, 0x32, 0x56, 0x30, 0x30, 0x33, 0x21,

    0xFE //Terminate
  }
};

void nfc_crc16(uint8_t *data, uint8_t len){
  uint8_t ch;
  uint16_t wCrc = 0x6363; // ITU-V.41

  do {
    ch = *data++;
    ch = (ch^(uint8_t)((wCrc) & 0x00FF));
    ch = (ch^(ch<<4));
    wCrc = (wCrc >> 8)^((uint16_t)ch << 8)^((uint16_t)ch<<3)^((uint16_t)ch>>4);
  } while (--len);

  *data++ = (uint8_t) (wCrc & 0xFF);
  *data = (uint8_t) ((wCrc >> 8) & 0xFF);
}

void nfc_tx_bit(uint8_t bit){
  while( ((int32_t)( SysTick->CNT - nfc_bit_time )) < 0 );

  if(bit){
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(7*4);
  }else{
    GPIOD->CFGLR &= ~(0xf<<(7*4));
  }
  
  nfc_bit_time += NFC_RX_BIT_DELAY;
  while( ((int32_t)( SysTick->CNT - nfc_bit_time )) < 0 );

  if(bit){
    GPIOD->CFGLR &= ~(0xf<<(7*4));
  }else{
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(7*4);
  }

  nfc_bit_time += NFC_RX_BIT_DELAY;
}

void nfc_tx_bytes(uint8_t *data, uint8_t len){
  uint8_t tx_parity;

  //Start bit
  nfc_tx_bit(1);

  for(uint8_t tx_byte = 0; tx_byte<len; tx_byte++){
    tx_parity = 1;
    for(uint8_t tx_bit = 0; tx_bit<8; tx_bit++){
      if( data[tx_byte] & (1U << tx_bit) ){
        tx_parity ^= 1;
        nfc_tx_bit(1);
      }else{
        nfc_tx_bit(0);
      }
    }
    nfc_tx_bit(tx_parity);
  }

  // Stop 'bit'
  while( ((int32_t)( SysTick->CNT - nfc_bit_time )) < 0 );
  GPIOD->CFGLR &= ~(0xf<<(7*4));
}

void nfc_tx_ack(uint8_t data){
  //Start bit
  nfc_tx_bit(1);

  for(uint8_t tx_bit = 0; tx_bit<4; tx_bit++) nfc_tx_bit( data & (1U << tx_bit) );

  // Stop 'bit'
  while( ((int32_t)( SysTick->CNT - nfc_bit_time )) < 0 );
  GPIOD->CFGLR &= ~(0xf<<(7*4));
}

void nfc_read(uint8_t block){
  uint16_t pos = (uint16_t)block * 4;

  for(uint8_t i=0; i < 16; i++){
    buffer[i] = (pos < sizeof(nfc_data.storage)) ? nfc_data.storage[pos] : 0;
    pos++;
  }
  
  nfc_crc16(buffer, 16);
  nfc_tx_bytes(buffer, 18);
}

void nfc_write(uint8_t block){
  uint16_t pos = (uint16_t)block * 4;

  nfc_crc16(buffer, 8);
  if ((buffer[8] | buffer[9]) == 0){
    for(uint8_t i=2; i < 6; i++){ //byte 2-5 contains Data
      if (pos < sizeof(nfc_data.storage)) nfc_data.storage[pos] = buffer[i];
      pos++;
    }
    nfc_tx_ack(NFC_ACK); // ACK
  }else{
    nfc_tx_ack(NFC_NAK_CRC); // NAK (CRC)
  }
}

void EXTI7_0_IRQHandler( void ) __attribute__((interrupt));
void EXTI7_0_IRQHandler( void ){
  // The interrupt triggers somewhere in the middle of the first pulse, 
  // so we can use this directly as the starting point. Offset if needed.
  nfc_bit_time = SysTick->CNT; 

  // We do not have enough cycles to initiate the entire buffer, we it on the fly.
  buffer[0] = 0;
  uint8_t rx_bit = 0;
  uint8_t rx_byte = 0;
  uint8_t i;

rx_loop:
  i = 3;
  do{
    nfc_bit_time += NFC_RX_BIT_DELAY;
    while( ((int32_t)( SysTick->CNT - nfc_bit_time )) < 0 );
    if((GPIOD->INDR & (1<<4))==0){
      switch(i){
        // There is no break here on purpose, we slide to the next goto.
        case 0: rx_bit++; // Increment by 2, write
        case 1: rx_bit++; // Increment by 1, write
        case 2: goto rx_write; // Write
        case 3: goto rx_reset; // Consecutive pulses, reset
      }
    }
  } while(i--); 
  goto rx_done;

rx_write:
  if(rx_bit>17){
    if(++rx_byte >= NFC_BUFFER_LENGTH) goto rx_reset;
    buffer[rx_byte] = 0;
    rx_bit -= 18;
  }

  if(rx_bit & 0x01) buffer[rx_byte] |= (1 << (rx_bit>>1));

  rx_bit += 2;
  goto rx_loop;

rx_done:
  if (rx_bit > 7) rx_byte++;
  if( rx_byte == 0 ) goto rx_reset;

  if (rx_bit & 0x01){ // Odd rx bit, we wrote a 1
    nfc_bit_time += NFC_TX_DELAY_HIGH; 
  }else{
    nfc_bit_time += NFC_TX_DELAY_LOW;
  }

  switch(buffer[0]){
    // Wakeup commands
    case NFC_CMD_REQA: //REQuest (type A)
    case NFC_CMD_WUPA: //Wake-UP (type A)
      // nfc_tx_bytes(ATQA, sizeof(ATQA)); // Answer To reQuest (type A)
      nfc_tx_bytes(nfc_data.ATQA, sizeof(nfc_data.ATQA)); // Answer To reQuest (type A)
      break;

    case NFC_CMD_CL1: // Cascade Level 1 (anticollision)
      if(buffer[1] == NFC_NVB_NONE){
        nfc_tx_bytes(&nfc_data.CT, 5);
      }else if(buffer[1] == NFC_NVB_ALL){
        buffer[0] = NFC_SAK_NC;
        nfc_crc16(buffer,1);
        nfc_tx_bytes(buffer, 3);
      }
      break;

    case NFC_CMD_CL2: // Cascade Level 2 (anticollision)
      if(buffer[1] == NFC_NVB_NONE){
        nfc_tx_bytes(&nfc_data.storage[4], 5);
      }else if(buffer[1] == NFC_NVB_ALL){
        buffer[0] = NFC_SAK_C;
        nfc_crc16(buffer,1);
        nfc_tx_bytes(buffer, 3);
      }
      break;

    case NFC_CMD_READ:
      nfc_read(buffer[1]);
      break;

    case NFC_CMD_WRITE:
      nfc_write(buffer[1]);
      break;
  }

rx_reset:
  EXTI->INTFR = 1<<4;
}

void nfc_init(){
  // Enable required peripherals
  RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;

  // Enable OPA, use PD7 and PD0 as input
  EXTEN->EXTEN_CTR |= EXTEN_OPA_EN | EXTEN_OPA_PSEL | EXTEN_OPA_NSEL;

  // Setup GPIOD
  GPIOD->CFGLR &= ~( (0xF<<(4*0)) | (0xF<<(4*4)) | (0xF<<(4*7)) );
  GPIOD->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD)<<(4*4);

  // External interupt triggers on the first pulse of the signal
  AFIO->EXTICR = 3<<(4*2); // EXT4 on PORTD
  EXTI->INTENR = 1<<4;     // Enable EXT4
  EXTI->FTENR  = 1<<4;     // Falling edge trigger

  // Transmission is by subcarrier at 1/16 of 13.56MHz
  // Setup timer for transmit, we modulate SIG (PD7)
  TIM2->PSC = 0x0000; // Prescaler 
  TIM2->ATRLR = 31; // Auto Reload - sets period
  TIM2->SWEVGR |= TIM_UG; // Reload immediately
  TIM2->CCER |= TIM_CC4E | TIM_CC4P; // Enable CH4 output, positive pol
  TIM2->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1; // CH4 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
  TIM2->CH4CVR = 16; // Set the Capture Compare Register value to 50%
  TIM2->BDTR |= TIM_MOE; // Enable TIM2 outputs
  TIM2->CTLR1 |= TIM_CEN; // Enable TIM2

  // Enable the interrupt
  NVIC_EnableIRQ( EXTI7_0_IRQn );
}

int main(){
  SystemInit();
  nfc_init();

  while(1){
    // Delay_Ms(1000);
  }
}