// inslude the SPI library:
#include <SPI.h>

//PB3 - MOSI
//PB4 - MISO
//PB5 - SCK
//PC1 - CS

//PC0 - Button

//PC4 - SDA
//PC5 - SCL

//PD0 - RxD
//PD1 - TxD

//PD3 - VTX On/Off
//PD6 - OC0A - RED PWM OUT
//PB1 - OC1A - GREEN PWM OUT
//PB2 - OC1B - BLUE PWM OUT

#define SCK (1<<5)
#define MISO (1<<4)
#define MOSI (1<<3)

#define VTX (1<<3)
#define BUTTON (1<<0)
#define CS (1<<1)

// set pin 10 as the slave select for the digital pot:
const int slaveSelectPin = 10;

//Write 25-Bit to teh RTC6705
//4-address bits
//1-read write bit
//20-bit payload LSB
//D20..D0 -> A0 A1 A2 A3 RW D0 D1 D2...D20
void SoftSPISend(uint8_t address, uint8_t upper, uint16_t lower)
{ 
  PORTB &= ~SCK;  //Clear sck
  PORTB &= ~MOSI;
  PORTC &= ~CS;
  //Send address bits
  for(uint8_t i = 0; i < 4; i++)
  {
    PORTB |= (address & (1 << i))?(MOSI):(0);
    asm("NOP");
    asm("NOP");
    PORTB |= SCK;
    delayMicroseconds(2);
    PORTB &= ~SCK;
    asm("NOP");
    asm("NOP");
    PORTB &= ~MOSI;
    asm("NOP");
    asm("NOP");
  }
  asm("NOP");
  asm("NOP");
  //Send R/W bit
  PORTB |= MOSI;
  asm("NOP");
  asm("NOP");
  PORTB |= SCK;
  delayMicroseconds(2);
  PORTB &= ~SCK;
  asm("NOP");
  asm("NOP");
  PORTB &= ~MOSI;
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  for(uint8_t i = 0; i < 16; i++)
  {
    PORTB |= (lower & (1 << i))?(MOSI):(0);
    asm("NOP");
    asm("NOP");
    PORTB |= SCK;
    delayMicroseconds(2);
    PORTB &= ~SCK;
    asm("NOP");
    asm("NOP");
    PORTB &= ~MOSI;
    asm("NOP");
    asm("NOP");
  }
  asm("NOP");
  asm("NOP");
  for(uint8_t i = 0; i < 4; i++)
  {
    PORTB |= (upper & (1 << i))?(MOSI):(0);
    asm("NOP");
    asm("NOP");
    PORTB |= SCK;
    delayMicroseconds(2);
    PORTB &= ~SCK;
    asm("NOP");
    asm("NOP");
    PORTB &= ~MOSI;
    asm("NOP");
    asm("NOP");
  }
  PORTB &= ~SCK;
  PORTB &= ~MOSI;
  PORTC |= CS;
}

uint8_t inputStream = 0;
uint8_t state = 0;
uint8_t channel = 0;

enum STATES
{
  STATE_IDLE=0,
  STATE_FS,
  STATE_RB,
  STATE_VTXOFF
};
//We can omit the upper 4 bits of the 20-bit dataword of reg1 as it is always 0100
uint16_t fatshark[] = {0x610c,0x6500,0x68b4,0x6ca8,0x709c,0x7490,0x7884,0x7c38};
uint16_t raceband[] = {0x510a,0x5827,0x5f84,0x66a1,0x6dbe,0x751b,0x7c38,0x8395};

uint8_t colors[8][3] = {{0, 0xff, 0},{0, 0, 0xff},{0xff, 0xff, 0},{0xff, 0, 0xff},{0, 0xff, 0xff},{0xff, 0xff, 0xff},{0x0f, 0xf0, 0xff},{0xff, 0, 0}};


void setup() {
  // set the slaveSelectPin as an output:
  pinMode(A0, INPUT_PULLUP);
  PORTB = 0;
  
  DDRB = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5); //Set MOSI, SCK, PWM Green and PWM Blue as outputs
  DDRC = CS;            //Set CS as output
  PORTC = (1 << 0) | CS;           //Pull-Up for the button
  DDRD = (1 << 3) | (1 << 1) | (1 << 6); //Set VTXOnOff and TxD as outputs, RED PWM

  //PORTD |= (1 << 3);          //Shut-down the VTX
  PORTD &= ~(1 << 3);          //Enable the VTX

  TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); //Fast PWM; non inverting
  TCCR0B = (1 << CS01) | (1 << CS00);         //Clk/64
  OCR0A = 0;

  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);  //8-bit fast pwm
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);    //CLk 64
  OCR1A = 0;
  OCR1B = 0;
  
  SoftSPISend(0x00, 0, 400);
  SoftSPISend(0x01, 4, fatshark[0]);
  OCR0A = colors[0][0];
  OCR1A = colors[0][1];
  OCR1B = colors[0][2];
}
//The R-Register is kept at 400, that gives 40kHz selection
//The output frequency can be calculated by Frf = 2*(64*N+A)*(F_osc/R)
//uint16_t channelFatsharkN[8] = {2242, 2250, 2257, 2265, 2273, 2281, 2289, 2296};
//uint8_t channelFatsharkA[8] = {12, 0, 52, 40, 28, 16, 4, 56};
//uint16_t channelRacebandN[8] = {2220, 2224, 2239, 2253, 2267, 2282, 2296, 2311};
//uint8_t channelRacebandA[8] = {45, 39, 4, 33, 62, 27, 56, 21};



#define MASK 0xFF
#define BUTTON_DOWN 0xF0

void loop() {
  inputStream <<= 1;
  inputStream |= (PINC & BUTTON)?(1):(0);
  if((inputStream & MASK) == BUTTON_DOWN)  //Button down
  {
    channel++;
    if(channel > 8)
    {
      channel = 0;
    }
    //SoftSPISend(0x00, 0, 400);
    SoftSPISend(0x01, 4, fatshark[channel]);
    OCR0A = colors[channel][0];
    OCR1A = colors[channel][1];
    OCR1B = colors[channel][2];
  }
  delay(50);
}

