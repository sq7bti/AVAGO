/*
  AVAGO ADNS-7550

 The circuit:

 // Default SPI pinout for MSP430G2553
// MISO P1.7
// MOSI P1.6
// CLK  P1.5
// set pin 8 as the slave select for the digital pot:
// CS   P1.2

                                 ______________
                            Vcc  |            | GND
                  RED_LED   P1_0 |            | P2_6   QRA
               MOTION/IRQ   P1_1 |            | P2_7   QRB
                      CS    P1_2 |            | TEST
                            P1_3 |            | RESET
        DB9 5         MMB   P1_4 |            | P1_7   MISO
                      CLK   P1_5 |            | P1_6   MOSI
        DB9 9         RMB   P2_0 |            | P2_5   LMB         DB9 6
X2      DB9 4         QXA   P2_1 |            | P2_4   QYA         DB9 3 Y1
X1      DB9 2         QXB   P2_2 |____________| P2_3   QYB         DB9 1 Y2

mouse pinout DB9:
     Y2  X1  Y1  X2  MMB
      U   D   L   R  PotY
   _______________________
   \  1   2   3   4   5  /
    \                   /
     \__6___7___8___9_ /
       LMB  +  gnd RMB
                   PotX
       
scroll wheel protocols:
micomys: 20ms apart on MMB, 600 .. 650 us - six hundred  http://wiki.icomp.de/wiki/Micromys_Protocol

cocolino: 20ms apart on MMB, 64 .. 70 us - sixty eight, 99.7% PWM
================== CocolinoTest ==================
FOURTH DOWN - QXA - X2 pin 4
FIVETH DOWN - QYA - Y1 pin 3

QXA QXB QYA QYB RMB
  0   0   0   0   0   wheel up
  1   0   0   0   0   wheel down
  0   1   1   1   1   FOURTH down
  1   1   1   0   1   FIVETH up  

==================
mm https://github.com/paulroberthill/AmigaPS2Mouse
26us ... 32us
WiP
==================
EZ Mouse

Wheel up    right
Wheel down  left

QXA QXB QYA QYB RMB
  0   1   0   0   0   wheel up
  0   0   0   1   0   wheel down

*/

//#define DEBUG 1

// wheel protocol used (use appropriate driver on host)
#define COCOLINO

#define  REG_PRODUCT_ID   0x00
#define  REG_INV_PRODUCT_ID   0x3E
#define  REG_REVISION_ID  0x01
#define  REG_INV_REVISION_ID  0x3F

#define  REG_MOTION       0x02
#define  REG_DELTA_X      0x03
#define  REG_DELTA_Y      0x04
#define  REG_DELTA_XY_H   0x05

#define  REG_SQUAL        0x06
#define  REG_MAX_PIXEL    0x09
#define  REG_PIXEL_SUM    0x0a
#define  REG_MIN_PIXEL    0x0b

#define CONFIG2_400CPI    0x08
#define CONFIG2_800CPI    0x28
#define CONFIG2_1200CPI    0x48
#define CONFIG2_1600CPI    0x68
#define  REG_CONFIGURATION2  0x12

#define  LASER_3MA        0x00
#define  LASER_5MA        0x30
#define  LASER_10MA       0xC0

#define  LASER_RANGE      LASER_3MA
/* 0x00 -> 33.6%, 0xff -> 100%*/
#define  LASER_POWER      0x70

#define  REG_LASER_CTRL0  0x1a
#define  REG_LASER_CTRL1  0x1f
#define  REG_LSRPWR_CFG0  0x1c
#define  REG_LSRPWR_CFG1  0x1d

#define  REG_OBSERVATION  0x2e
#define  REG_MBURST       0x42
#define  REG_POWER_UP_RESET 0x3a

// include the SPI library:
#include <SPI.h>

#define NCS P1_2

// MOTION pin set to trigger IRQ P1_1
#define MOTION_PIN P1_1

// quadrature inputs
#define QRA P2_6
#define QRB P2_7
#define LMB P2_5
#define MMB P1_4
#define RMB P2_0
#define WHEEL_DELAY 5000

// quadrature outputs
#define QXA P2_1
#define QXB P2_2
#define QYA P2_3
#define QYB P2_4

//RED_LED
#define RED_LED P1_0
#define MMBUTT PUSH2

// approx 5us instead of 13us
#define GPIO_OUT_SET_SUB(port, pin) (P##port##OUT |=  (1<<pin))
#define GPIO_OUT_CLR_SUB(port, pin) (P##port##OUT &= ~(1<<pin))

#define MHZ 12
#define SET_CPU_CLOCK(mhz) { DCOCTL = CALDCO_##mhz##MHZ; BCSCTL1 = CALBC1_##mhz##MHZ; };

unsigned int motion = 200, k = 0, adc_avg;
unsigned int quad_x, quad_y;
signed delta_x, delta_y;

volatile unsigned char SW_state;
volatile signed int scroll_change = 0;
volatile byte mmb_trigger; //, buttons;
int adc[16] = {0}; //Sets up an array of 16 integers and zero's the values

void setup() {

  // set the NCS as an output:
  pinMode(NCS, OUTPUT);

  // configure RED LED for output
  pinMode(RED_LED, OUTPUT);
  pinMode(MOTION_PIN, INPUT_PULLUP);

  // initialize SPI:
  SPI.begin(); 
  SPI.setClockDivider(16); // 1MHz SPI

  delayMicroseconds(250);

  digitalWrite(NCS,LOW);
  delayMicroseconds(50);
  digitalWrite(NCS,HIGH);
  delayMicroseconds(50);

//#if DEBUG
  get_reg(REG_PRODUCT_ID); // 0x00 -> 0x32
  get_reg(REG_INV_PRODUCT_ID); // 0x3e -> 0xfc
  get_reg(REG_REVISION_ID); // 0x01 -> 0x03
  get_reg(REG_INV_REVISION_ID); // 0x3f -> 0xcd
//#endif // DEBUG

//  delayMicroseconds(250);

  set_reg(REG_POWER_UP_RESET, 0x5a); // (0x80 | 0x3a = 0xba)

/* LASER_3MA LASER_5MA LASER_10MA */
  set_reg(REG_LASER_CTRL0, (0xC0) & LASER_RANGE); // 0x1a (0x9a)
  set_reg(REG_LASER_CTRL1, (0xC0) & ~LASER_RANGE); // 0x1f (0x9f)
  set_reg(REG_LSRPWR_CFG0, LASER_POWER); // 0x1c (0x9c)
  set_reg(REG_LSRPWR_CFG1, ~LASER_POWER); // 0x1d (0x9d)

/* CONFIG2_400CPI, CONFIG2_800CPI, CONFIG2_1200CPI, CONFIG2_1600CPI */
  set_reg(REG_CONFIGURATION2, CONFIG2_400CPI); //   0x12 (0x92)

  // wait for at least one frame ?
  delayMicroseconds(250);

  // clear observation register
  set_reg(REG_OBSERVATION, 0x00);

  // wait for at least one frame
  delayMicroseconds(250);
  // and check observation register, all bits 0-3 must be set
  while((get_reg(REG_OBSERVATION) & 0x0F) != 0x0F) {
    delayMicroseconds(500);
    --motion;
    if(!motion) {
      motion = 200;
      if(k) {
        digitalWrite(RED_LED, HIGH);
        k = 0;
      } else {
        digitalWrite(RED_LED, LOW);
        k = 1;
      }
    }
  }

  get_reg(REG_MOTION);                     // read from registers one time regardless of the motion pin state 0x02
  get_reg(REG_DELTA_X);                    // 0x03
  get_reg(REG_DELTA_Y);                    // 0x04
  get_reg(REG_DELTA_XY_H);                 // 0x05
 
  set_reg(0x3c, 0x27);    // 0xbc
  delayMicroseconds(10);
  set_reg(0x22, 0x0a);    // 0xa2
  delayMicroseconds(10);
  set_reg(0x21, 0x01);    // 0xa1
  delayMicroseconds(10);
  set_reg(0x3c, 0x32);    // 0xbc
  delayMicroseconds(10);
  set_reg(0x23, 0x20);    // 0xa3
  delayMicroseconds(10);
  set_reg(0x3c, 0x05);    // 0xbc
  delayMicroseconds(10);
  set_reg(0x37, 0xb9);    // 0xb7

  delayMicroseconds(100);

  motion = 0;

  // quadrature outputs
  pinMode(QXA, OUTPUT);
  pinMode(QXB, OUTPUT);
  pinMode(QYA, OUTPUT);
  pinMode(QYB, OUTPUT);

  digitalWrite(QXA, LOW);
  digitalWrite(QXB, LOW);
  digitalWrite(QYA, LOW);
  digitalWrite(QYB, LOW);
  
  quad_x = 0;
  quad_y = 0;

  delta_x = 0;
  delta_y = 0;

  // quadrature input for scroll roll
  pinMode(QRA, INPUT_PULLUP);
  pinMode(QRB, INPUT_PULLUP);
  pinMode(MMB, INPUT_PULLUP); // set it as output only when we need to pull it down

  pinMode(LMB, OUTPUT); // connected via diodes to corresponding pins in DB9
  pinMode(RMB, OUTPUT); // connected via diodes to corresponding pins in DB9

//  SW_state = (digitalRead(QRA) << 1) + digitalRead(QRB);
  SW_state = (P2IN >> 6) & 0x03;

  mmb_trigger = 0;
  attachInterrupt(MMB, mmb_falling, FALLING);

  // set motion pin as interrupt input FALLING
  attachInterrupt(MOTION_PIN, set_motion, FALLING);

  // setting up ADC on pin P1_3
//  pinMode(MMBUTT, INPUT_PULLUP); // input from left mouse button switch
  ADC10CTL1 = CONSEQ_2 | INCH_3 | ADC10DIV_7;            // Repeat single channel, A3
  ADC10CTL0 = SREF_0 + ADC10SHT_2 + MSC + ADC10ON + ADC10IE; // Sample & Hold Time + ADC10 ON + Interrupt Enable

  ADC10DTC1 = 0x10;                 // 16 conversions
  ADC10DTC0 |= ADC10CT;                 // continuous transfer
  ADC10AE0 |= BIT3;                 // P1.3 ADC option select

  ADC10CTL0 &= ~ENC;        // Disable Conversion
  while (ADC10CTL1 & BUSY);   // Wait if ADC10 busy
  ADC10SA = (int)adc;       // Transfers data to next array (DTC auto increments address)
  ADC10CTL0 |= ENC + ADC10SC;   // Enable Conversion and conversion start

}

unsigned int reg_val, change_period_x, change_period_y;
unsigned long mnow, last_change_x, last_change_y;
unsigned int delta_x_raw, delta_y_raw, delta_xy_raw;
const unsigned int quad_state[] = { 0, 1, 3, 2 };
unsigned char output_sweep = 0;
unsigned long last_update;
volatile byte button_state;

void loop() {
  if(motion) { //|| ((millis() - last_update) > 250)) {

//    last_update = millis();

#if 0
//    get_reg(REG_OBSERVATION); // 0x2e
//    reg_val = get_reg(REG_MOTION); // 0x02
    delta_x_raw = get_reg(REG_DELTA_X); // 0x03
    delta_y_raw = get_reg(REG_DELTA_Y); // 0x04
    delta_xy_raw = get_reg(REG_DELTA_XY_H); // 0x05
#else

    get_burst();

#if DEBUG
    get_reg(REG_SQUAL); //    0x06
    get_reg(REG_MAX_PIXEL); //    0x09
    get_reg(REG_PIXEL_SUM); //    0x0a
    get_reg(REG_MIN_PIXEL); //    0x0b
#endif // DEBUG

#endif

    delta_x_raw |= (delta_xy_raw >> 4) << 8;
    delta_y_raw |= (0x0F & delta_xy_raw) << 8;

    if(delta_x_raw < 0x800) {
      delta_x += delta_x_raw;
    } else {
      delta_x += ((signed int)delta_x_raw - 0x1000);
    }

    if(delta_y_raw < 0x800) {
      delta_y += delta_y_raw;
    } else {
      delta_y += ((signed int)delta_y_raw - 0x1000);
    }

#ifdef GPIO_OUT_SET_SUB
    GPIO_OUT_CLR_SUB(1, 0);
#else
    digitalWrite(RED_LED, LOW);
#endif

    if(motion)
      --motion;

    change_period_x = constrain(1000 / abs(delta_x), 5, 2000);
    change_period_y = constrain(1000 / abs(delta_y), 5, 2000);

  } else {
  
//    if((delta_x != 0) || (delta_y != 0))
//      delayMicroseconds(min(change_period_x, change_period_y));
//    else
    {
      SW_state &= 0x3;
      SW_state <<= 2;
//      SW_state += (digitalRead(QRB) << 1) + digitalRead(QRA);
      SW_state |= (P2IN >> 6) & 0x03;


//             >>> increase
//         _______     _______
// QRA  ___|     |_____|     |___
//            _______     _______
// QRB  ______|     |_____|
//       00 01 11 10 00 01 11 10

      switch(SW_state) {
        // 0001, 0111, 1110, 1000
        case 0x1:
        case 0x7:
        case 0xE:
        case 0x8:
          ++scroll_change;
        break;
        // 0100, 1101, 1011, 0010
        case 0x4:
        case 0xD:
        case 0xB:
        case 0x2:
          --scroll_change;
        break;
        default:
        // 0000, 0101, 1111, 1010 -> no change
        break;
      }
    }
  }

  mnow = micros();

  if((delta_x != 0) && ((mnow - last_change_x) > change_period_x)) {
    last_change_x = mnow;
    if(delta_x > 0) {
      ++quad_x;
      --delta_x;
    } else {
      --quad_x;
      ++delta_x;
    }
    quad_x &= 0x03;
#ifdef GPIO_OUT_SET_SUB
    (quad_state[quad_x] & 0x01)?GPIO_OUT_SET_SUB(2, 1):GPIO_OUT_CLR_SUB(2, 1);
    (quad_state[quad_x] & 0x02)?GPIO_OUT_SET_SUB(2, 2):GPIO_OUT_CLR_SUB(2, 2);
#else
    digitalWrite(QXA, (quad_state[quad_x] & 0x01)?HIGH:LOW);
    digitalWrite(QXB, (quad_state[quad_x] & 0x02)?HIGH:LOW);
#endif
  }

  if((delta_y != 0) && ((mnow - last_change_y) > change_period_y)) {
    last_change_y = mnow;
    if(delta_y > 0) {
      ++quad_y;
      --delta_y;
    } else {
      --quad_y;
      ++delta_y;
    }
    quad_y &= 0x03;
#ifdef GPIO_OUT_SET_SUB
    (quad_state[quad_y] & 0x01)?GPIO_OUT_SET_SUB(2, 3):GPIO_OUT_CLR_SUB(2, 3);
    (quad_state[quad_y] & 0x02)?GPIO_OUT_SET_SUB(2, 4):GPIO_OUT_CLR_SUB(2, 4);
#else
    digitalWrite(QYA, (quad_state[quad_y] & 0x01)?HIGH:LOW);
    digitalWrite(QYB, (quad_state[quad_y] & 0x02)?HIGH:LOW);
#endif
  }

  adc_avg = 1024 - ((adc[0]+adc[1]+adc[2]+adc[3]+adc[4]+adc[5]+adc[6]+adc[7]+adc[8]+adc[9]+adc[10]+adc[11]+adc[12]+adc[13]+adc[14]+adc[15]) / 16);

// 000 - 0
#define THR1 55
// 001 - 109 ... 113
#define THR2 122
// 010 - 179 ... 183
#define THR3 220
// 011 - 257 ... 267
#define THR4 300
// 100 - 333 ... 336
#define THR5 364
// 101 - 392 ... 394
#define THR6 412
// 110 - 430 ... 434
#define THR7 440
// 111 - 446 ... 462

//       7654 3210
//000 -> 0000 0000 
//001 -> 0010 0000 
//010 -> 0001 0000 
//011 -> 0011 0000 
//100 -> 0000 0010 
//101 -> 0010 0010 
//110 -> 0001 0010 
//111 -> 0011 0010 

//  buttons = 7;
  if(adc_avg > THR7) {
//    buttons = 0; //0;
    button_state = 0;
  } else {
    if(adc_avg > THR6) {
//      buttons = 1; //0x20; //1;
      button_state = BIT5;
    } else {
      if(adc_avg > THR5) {
//        buttons = 2; //0x10; //2;
        button_state = BIT1;
      } else {
        if(adc_avg > THR4) {
//          buttons = 3; //0x30; //3;
          button_state = BIT1 | BIT5;
        } else {
          if(adc_avg > THR3) {
//            buttons = 4; //0x02; //4;
            button_state = BIT4 | BIT1;
          } else {
            if(adc_avg > THR2) {
//              buttons = 5; //0x22; //5;
              button_state = BIT4 | BIT5;
            } else {
              if(adc_avg > THR1) {
//                buttons = 6; //0x12; //6;
                button_state = BIT4 | BIT1;
              } else {
//                buttons = 7; //0x32; //7;
                button_state = BIT4 | BIT1 | BIT5;
              }
            }
          }
        }
      }
    }
  }
//  buttons ^= 0xff;
//  buttons &= 0x32;
//                      BIT4                     BIT1               BIT5
//  button_state = ((buttons & BIT2) << 2) | (buttons & BIT1) | ((buttons & BIT0) << 5);
}

void set_reg(int address, int value) {
  // take the SS pin low to select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_CLR_SUB(1, 2);
#else
  digitalWrite(NCS,LOW);
#endif
  //  send in the address and value via SPI:
  SPI.transfer(0x80 | address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(1, 2);
#else
  digitalWrite(NCS,HIGH); 
#endif
}

int get_reg(int address) {
  unsigned int value = 0xFF;
  // take the SS pin low to select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_CLR_SUB(1, 2);
#else
  digitalWrite(NCS,LOW);
#endif
  //  send in the address and value via SPI:
  SPI.transfer(address);
  value = SPI.transfer(0xFF);
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(1, 2);
#else
  digitalWrite(NCS,HIGH); 
#endif
  return value;
}

void get_burst() {
  unsigned int value = 0xFF;
  // take the SS pin low to select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_CLR_SUB(1, 2);
#else
  digitalWrite(NCS,LOW);
#endif
  //  send in the address and value via SPI:
  SPI.transfer(REG_MBURST);
  // tSRAD
  delayMicroseconds(4);
  value = SPI.transfer(0xFF); // MOTION 0x02
//  delayMicroseconds(1);
  delta_x_raw = SPI.transfer(0xFF); // REG_DELTA_X 0x03
  delta_y_raw = SPI.transfer(0xFF); // REG_DELTA_Y 0x04
  delta_xy_raw = SPI.transfer(0xFF); // REG_DELTA_XY_H 0x05
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(1, 2);
#else
  digitalWrite(NCS,HIGH); 
#endif
}

void set_motion() {
  ++motion;
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(1, 0);
#else
  digitalWrite(RED_LED, HIGH);
#endif
}

volatile byte quad_raw_out, test;

//P2_0 RMB
//P2_1 QXA
//P2_2 QXB
//P2_3 QYA
//P2_4 QYB
//P2_5 LMB

void mmb_falling() {

  quad_raw_out = P2OUT;

  if(scroll_change != 0) {
#ifdef COCOLINO
    if(scroll_change < 0) {
      P2OUT =  BIT1 | BIT6 | BIT7 | (button_state & BIT5); //QXA
      ++scroll_change;
    } else {
      P2OUT = BIT6 | BIT7 | (button_state & BIT5); // RMB
      --scroll_change;
    }
#else // EZMOUSE
    //         RMB clear
    if(scroll_change < 0) {
      P2OUT =  BIT1 | BIT5 | BIT6 | BIT7; // | ((buttons & BIT1)?BIT5:0); // QXA
      ++scroll_change;
    } else  {
      P2OUT =  BIT4 | BIT5 | BIT6 | BIT7; // | ((buttons & BIT1)?BIT5:0); // QYB
      --scroll_change;
    }
#endif
  } else {
    P2OUT =  BIT0 | button_state | BIT6 | BIT7 | (quad_raw_out & (BIT2 | BIT3));

    // middle button state MMB
//  if(buttons & BIT0) // P1IN & BIT3) {
//    P2OUT |= BIT5; // LMB

  // 4th QXA
//  if(buttons & BIT1) //button_state_4th)
//    P2OUT |= BIT1; // 4th

  // 5th QYB
//  if(buttons & BIT2) //button_state_5th)
//    P2OUT |= BIT4; // 5th
}

#ifdef COCOLINO

  // total time of MMB low: 64...65us
  // excluding ISR reaction: 32us ... 33 (45us) 

  delayMicroseconds(47);
#else // EZMOUSE
  delayMicroseconds(30);
#endif

//  while(!(P1IN & BIT4));

//  pinMode(MMB, INPUT_PULLUP);
  P2OUT = quad_raw_out;

  mmb_trigger = 1;
}
