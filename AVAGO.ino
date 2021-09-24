/*
  AVAGO ADNS-7550

 The circuit:

 // Default SPI pinout for MSP430G2553
// MISO P1.7
// MOSI P1.6
// CLK  P1.5
// set pin 8 as the slave select for the digital pot:
// CS   P2.0

           ______________
      Vcc  |            | GND
LED0  P1_0 |            | P2_6   QRA
LMB   P1_1 |            | P2_7   QRB
RMB   P1_2 |            | TEST
IRQ   P1_3 |            | RESET
MMB   P1_4 |            | P1_7   MISO
CLK   P1_5 |            | P1_6   MOSI
CS    P2_0 |            | P2_5   BUTT
QXA   P2_1 |            | P2_4   QYA
QXB   P2_2 |____________| P2_3   QYB

mouse pinout:
     X2  X1  Y1  Y2  MMB
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
  0   0   0   0   1   FOURTH down
WiP                   FIVETH down  
==================
mm https://github.com/paulroberthill/AmigaPS2Mouse
26us ... 32us
WiP
==================

*/

//#define DEBUG 1

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

const int slaveSelectPin = SS;

// MOTION pin set to trigger IRQ P1_3
#define MOTION_PIN PUSH2

// quadrature inputs
#define QRA P2_6
#define QRB P2_7
#define LMB P1_1
#define MMB P1_4
#define RMB P1_2
#define WHEEL_DELAY 5000

// quadrature outputs
#define QXA P2_1
#define QXB P2_2
#define QYA P2_3
#define QYB P2_4

//RED_LED
#define LED0 P1_0
//#define LED1 P1_1
//#define LED2 P1_2
//#define BUTT P2_5


// approx 5us instead of 13us
#define GPIO_OUT_SET_SUB(port, pin) (P##port##OUT |=  (1<<pin))
#define GPIO_OUT_CLR_SUB(port, pin) (P##port##OUT &= ~(1<<pin))

#define MHZ 12
#define SET_CPU_CLOCK(mhz) { DCOCTL = CALDCO_##mhz##MHZ; BCSCTL1 = CALBC1_##mhz##MHZ; };

unsigned int motion = 200, k = 0;
unsigned int quad_x, quad_y;
signed delta_x, delta_y;

volatile unsigned char SW_state;
volatile signed int scroll_change = 0;

void setup() {

// set CPU clock 1, 8, 12 or 16 MHz
//  DCOCTL = CALDCO_8MHZ;
//  BCSCTL1 = CALBC1_8MHZ;
  SET_CPU_CLOCK(12);

  // set the slaveSelectPin as an output:
  pinMode (slaveSelectPin, OUTPUT);

  // configure RED LED for output
  pinMode(LED0, OUTPUT);
//  pinMode(LED1, OUTPUT);
//  pinMode(LED2, OUTPUT);
//  pinMode(BUTT, INPUT_PULLUP);

//  attachInterrupt(BUTT, set_sens_mode, FALLING);

  // initialize SPI:
  SPI.begin(); 
  SPI.setClockDivider(MHZ/2); // 1MHz SPI

//  Serial.begin(9600);
//  Serial.println("AVAGO SPI demo.");

  delayMicroseconds(250);

  digitalWrite(slaveSelectPin,LOW);
  delayMicroseconds(25);
  digitalWrite(slaveSelectPin,HIGH);
  delayMicroseconds(25);

#if DEBUG
  get_reg(REG_PRODUCT_ID); // 0x00 -> 0x32
  get_reg(REG_INV_PRODUCT_ID); // 0x3e -> 0xfc
  get_reg(REG_REVISION_ID); // 0x01 -> 0x03
  get_reg(REG_INV_REVISION_ID); // 0x3f -> 0xcd
#endif // DEBUG

//  delayMicroseconds(250);

  set_reg(REG_POWER_UP_RESET, 0x5a); // (0x80 | 0x3a = 0xba)

/* LASER_3MA LASER_5MA LASER_10MA */
  set_reg(REG_LASER_CTRL0, (0xC0) & LASER_RANGE); // 0x1a
  set_reg(REG_LASER_CTRL1, (0xC0) & ~LASER_RANGE); // 0x1f
  set_reg(REG_LSRPWR_CFG0, LASER_POWER); // 0x1c
  set_reg(REG_LSRPWR_CFG1, ~LASER_POWER); // 0x1d

/* CONFIG2_400CPI, CONFIG2_800CPI, CONFIG2_1200CPI, CONFIG2_1600CPI */
  set_reg(REG_CONFIGURATION2, CONFIG2_400CPI); //   0x12

  // wait for at least one frame ?
  delayMicroseconds(250);

  // clear observation register
  set_reg(REG_OBSERVATION, 0x00);

  // wait for at least one frame
  delayMicroseconds(250);
  // and check observation register, all bits 0-3 must be set
  while((get_reg(REG_OBSERVATION) & 0x0F) != 0x0F) {
    delayMicroseconds(250);
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

  // set motion pin as interrupt input FALLING
  pinMode(MOTION_PIN, INPUT_PULLUP);
  attachInterrupt(MOTION_PIN, set_motion, FALLING);

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
//  pinMode(MMB, OUTPUT); // set it as output only when we need to pull it down

//  pinMode(LMB, INPUT_PULLUP);
//  pinMode(RMB, INPUT_PULLUP);
  pinMode(LMB, OUTPUT);
  pinMode(RMB, OUTPUT);
//  P1DIR |= BIT2;
//  P1SEL &= ~BIT2;
//  P1SEL2 &= ~BIT2;

  SW_state = (digitalRead(QRA) << 1) + digitalRead(QRB);

  attachInterrupt(MMB, mmb_falling, FALLING);
}

unsigned int reg_val, change_period;
unsigned int delta_x_raw, delta_y_raw, delta_xy_raw;
const unsigned int quad_state[] = { 0, 1, 3, 2 };

unsigned char output_sweep = 0;

void loop() {
  if(motion) {
#ifdef GPIO_OUT_SET_SUB
      GPIO_OUT_SET_SUB(1, 0);
#else
    digitalWrite(RED_LED, HIGH);
#endif

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

    --motion;

  // for 16MHz 1650
  // for 8MHz  800
    change_period = constrain(100 * MHZ / max(abs(delta_x),abs(delta_y)), 5, 100 * MHZ);
  } else {
    if((delta_x != 0) || (delta_y != 0))
      delayMicroseconds(change_period);
    else {
      SW_state &= 0x3;
      SW_state <<= 2;
      SW_state += (digitalRead(QRB) << 1) + digitalRead(QRA);

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

  if(delta_x != 0) {
    if(delta_x > 0) {
      ++quad_x;
      --delta_x;
    } else {
      --quad_x;
      ++delta_x;
    }
    quad_x &= 0x03;
#ifdef GPIO_OUT_SET_SUB
    if(quad_state[quad_x] & 0x01)
      GPIO_OUT_SET_SUB(2, 1);
    else
      GPIO_OUT_CLR_SUB(2, 1);
    if(quad_state[quad_x] & 0x02)
      GPIO_OUT_SET_SUB(2, 2);
    else
      GPIO_OUT_CLR_SUB(2, 2);
#else
    digitalWrite(QXA, (quad_state[quad_x] & 0x01)?HIGH:LOW);
    digitalWrite(QXB, (quad_state[quad_x] & 0x02)?HIGH:LOW);
#endif
  }

  if(delta_y != 0) {
    if(delta_y > 0) {
      ++quad_y;
      --delta_y;
    } else {
      --quad_y;
      ++delta_y;
    }
    quad_y &= 0x03;
#ifdef GPIO_OUT_SET_SUB
    if(quad_state[quad_y] & 0x01)
      GPIO_OUT_SET_SUB(2, 3);
    else
      GPIO_OUT_CLR_SUB(2, 3);
    if(quad_state[quad_y] & 0x02)
      GPIO_OUT_SET_SUB(2, 4);
    else
      GPIO_OUT_CLR_SUB(2, 4);
#else
    digitalWrite(QYA, (quad_state[quad_y] & 0x01)?HIGH:LOW);
    digitalWrite(QYB, (quad_state[quad_y] & 0x02)?HIGH:LOW);
#endif
  }

}

void set_reg(int address, int value) {
  // take the SS pin low to select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_CLR_SUB(2, 0);
#else
  digitalWrite(slaveSelectPin,LOW);
#endif
  //  send in the address and value via SPI:
  SPI.transfer(0x80 | address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(2, 0);
#else
  digitalWrite(slaveSelectPin,HIGH); 
#endif
}

int get_reg(int address) {
  unsigned int value = 0xFF;
  // take the SS pin low to select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_CLR_SUB(2, 0);
#else
  digitalWrite(slaveSelectPin,LOW);
#endif
  //  send in the address and value via SPI:
  SPI.transfer(address);
  value = SPI.transfer(0xFF);
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(2, 0);
#else
  digitalWrite(slaveSelectPin,HIGH); 
#endif
  return value;
}

void get_burst() {
  unsigned int value = 0xFF;
  // take the SS pin low to select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_CLR_SUB(2, 0);
#else
  digitalWrite(slaveSelectPin,LOW);
#endif
  //  send in the address and value via SPI:
  SPI.transfer(REG_MBURST);
  delayMicroseconds(MHZ / 4);
  value = SPI.transfer(0xFF); // MOTION 0x02
  delta_x_raw = SPI.transfer(0xFF); // REG_DELTA_X 0x03
  delta_y_raw = SPI.transfer(0xFF); // REG_DELTA_Y 0x04
  delta_xy_raw = SPI.transfer(0xFF); // REG_DELTA_XY_H 0x05
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(2, 0);
#else
  digitalWrite(slaveSelectPin,HIGH); 
#endif
}

volatile byte sens_mode = 0;

void set_sens_mode() {
  switch(++sens_mode) {
    default:
      sens_mode = 0;
      analogWrite(LED0, 255);
//      analogWrite(LED1, 0);
//      analogWrite(LED2, 0);
      break;
    case 1:
      analogWrite(LED0, 0);
//      analogWrite(LED1, 255);
//      analogWrite(LED2, 0);
      break;
    case 2:
      analogWrite(LED0, 0);
//      analogWrite(LED1, 0);
//      analogWrite(LED2, 255);
      break;
  }
}

void set_motion() {
  ++motion;
}
volatile byte quad_raw_out;

void mmb_falling() {

  quad_raw_out = P2OUT;
  P2OUT &= ~(BIT2 | BIT3 | BIT4);

  if(scroll_change < 0) {
    P2OUT &= ~(BIT2 | BIT3 | BIT4);
    P2OUT |= BIT1;
    // RMB clear
    P1OUT &= ~BIT2;
    ++scroll_change;
  }

  if(scroll_change > 0) {
    // pull RMB for 20us
    P2OUT &= ~(BIT1 | BIT2 | BIT3 | BIT4);
    // RMB clear
    P1OUT &= ~BIT2;
    --scroll_change;
  }

  delayMicroseconds(30);

  pinMode(MMB, INPUT_PULLUP);
  P1OUT |= BIT1 | BIT2;
  P2OUT |= quad_raw_out;
}
