/*
  AVAGO ADNS-7550

 The circuit:

 // Default SPI pinout for MSP430G2553
 either use 3V from ADNS and clock MSP with 12MHz or use 3v6 regulator and use default 16MHz
// MISO P1.7
// MOSI P1.6
// CLK  P1.5
// CS   P1.2

                                 ______________
                            Vcc  | 1       20 | GND                DB9 8
             3b/ADC input   P1_0 |            | P2_6   QRA
               MOTION/IRQ   P1_1 |            | P2_7   QRB
                      CS    P1_2 |            | TEST
        DB9 5         MMB   P1_3 | 5          | RESET
                mmb-in      P1_4 |            | P1_7   MOSI
                      CLK   P1_5 |            | P1_6   MISO
        DB9 9        !RMB   P2_0 |            | P2_5   LMB         DB9 6
   XQ   DB9 4         QXA   P2_1 |            | P2_4   QYA         DB9 3 YQ
    X   DB9 2         QXB   P2_2 |_10______11_| P2_3   QYB         DB9 1  Y

mouse pinout DB9:
     QYB QXB QYA QXA
      Y   X  YQ  XQ  MMB     DB9   color   MCU
      U   D   L   R  PotY      1   red     11
   _______________________     2   blk     10
   \  1   2   3   4   5  /     3   gry     12
    \                   /      4   org      9
     \__6___7___8___9_ /       5   brw      5
       LMB  +  gnd RMB         6   grn    LMB -|<|- 13 = low VF diode
                   PotX        7   wht     +5vcc 
                               8   blu    GND
                               9   ylw      8 to gate of MOSFET, RMB pulled down when high

*/

//#define DEBUG 1

// wheel protocol used (use appropriate driver on host)
//#define DRIVER_COCOLINO
//#define DRIVER_EZMOUSE
#define DRIVER_BLABBER

#ifdef DRIVER_COCOLINO
  // total time of MMB low: 64...65us
  // excluding ISR reaction: 32us ... 33 (45us) 
#define USE_FIXED_DELAY 47
#endif
#ifdef DRIVER_EZMOUSE
#define USE_FIXED_DELAY 30
#endif
#ifdef DRIVER_BLABBER
// 25 lines in VBR interrupt routine corresponds to approx 50us pulse
// IRQ reacts approx 18..20us after falling edge
//#define USE_FIXED_DELAY 35
#define USE_FIXED_DELAY 45
#endif

#define  REG_PRODUCT_ID       0x00
#define  REG_INV_PRODUCT_ID   0x3E
#define  REG_REVISION_ID      0x01
#define  REG_INV_REVISION_ID  0x3F

#define  REG_MOTION           0x02
#define  REG_MOTION_MOT       0x80
#define  REG_MOTION_PIXRDY    0x40
#define  REG_MOTION_PIXFIRST  0x20
#define  REG_MOTION_OVF       0x10
#define  REG_MOTION_LP_VALID  0x08
#define  REG_MOTION_FAULT     0x04

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

#define  LASER_RANGE      LASER_10MA
/* 0x00 -> 33.6%, 0xff -> 100%*/
//#define  LASER_POWER      0xB5
#define  LASER_POWER      0xC2

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
#define MMB P1_3
#define MMB_IN P1_4
#define RMB P2_0
#define WHEEL_DELAY 5000

// quadrature outputs
#define QXA P2_1
#define QXB P2_2
#define QYA P2_3
#define QYB P2_4

#define MMBUTT PUSH2

// approx 5us instead of 13us
#define GPIO_OUT_SET_SUB(port, pin) (P##port##OUT |=  (1<<pin))
#define GPIO_OUT_CLR_SUB(port, pin) (P##port##OUT &= ~(1<<pin))

#define MHZ 12
#define SET_CPU_CLOCK(mhz) { DCOCTL = CALDCO_##mhz##MHZ; BCSCTL1 = CALBC1_##mhz##MHZ; };

const unsigned int quad_state[] = { 0, 1, 3, 2 };
unsigned int motion = 200, k = 0, adc_avg;
unsigned int quad_x, quad_y, t;
signed delta_x, delta_y;
unsigned int sensor_resolution;

volatile unsigned char SW_state;
volatile signed int scroll_change = 0;
volatile unsigned int mmb_trigger;
bool mmb_attached_irq;
int adc[16] = {0}; //Sets up an array of 16 integers and zero's the values
int max_adc, min_adc;

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
  // tSRAD
  delayMicroseconds(4);
  value = SPI.transfer(0xFF);
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(1, 2);
#else
  digitalWrite(NCS,HIGH); 
#endif
  return value;
}

void setup() {

  // set the NCS as an output:
  pinMode(NCS, OUTPUT);

  pinMode(MOTION_PIN, INPUT_PULLUP);

  // initialize SPI:
  SPI.begin(); 
  SPI.setClockDivider(24); // 1MHz SPI

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
  sensor_resolution = 0;
  set_reg(REG_CONFIGURATION2, CONFIG2_400CPI | (sensor_resolution << 5)); //   0x12 (0x92)

  // wait for at least one frame ?
  delayMicroseconds(250);

  // clear observation register
  set_reg(REG_OBSERVATION, 0x00);

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

  // wait for at least one frame
  delayMicroseconds(250);
  // and check observation register, all bits 0-3 must be set
  while((get_reg(REG_OBSERVATION) & 0x0F) != 0x0F) {
    delayMicroseconds(5000);
    ++motion;
    if(motion > 40) {
      motion = 0;
    } else {
      if(motion > 30) {
        ++quad_x;
        --quad_y;
      } else {
        if(motion > 20) {
          --quad_x;
          --quad_y;
        } else {
          if(motion > 10) {
            --quad_x;
            ++quad_y;
          } else {
            ++quad_x;
            ++quad_y;
          }
        }
      }
    }
    quad_x &= 0x03;
    (quad_state[quad_x] & 0x01)?GPIO_OUT_SET_SUB(2, 1):GPIO_OUT_CLR_SUB(2, 1);
    (quad_state[quad_x] & 0x02)?GPIO_OUT_SET_SUB(2, 2):GPIO_OUT_CLR_SUB(2, 2);
    quad_y &= 0x03;
    (quad_state[quad_y] & 0x01)?GPIO_OUT_SET_SUB(2, 3):GPIO_OUT_CLR_SUB(2, 3);
    (quad_state[quad_y] & 0x02)?GPIO_OUT_SET_SUB(2, 4):GPIO_OUT_CLR_SUB(2, 4);
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

  delta_x = 0;
  delta_y = 0;

  // quadrature input for scroll roll
  pinMode(QRA, INPUT_PULLUP);
  pinMode(QRB, INPUT_PULLUP);
  pinMode(MMB, INPUT_PULLUP); // set it as output only when we need to pull it down
  pinMode(MMB_IN, INPUT_PULLUP);

  pinMode(LMB, OUTPUT); // connected via diodes to corresponding pins in DB9
  pinMode(RMB, OUTPUT); // connected via diodes to corresponding pins in DB9
//  pinMode(RMB, INPUT_PULLUP); // connected via diodes to corresponding pins in DB9

  digitalWrite(LMB, HIGH);
//  digitalWrite(RMB, HIGH);
  digitalWrite(RMB, LOW);

//  SW_state = (digitalRead(QRA) << 1) + digitalRead(QRB);
  SW_state = (P2IN >> 6) & 0x03;

  mmb_trigger = 0;
  attachInterrupt(MMB, mmb_falling, FALLING);
  mmb_attached_irq = 1;

  // set motion pin as interrupt input FALLING
  attachInterrupt(MOTION_PIN, set_motion, FALLING);

  // setting up ADC on pin P1_0
  ADC10CTL1 = CONSEQ_2 | INCH_0 | ADC10DIV_7;            // Repeat single channel, A3
  ADC10CTL0 = SREF_0 + ADC10SHT_2 + MSC + ADC10ON + ADC10IE; // Sample & Hold Time + ADC10 ON + Interrupt Enable

  ADC10DTC1 = 0x10;                 // 16 conversions
  ADC10DTC0 |= ADC10CT;             // continuous transfer
  ADC10AE0 |= BIT0;                 // P1.0 ADC option select

  ADC10CTL0 &= ~ENC;                // Disable Conversion
  while (ADC10CTL1 & BUSY);         // Wait if ADC10 busy
  ADC10SA = (int)adc;               // Transfers data to next array (DTC auto increments address)
  ADC10CTL0 |= ENC + ADC10SC;       // Enable Conversion and conversion start
}

unsigned int reg_val;
long change_period_x, change_period_y, change_period_lapsed_x, change_period_lapsed_y;
unsigned long last_change_z, mnow;
unsigned int delta_x_raw, delta_y_raw, delta_xy_raw;

//             >>> increment
//         _______     _______
//   A  ___|     |_____|     |___
//            _______     _______
//   B  ______|     |_____|
//       00 01 11 10 00 01 11 10

unsigned char output_sweep = 0;
unsigned long last_update;
volatile byte button_state, prev_button_state, button_update;
unsigned int motion_status;

unsigned int get_burst(bool get_all = true) {
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
  if(get_all || (value & REG_MOTION_MOT)) {
    delta_x_raw = SPI.transfer(0xFF); // REG_DELTA_X 0x03
    delta_y_raw = SPI.transfer(0xFF); // REG_DELTA_Y 0x04
    delta_xy_raw = SPI.transfer(0xFF); // REG_DELTA_XY_H 0x05
  } else {
    // no change so no need to retrieve registers
    delta_x_raw = delta_y_raw = delta_xy_raw = 0x00;
  }
  // take the SS pin high to de-select the chip:
#ifdef GPIO_OUT_SET_SUB
  GPIO_OUT_SET_SUB(1, 2);
#else
  digitalWrite(NCS,HIGH); 
#endif

  return value;
}

//volatile byte fake_code;
unsigned long mmb_last_trigger;
volatile byte mmb_prev_state;

void loop() {
  if((motion) || ((mmb_trigger > 256) && ((millis() - mmb_last_trigger) > 10))) {

    if(mmb_trigger > 16)
      mmb_trigger = 1;

    if(motion)
      mmb_last_trigger = millis();
#if 0
//    get_reg(REG_OBSERVATION); // 0x2e
//    reg_val = get_reg(REG_MOTION); // 0x02
    delta_x_raw = get_reg(REG_DELTA_X); // 0x03
    delta_y_raw = get_reg(REG_DELTA_Y); // 0x04
    delta_xy_raw = get_reg(REG_DELTA_XY_H); // 0x05
#else

    motion_status = get_burst(false);

    while((motion_status == 0xFF) && \
          (delta_xy_raw == 0xFF) && \
          (delta_y_raw == 0xFF) && \
          (delta_x_raw == 0xFF)) {
        delay(2);
        motion_status = get_burst(false);
    }

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

    if(motion)
      --motion;

    if(motion_status & REG_MOTION_MOT) {
      if(abs(delta_x) > 1)
        change_period_x = constrain(133 / (abs(delta_x)), 1, 45);
      else
        change_period_x = 1;
      if(abs(delta_y) > 1)
        change_period_y = constrain(133 / (abs(delta_y)), 1, 45);
      else
        change_period_y = 1;
      change_period_lapsed_x = 0;
      change_period_lapsed_y = 0;
    }

  }

  mnow = millis();
  if(((mnow - last_change_z) > 2)) {
    last_change_z = mnow;

    SW_state &= 0x3;
    SW_state <<= 2;
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
//        ++fake_code;
      break;
      // 0100, 1101, 1011, 0010
      case 0x4:
      case 0xD:
      case 0xB:
      case 0x2:
        --scroll_change;
//        --fake_code;
      break;
      default:
      // 0000, 0101, 1111, 1010 -> no change
      // 0011, 0110, 0110, 1100 -> invalid transitions
      break;
    }
//    fake_code &= 0x3F;
  }

  if((delta_x != 0) && (!change_period_lapsed_x)) {
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
    change_period_lapsed_x = change_period_x;
  }
  --change_period_lapsed_x;

  if((delta_y != 0) && (!change_period_lapsed_y)) {
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
    change_period_lapsed_y = change_period_y;
  }
  --change_period_lapsed_y;

  // just in case there is still IRQ pending
  if((delta_x == 0) && (delta_y == 0) && (!(P1IN & BIT1))) {
    ++motion;
  }

  adc_avg = 1024 - ((adc[0]+adc[1]+adc[2]+adc[3]+adc[4]+adc[5]+adc[6]+adc[7]+adc[8]+adc[9]+adc[10]+adc[11]+adc[12]+adc[13]+adc[14]+adc[15]) / 16);

  max_adc = 0; min_adc = 1023;
  for(k = 0; k < 10; ++k) {
    max_adc = max(max_adc, adc[k]);
    min_adc = min(min_adc, adc[k]);
  }

  if((max_adc - min_adc) < 10) {
// 000 - 0
#define THR1 55
// 001 - 109 ... 113
#define THR2 146
// 010 - 179 ... 183
#define THR3 220
// 011 - 257 ... 267
#define THR4 298
// 100 - 328 ... 331
#define THR5 356
// 101 - 380 ... 383
#define THR6 399
// 110 - 415 ... 420
#define THR7 439
// 111 - 457 ... 459

// 000 - 0
// 001 - 109 ... 113  -> 4
//           66
// 010 - 179 ... 183  -> 4
//           74
// 011 - 257 ... 267  -> 10
//           61
// 100 - 328 ... 331  -> 3
//           49
// 101 - 380 ... 383  -> 3
//           32
// 110 - 415 ... 420  -> 5
//           37
// 111 - 457 ... 459  -> 2

//       7654 3210
//000 -> 0000 0000
//001 -> 0010 0000
//010 -> 0001 0000
//011 -> 0011 0000
//100 -> 0000 0010
//101 -> 0010 0010
//110 -> 0001 0010
//111 -> 0011 0010

    button_state &= 0x1f;
    prev_button_state = button_state;
    button_state = 0;
    if(adc_avg > THR4) {
      button_state |= BIT2;
      if(adc_avg > THR6) {
        button_state |= BIT1;
        if(adc_avg > THR7)
          button_state |= BIT0;
      } else {
        if(adc_avg > THR5)
          button_state |= BIT0;
      }
    } else {
      if(adc_avg > THR2) {
        button_state |= BIT1;
        if(adc_avg > THR3)
          button_state |= BIT0;
      } else {
        if(adc_avg > THR1)
          button_state |= BIT0;
      }
    }

    button_state &= ~BIT4;
    button_state |= (P1IN & BIT4);

    if(prev_button_state ^ button_state) {
      button_update |= prev_button_state ^ button_state;
    }
    if(button_state & BIT5) {
      // do whatever should be done when top case button is pressed
      // modify sensitivity
      // flash LEDS to indicate, etc.
    }

  // ignore 
//  button_state &= ~BIT5;

//  buttons ^= 0xff;
//  buttons &= 0x32;
//                      BIT4                     BIT1               BIT5
//  button_state = ((buttons & BIT2) << 2) | (buttons & BIT1) | ((buttons & BIT0) << 5);

  }

  if(button_state & BIT2) {
    if(button_state & BIT1) {
      // change resolution to finer - button 4th
      if(sensor_resolution < 3) {
        ++sensor_resolution;
        sensor_resolution &= 0x03;
        set_reg(REG_CONFIGURATION2, CONFIG2_400CPI | (sensor_resolution << 5)); //   0x12 (0x92)
      }
      button_update &= ~BIT1;
      button_state &= ~BIT1;
    }
    if(button_state & BIT0) {
      // change resolution to coarser - button 5th
      if(sensor_resolution > 1) {
        --sensor_resolution;
        sensor_resolution &= 0x03;
        set_reg(REG_CONFIGURATION2, CONFIG2_400CPI | (sensor_resolution << 5)); //   0x12 (0x92)
      }
      button_update &= ~BIT0;
      button_state &= ~BIT0;
    }
  }
}

void set_motion() {
  ++motion;
}

volatile byte quad_raw_out, test;

//P2_0 RMB -> inverse logic
//P2_1 QXA XQ
//P2_2 QXB  X
//P2_3 QYA YQ
//P2_4 QYB  Y
//P2_5 LMB -> HIGH

//QXA QXB QYA QYB RMB
//  1   1   1   1   0   idle
//  0   x   x   x   0   MMB down
//  x   0   1   0   0   wheel up
//  x   1   0   0   0   wheel down
//  x   0   0   0   0   4th
//  x   0   0   1   0   5th

volatile byte code_send;

#define CODE_IDLE         0
#define CODE_WHEEL_UP     (0x0C << 1)
#define CODE_WHEEL_DOWN   (0x0E << 1)
#define CODE_WHEEL_LEFT   (0x0B << 1)
#define CODE_WHEEL_RIGHT  (0x09 << 1)
#define CODE_MMB_DOWN     (0x0F << 1)
#define CODE_MMB_UP       (0x0D << 1)
#define CODE_4TH_DOWN     (0x05 << 1)
#define CODE_4TH_UP       (0x07 << 1)
#define CODE_5TH_DOWN     (0x0A << 1)
#define CODE_5TH_UP       (0x06 << 1)

void mmb_falling() {

  quad_raw_out = P2OUT | BIT5;

#ifdef DRIVER_COCOLINO
  if(scroll_change != 0) {
    if(scroll_change < 0) {
      P2OUT =  BIT0 | BIT1 | BIT6 | BIT7 | ((P1IN & BIT4) << 1); //QXA
      ++scroll_change;
    } else {
      P2OUT = BIT0 |         BIT6 | BIT7 | ((P1IN & BIT4) << 1); // activate MOSFET on RMB
      --scroll_change;
    }
  } else { // 5th BIT1 -> QXA -> XQ pin 4   4th BIT4 -> QYB -> Y pin 3                                                    MMB BIT5 LMB pin 6
    P2OUT =  ((button_state & BIT0) << 1) | ((button_state & BIT1) << 3) | BIT6 | BIT7 | (quad_raw_out & (BIT2 | BIT3)) | ((P1IN & BIT4) << 1);
  }
#endif // COCOLINO

#ifdef DRIVER_EZMOUSE
  if(scroll_change != 0) {
    // RMB clear, LMB unchanged
    if(scroll_change < 0) {
      //             MMB -> BIT1 -> QXB
      P2OUT = BIT0 | ((P1IN & BIT4) >> 3) | BIT2 |        BIT5; // QXB
      ++scroll_change;
    } else  {
      //             MMB -> BIT1 -> QYA
      P2OUT = BIT0 | ((P1IN & BIT4) >> 3) |        BIT3 | BIT5; // QYA
      --scroll_change;
    }
  } else { //                                        MMB -> BIT3?
    P2OUT =  BIT0 | (quad_raw_out & (BIT2 | BIT3)) | ((P1IN & BIT4) << 1) | BIT6 | BIT7;
  }
#endif // EZMOUSE

#ifdef DRIVER_BLABBER
  code_send = CODE_IDLE;
  if(scroll_change != 0) {
    if(scroll_change < 0) {
      code_send = CODE_WHEEL_DOWN;
    } else  {
      code_send = CODE_WHEEL_UP;
    }
    if(button_state & BIT2)
      code_send ^= (BIT1 | BIT3);
    P2OUT = (quad_raw_out ^ code_send) | BIT5;
  } else {
    if((button_update & BIT4) || (mmb_prev_state ^ (P1IN & BIT4))) {
      if((P1IN & BIT4)) {
        mmb_prev_state = BIT4 ;//(P1IN & BIT4);
        code_send = CODE_MMB_UP;
      } else {
        mmb_prev_state = 0; //(P1IN & BIT4);
        code_send = CODE_MMB_DOWN;
      }
      P2OUT = (quad_raw_out ^ code_send) | BIT5;
    } else {
      if(button_update & BIT1) {
        // 4th - left side button
        if(button_state & BIT1)
          code_send = CODE_4TH_DOWN;
        else
          code_send = CODE_4TH_UP;
        P2OUT = (quad_raw_out ^ code_send) | BIT5;
      } else {
        if(button_update & BIT0) {
          // 5th button - right side button
          if(button_state & BIT0)
            code_send = CODE_5TH_DOWN;
          else
            code_send = CODE_5TH_UP;
          P2OUT = (quad_raw_out ^ code_send) | BIT5; // 0x0100
        }
      }
    }
  }
#endif // BLABBER

  // with code confirmation we must not wait for falling edge
  delayMicroseconds(USE_FIXED_DELAY);

  // make sure to switch off MOSFET on RMB
  P2OUT = quad_raw_out & ~BIT0;

  ++mmb_trigger;
  mmb_last_trigger = millis();

  if(!(P1IN & BIT3)) {
    switch(code_send)  {
      case CODE_WHEEL_UP:
      case CODE_WHEEL_RIGHT:
          --scroll_change;
          break;
      case CODE_WHEEL_DOWN:
      case CODE_WHEEL_LEFT:
          ++scroll_change;
          break;
      case CODE_MMB_DOWN:
      case CODE_MMB_UP:
          button_update &= ~BIT4;
          break;
      case CODE_4TH_DOWN:
      case CODE_4TH_UP:
          button_update &= ~BIT1;
          break;
      case CODE_5TH_DOWN:
      case CODE_5TH_UP:
          button_update &= ~BIT0;
          break;
    }
  }
}
