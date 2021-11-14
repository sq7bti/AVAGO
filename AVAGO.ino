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
               top button   P1_0 |            | P2_6   QRA
               MOTION/IRQ   P1_1 |            | P2_7   QRB
                      CS    P1_2 |            | TEST
        DB9 5         MMB   P1_3 | 5          | RESET
                mmb-in      P1_4 |            | P1_7   MOSI
                      CLK   P1_5 |            | P1_6   MISO
   XQ   DB9 4         QXA   P2_0 |            | P2_5   5th button
    X   DB9 2         QXB   P2_1 |            | P2_4   4th button
    Y   DB9 1         QYB   P2_2 |_10______11_| P2_3   QYA      DB9 3 YQ   

mouse pinout DB9:
     QYB QXB QYA QXA
      Y   X  YQ  XQ  MMB     DB9   color   MCU
      U   D   L   R  PotY      1   red     10
   _______________________     2   blk      0
   \  1   2   3   4   5  /     3   gry     11
    \                   /      4   org      8
     \__6___7___8___9_ /       5   brw      5
       LMB  +  gnd RMB         6   grn    LMB
                   PotX        7   wht  +5vcc
                               8   blu    GND
                               9   ylw    RMB
*/

//#define DEBUG 1

// 25 lines in VBR interrupt routine corresponds to approx 50us pulse
// IRQ reacts approx 18..20us after falling edge
#define USE_FIXED_DELAY 20
//#define USE_FIXED_DELAY 40

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
#define  LASER_POWER      0xB5
//#define  LASER_POWER      0xC2

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

//extra buttons
#define BUTTON_5TH P2_5
#define BUTTON_4TH P2_4
#define TOP_BUTTON P1_0

#define MMB P1_3
#define MMB_IN P1_4
#define WHEEL_DELAY 5000

// quadrature outputs
#define QXA P2_0
#define QXB P2_1
#define QYA P2_2
#define QYB P2_3

#define MMBUTT PUSH2

// approx 5us instead of 13us
#define GPIO_OUT_SET_SUB(port, pin) (P##port##OUT |=  (1<<pin))
#define GPIO_OUT_CLR_SUB(port, pin) (P##port##OUT &= ~(1<<pin))

#define MHZ 12
#define SET_CPU_CLOCK(mhz) { DCOCTL = CALDCO_##mhz##MHZ; BCSCTL1 = CALBC1_##mhz##MHZ; };

const unsigned int quad_state[] = { 0, 1, 3, 2 };
unsigned int motion = 200, k = 0;
unsigned int quad_x, quad_y, t;
signed delta_x, delta_y;
signed int sensor_resolution;
signed int sensor_divisor;
volatile byte button_state, prev_button_state, button_update;

volatile unsigned char SW_state;
volatile signed int scroll_change = 0;
volatile unsigned int mmb_trigger;
bool mmb_attached_irq;

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
  sensor_divisor = 1;
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

  pinMode(BUTTON_5TH, INPUT_PULLUP);
  pinMode(BUTTON_4TH, INPUT_PULLUP);
  pinMode(TOP_BUTTON, INPUT_PULLUP);

//  SW_state = (digitalRead(QRA) << 1) + digitalRead(QRB);
  SW_state = (P2IN >> 6) & 0x03;

  mmb_trigger = 0;
  attachInterrupt(MMB, mmb_falling, FALLING);
  mmb_attached_irq = 1;

  // set motion pin as interrupt input FALLING
  attachInterrupt(MOTION_PIN, set_motion, FALLING);

  button_state = 0;
  prev_button_state = 0;
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

  if((abs(delta_x) >= sensor_divisor) && (!change_period_lapsed_x)) {
    if(delta_x > 0) {
      ++quad_x;
      delta_x -= sensor_divisor;
    } else {
      --quad_x;
      delta_x += sensor_divisor;
    }
    quad_x &= 0x03;
#ifdef GPIO_OUT_SET_SUB
    (quad_state[quad_x] & 0x01)?GPIO_OUT_SET_SUB(2, 0):GPIO_OUT_CLR_SUB(2, 0);
    (quad_state[quad_x] & 0x02)?GPIO_OUT_SET_SUB(2, 1):GPIO_OUT_CLR_SUB(2, 1);
#else
    digitalWrite(QXA, (quad_state[quad_x] & 0x01)?HIGH:LOW);
    digitalWrite(QXB, (quad_state[quad_x] & 0x02)?HIGH:LOW);
#endif
    change_period_lapsed_x = change_period_x;
  }
  --change_period_lapsed_x;

  if((abs(delta_y) >= sensor_divisor) && (!change_period_lapsed_y)) {
    if(delta_y > 0) {
      ++quad_y;
      delta_y -= sensor_divisor;
    } else {
      --quad_y;
      delta_y += sensor_divisor;
    }
    quad_y &= 0x03;
#ifdef GPIO_OUT_SET_SUB
    (quad_state[quad_y] & 0x01)?GPIO_OUT_SET_SUB(2, 2):GPIO_OUT_CLR_SUB(2, 2);
    (quad_state[quad_y] & 0x02)?GPIO_OUT_SET_SUB(2, 3):GPIO_OUT_CLR_SUB(2, 3);
#else
    digitalWrite(QYA, (quad_state[quad_y] & 0x01)?HIGH:LOW);
    digitalWrite(QYB, (quad_state[quad_y] & 0x02)?HIGH:LOW);
#endif
    change_period_lapsed_y = change_period_y;
  }
  --change_period_lapsed_y;

  // just in case there is still IRQ pending
  if((abs(delta_x) < sensor_divisor) && (abs(delta_y) < sensor_divisor) && (!(P1IN & BIT1))) {
    ++motion;
  }

  //button_state &= BIT0 | BIT4 | BIT5;
  prev_button_state = button_state;
  //             top button               4th    5th        MMB
  button_state = (P1IN & BIT0) | (P2IN & (BIT4 | BIT5)) | ((P1IN & BIT4) >> 3);

  if(prev_button_state ^ button_state) {
    button_update |= prev_button_state ^ button_state;
  }

  if(!(button_state & BIT0)) {
    if((button_update & BIT4) && !(button_state & BIT4)) {
      // change resolution to finer - button 4th
      if(sensor_resolution < 3) {
        ++sensor_resolution;
    		if(sensor_resolution < 0) {
          sensor_divisor = abs(sensor_resolution);
        } else {
    			sensor_resolution &= 0x03;
    			set_reg(REG_CONFIGURATION2, CONFIG2_400CPI | (sensor_resolution << 5)); //   0x12 (0x92)
    			sensor_divisor = 1;
    		}
        P2OUT = BIT4 | BIT5 | BIT6 | BIT7 | (abs(sensor_resolution+4) & 0x0F);
      }
      button_update &= ~BIT4;
      //button_state |= BIT4;
    } else {
      if((button_update & BIT5) && !(button_state & BIT5)) {
        // change resolution to coarser - button 5th
        if(sensor_resolution > -4) {
          --sensor_resolution;
          if(sensor_resolution < 0) {
            sensor_divisor = abs(sensor_resolution);
          } else {
      			sensor_resolution &= 0x03;
      			set_reg(REG_CONFIGURATION2, CONFIG2_400CPI | (sensor_resolution << 5)); //   0x12 (0x92)
      			sensor_divisor = 1;
          }
          P2OUT = BIT4 | BIT5 | BIT6 | BIT7 | (abs(sensor_resolution+4) & 0x0F);
        }
        button_update &= ~BIT5;
        //button_state |= BIT5;
      }
    }
  }
}


void set_motion() {
  ++motion;
}

volatile byte quad_raw_out, test;

//P2_0 QXA XQ
//P2_1 QXB  X
//P2_2 QYA YQ
//P2_3 QYB  Y

// /----- QYB
// |/---- QYA
// ||/--- QXB
// |||/-- QXA
// 3210
// 0011 CODE_5TH_DOWN     (0x03)
// 0101 CODE_WHEEL_RIGHT  (0x05)
// 0110 CODE_5TH_UP       (0x06)
// 0111 CODE_WHEEL_DOWN   (0x07)
// 1001 CODE_4TH_UP       (0x09)
// 1010 CODE_WHEEL_LEFT   (0x0A)
// 1011 CODE_WHEEL_UP     (0x0B)
// 1100 CODE_4TH_DOWN     (0x0C)
// 1101 CODE_MMB_DOWN     (0x0D)
// 1110 CODE_MMB_UP       (0x0E)

volatile byte code_send;

#define CODE_IDLE         0
#define CODE_5TH_DOWN     (0x03)
#define CODE_WHEEL_RIGHT  (0x05)
#define CODE_5TH_UP       (0x06)
#define CODE_WHEEL_DOWN   (0x07)
#define CODE_4TH_UP       (0x09)
#define CODE_WHEEL_LEFT   (0x0A)
#define CODE_WHEEL_UP     (0x0B)
#define CODE_4TH_DOWN     (0x0C)
#define CODE_MMB_DOWN     (0x0D)
#define CODE_MMB_UP       (0x0E)

void mmb_falling() {

  quad_raw_out = P2OUT; // | BIT4 | BIT5;

  code_send = CODE_IDLE;

  if((button_update & BIT1) || (mmb_prev_state ^ (P1IN & BIT4))) {
    if((P1IN & BIT4)) {
      mmb_prev_state = BIT4 ;//(P1IN & BIT4);
      code_send = CODE_MMB_UP;
    } else {
      mmb_prev_state = 0; //(P1IN & BIT4);
      code_send = CODE_MMB_DOWN;
    }
    P2OUT = (quad_raw_out ^ code_send);
  } else {
    if(button_update & BIT4) {
      // 4th - left side button
      if(button_state & BIT4)
        code_send = CODE_4TH_UP;
      else
        code_send = CODE_4TH_DOWN;
      P2OUT = (quad_raw_out ^ code_send);
    } else {
      if(button_update & BIT5) {
        // 5th button - right side button
        if(button_state & BIT5)
          code_send = CODE_5TH_UP;
        else
          code_send = CODE_5TH_DOWN;
        P2OUT = (quad_raw_out ^ code_send); // 0x0100
      } else {
        if(scroll_change != 0) {
          if(scroll_change < 0)
            code_send = (button_state & BIT0)?CODE_WHEEL_DOWN:CODE_WHEEL_LEFT;
          else
            code_send = (button_state & BIT0)?CODE_WHEEL_UP:CODE_WHEEL_RIGHT;
          P2OUT = (quad_raw_out ^ code_send);
        }
      }
    }
  }

  // with code confirmation we must not wait for falling edge
  delayMicroseconds(USE_FIXED_DELAY);

  P2OUT = quad_raw_out;

  ++mmb_trigger;
  mmb_last_trigger = millis();

  // MMB is kept longer to confirm reception
  // only then we can clear status for each code sent
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
          button_update &= ~BIT1;
          break;
      case CODE_4TH_DOWN:
      case CODE_4TH_UP:
          button_update &= ~BIT4;
          break;
      case CODE_5TH_DOWN:
      case CODE_5TH_UP:
          button_update &= ~BIT5;
          break;
    }
  }
}
