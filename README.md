# AVAGO
ADNS-7550 laser sensor chip for quadrature output mouse

Before desoldering controller chip from the original mouse, dump the calibration values that are send during boot-up. Write down what values are written to register 0x1a LASER_CFG0/1 (0x1f) and 0x1c LSRPWR_CFG0/1 (0x1d). See datasheet page 7 for details.

Communication protocol
- at constant intervals (VerticalBlankISR), host is polling with a low level at MMB line
- controller reacts at the falling edge of MMB linkage (see marker 1 - blue line MMB)
- quadrature lines are negated with command for a brief movement (20us ... 40us) (see marker 2 - lines QXB and QYB)
- after restoring original state of quadrature lines, state of MMB is checked to verify reception
  - if MMB line is still low - treat transaction as successfully (see marker 3)

![single transaction](SPI_interrupted.png)
