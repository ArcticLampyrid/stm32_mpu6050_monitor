## MPU6050 Monitor
A demo to monitor MPU6050's data, using Kalman filter to calculate Euler angles.

## BOM
- [STM32F401RCT6](https://www.st.com/en/microcontrollers-microprocessors/stm32f401rc.html)
- [MPU6050](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- OLED Screen (SPI, SSD1306, 128x64)

## Pin Configuration
| Device                        | Pin                                                           |
| ----------------------------- | ------------------------------------------------------------- |
| Board                         | LED --> C13                                                   |
| MPU6050                       | SCL --> B6, SDA --> B7                                        |
| Screen (SPI, SSD1306, 128x64) | SCK --> A5, MOSI --> A7, CS --> A8, RESET --> A11, DC --> A12 |
| UART                          | Host's RX --> A9 --> MCU's TX, Host's TX --> A10 --> MCU's RX |

## Reference
- [MPU-6000 and MPU-6050 Product Specification Revision 3.4](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [How a Kalman filter works, in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
- [A practical approach to Kalman filter and how to implement it](http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/)