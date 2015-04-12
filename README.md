# Drone sandbox
This projects aims to develop an application to control quadcopters using the Intel Edison Platform.
The control unit is based on an Intel Edison connected to Sparkfun peripheral blocks :
 - [SparkFun Block for Intel® Edison - PWM](https://www.sparkfun.com/products/13042) for ESCs control
 - [SparkFun Block for Intel® Edison - 9 Degrees of Freedom](https://www.sparkfun.com/products/13033) for orientation
 - [SparkFun Block for Intel® Edison - Console](https://www.sparkfun.com/products/13039) for debug and power supply regulator
 - [SparkFun Block for Intel® Edison - GPIO](https://www.sparkfun.com/products/13038) for UART communication to RF module
 
The RF module used is an [XBEE-ZB Pro](http://www.digi.com/products/wireless-wired-embedded-solutions/zigbee-rf-modules/zigbee-mesh-module/xbee-zb-module#overview) module for 2.4GHz long range communication. A custom protocol is implemented in the library.
 

