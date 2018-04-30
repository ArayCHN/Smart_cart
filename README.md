# Smart_cart
Rui Wang
###This project is for course Mechatronics Systems Lab 2018 spring, offered by Dpt. o'MechE at Tsinghua University.

## Baic Information
*Goal*: 

* A cart that can track a double-lined lane and avoid obstacles.

*Basic Config*:

* [STM32 standard peripheral library](http://stm32.kosyak.info/doc/) 
* Keil mdk as IDE (only on Windows)
* linear CCD for lane detection
* ultrasonic radar for obstacle detection
* unbalanced velocity for cart turns
* remote control module with Wifi
* offline programming

## Mechanical Design
Mechanical structures will be uploaded shortly.

## Circuit Configuration
Circuit config will be uploaded shortly.

## Tests
Here a list of basic information about standard peripherals used on STM32 are listed.

### ENCODER MODE TEST - for cart velocity measurement

The basic idea is to utilize timer `encoder mode` (which counts numbers), and have `systick` exception every thrown 50ms (or other values) to calculate the current velocity. Note that `Systick` here is integrated on Cortex system, not offered by ST.

A list of useful sources:

* [Redirect `printf()` to Keil debug screen](https://www.douban.com/note/248637026/)
* [Motor parameters on taobao.com, offered in Chinese](https://item.taobao.com/item.htm?spm=a230r.1.14.19.3b516d0aHPEsqs&id=533000737918&ns=1&abbucket=5#detail)
* According to my measurement, for this encoder, each rotation cycle has 11 counts.
* [One example for STM32 timer encoder mode usage](https://github.com/xiahouzuoxin/notes/blob/master/essays/STM32%E7%94%A8%E4%BD%9C%E7%BC%96%E7%A0%81%E5%99%A8%E6%8E%A5%E5%8F%A3%E7%9A%84%E5%8E%9F%E7%90%86.md)
* [The official reference for `systick`](https://www.keil.com/pack/doc/CMSIS/Core/html/group__SysTick__gr.html#gabe47de40e9b0ad465b752297a9d9f427)

Redirecting `printf()` to screen is extremely useful here as the debugger in Keil cannot display values real-time for some variables that are changing as fast as the _velocity_ here.

## List of files under  /USER
```
retarget.c     for prinft() redirection
motor_test.c   for motor test
```