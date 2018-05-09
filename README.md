# Smart_cart
Rui Wang

### This project is for course Mechatronics Systems Lab 2018 spring, offered by Dpt. O'MechE at Tsinghua University.

## Basic Information
**Goal**: 

A cart that

* can track a double-lined lane
* can avoid obstacles.
* drives as fast as it can

**Basic Config**:

* [STM32 standard peripheral library](http://stm32.kosyak.info/doc/) 
* Keil mdk as IDE (only on Windows)
* linear CCD for lane detection
* ultrasonic radar for obstacle detection
* unbalanced velocity for cart turns
* remote control module with Wifi
* offline programming

## Disclaimer
Generally it is recommended to use HAL library, while standard peripheral library is no longer supported by the official ST. With CubeMX and HAL lib, things will be much easier. There are loads more English docs on websites and it also works on Mac. However, due to the course reqs, I had to use the STP3.5 library.

Also, not every link provided here works fine. They are just references and it is highly recommended to get the idea and implement your own code. My code, however, is guaranteed to work on my own hardware settings.

## Mechanical Design
Mechanical structures will be uploaded shortly.

## Circuit Configuration
Circuit config will be uploaded shortly.

## Tests
Here a list of basic information about standard peripherals used on STM32 are listed.

### ENCODER MODE TEST - for cart velocity measurement

The basic idea is to utilize timer `encoder mode` (which counts numbers), and have `systick` exception every thrown 50ms (or other values) to calculate the current velocity. Note that `Systick` here is integrated on Cortex system, not offered by ST.

**A list of useful resources:**

* [Redirect `printf()` to Keil debug screen with JLink](https://www.douban.com/note/248637026/)
* [Motor parameters on taobao.com, offered in Chinese](https://item.taobao.com/item.htm?spm=a230r.1.14.19.3b516d0aHPEsqs&id=533000737918&ns=1&abbucket=5#detail)
* According to my measurement, for this encoder, each rotation cycle has 11 counts.
* [One example for STM32 timer encoder mode usage](https://github.com/xiahouzuoxin/notes/blob/master/essays/STM32%E7%94%A8%E4%BD%9C%E7%BC%96%E7%A0%81%E5%99%A8%E6%8E%A5%E5%8F%A3%E7%9A%84%E5%8E%9F%E7%90%86.md)
* [The official reference for `systick`](https://www.keil.com/pack/doc/CMSIS/Core/html/group__SysTick__gr.html#gabe47de40e9b0ad465b752297a9d9f427)

Redirecting `printf()` to screen is extremely useful here as the debugger in Keil cannot display values real-time for some variables that are changing as fast as the _velocity_ here.

### TIME INTERVAL TEST - for velocity measurement
Due to the limited pins offered by the boarder vendor, I have to find another way of measuring velocity apart from the orthogonal encoder mode. The idea is to use the capture mode and measure the total time interval between several impulses brought by one pin of the encoder. In my implementation, an interrupt will generate every 4 times the encoder crosses a spoke.

**A list of useful resources:**

* [for timer1 and 8, the interrupt process is different from general timers](https://blog.csdn.net/qq_14997473/article/details/46942927)
* [timer interrupt config overview and explanation, without GPIO input](https://blog.csdn.net/longintchar/article/details/43453393)
* [an example of timer interrupt with GPIO input as capture source](https://blog.csdn.net/mvp_dong/article/details/43120533)

**Important tips:**

* Syntax: we use GPIO_Pin1 | GPIO_Pin2 to represent both pins are used. However, for `NVIC` and `TIM_Channel`, this is not the case. When we want to config multiple interruptions or multiple channels of one timer, we have to config each one separately. Refer to my code on this point.
* `TIM8_UP_IRQHandler()` is carried out each time `TIM8->CNT` reloads. This function here specifically determines if a long time has passed and no interrupts happen (meaning, the cart velocity is very slow). In such case, it will assign $v = 0$ arbitrarily because otherwise, the velocity is not getting updated and will keep its previous value even when it is actually 0.

**!!PROBLEM!!**

**unsolved: 一个奇怪的问题：在一些时候，当速度为0好久，然后启动一下，一瞬间速度会很小，紧接着接下来就会出现一个很大的速度。这个速度是错误的，但是为什么会得到这个结果？**

### ULTRASONIC MODULE TEST
We use an ultrasonic module **HC-SR04** to check if there is an obstacle nearby. The ultrasonic module emits a wave when the pin TRIGGER is high for >10 us. The pin ECHO receives a high for a time t, meaning that the time sound travelled was t.

We trigger a timer to record time when rising edge is detected, and stop recording when falling edge is detected.

There are two things worth noting here. First, since there are two ultrasonic modules and we need to know whether the obstacle is on the left side or the right side, a little logistic is needed. We need some flags to mark if it is the left, right or no ultrasonic module that has detected an obstacle within a certain distance. Once we detected an obstacle, we will record the current state in a global variable for our control algorithm to recognize.

Second, different from most examples found online, in our implementation we are not interested in exactly how far away the obstacle is. Rather, we are concerned about whether that distance exceeds a specific _threshold_. For example, let us set the threshold at 50cm. Now the simplest way is when we configure our timer, we let it have a period that accords with a distance of 50cm. Now if the timer ever experiences reloading during a trigger-and-echo process, the distance can directly be determined to be above that threshold. Therefore, there is no need to do extra calculation, which saves our time.

An example: timer prescaler = 72.

72000000 / 72 = 1000000, 1ms = 1000 counts. period = 3000

This will result in a rough distance threshold of 510mm.

**A list of useful resources:**

* [neat code, but not good implementation in that the author did not use interrupt, but wait until signal comes. Not suitable in our case.](https://blog.csdn.net/zhangdaxia2/article/details/50783566)
* [Used interruption mechanism, but there are some minor unnecessities in code.](https://blog.csdn.net/tcjy1000/article/details/70170058)


**Important tips:**

* the ultrasonic module must be connected to a voltage source of 5V, not 3.3V! Otherwise there might be weird problems such as a lot of wrong rising edges on the ECHO pin.
* The time interval between 2 trigger should be longer than 60ms. Otherwise, the echos and triggers will overlap and it is impossible to distinguish which echo is paired with which trigger.

## List of files under  /USER
```
retarget.c     for prinft() redirection
motor_test.c   for motor test
```