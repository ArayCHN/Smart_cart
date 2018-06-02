#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include "sys.h" 

extern u8 BluetoothReceiver;
extern u8 BluetoothControlMode;
#define STOP 0
#define TRACK 1
#define LEFT 2
#define RIGHT 3
#define FORWARD 4
#define BACKWARD 5

void Bluetooth_Init(u32 bound);
void BluetoothControl(void);

#endif
