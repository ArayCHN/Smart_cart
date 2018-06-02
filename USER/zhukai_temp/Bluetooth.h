#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include "sys.h" 

extern u8 BluetoothReceiver;
extern u8 BluetoothControlMode;

void Bluetooth_Init(u32 bound);
void BluetoothControl(void);

#endif


