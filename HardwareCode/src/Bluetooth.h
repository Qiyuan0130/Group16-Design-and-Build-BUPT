#pragma once
#include <Arduino.h>

#ifndef HAS_BT_SERIAL
#define HAS_BT_SERIAL 1
extern HardwareSerial BT_SERIAL;     // defined in Bluetooth.cpp
#endif

// Init and TX control
void   Bluetooth_Init(uint32_t baud = 921600);
size_t Bluetooth_TxWritable();       // queue free space (bytes)
void   Bluetooth_TxPoll();           // drain queue to UART (non-blocking)
size_t Bluetooth_TxPending();        // queued bytes waiting to send

// Send APIs (non-blocking: enqueue only)
void   Bluetooth_SendLine(const char* s);
void   Bluetooth_SendRaw(const uint8_t* p, size_t n);

// Receive APIs
int    Bluetooth_Available();        // readable bytes
char   Bluetooth_Read();             // read one char

// Helpers (non-blocking)
void   Bluetooth_SendString(const char* s);
void   Bluetooth_SendFloat(float val, int precision);
void   Bluetooth_SendNewLine();
