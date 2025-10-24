#include "Bluetooth.h"
#include <string.h>
#include <stdlib.h>

// HC-04 wiring: TXD→PA1(RX), RXD→PA0(TX)
HardwareSerial BT_SERIAL(PA1 /*RX*/, PA0 /*TX*/);

// -------- Non-blocking TX ring buffer --------
static constexpr size_t BT_TX_BUF_SZ = 4096;     // power of two, increased for burst tolerance
static constexpr size_t BT_TX_MASK   = BT_TX_BUF_SZ - 1;
static volatile uint32_t bt_tx_head = 0;         // write position
static volatile uint32_t bt_tx_tail = 0;         // read/drain position
static uint8_t bt_tx_buf[BT_TX_BUF_SZ];

static inline size_t tx_count() {
    return (size_t)((bt_tx_head - bt_tx_tail) & BT_TX_MASK);
}
static inline size_t tx_free() {
    return (BT_TX_BUF_SZ - 1) - tx_count();
}

static size_t tx_enqueue(const uint8_t* p, size_t n) {
    size_t free_bytes = tx_free();
    size_t push = (n < free_bytes) ? n : free_bytes;
    if (!p || push == 0) return 0;

    uint32_t head = bt_tx_head & BT_TX_MASK;
    size_t first = BT_TX_BUF_SZ - head; // space until wrap
    if (first > push) first = push;
    memcpy(&bt_tx_buf[head], p, first);
    if (push > first) {
        memcpy(&bt_tx_buf[0], p + first, push - first);
    }
    bt_tx_head = (bt_tx_head + push) & BT_TX_MASK;
    return push;
}

void Bluetooth_Init(uint32_t baud){
    BT_SERIAL.begin(baud);
    bt_tx_head = bt_tx_tail = 0;
}

size_t Bluetooth_TxWritable(){
    return tx_free();
}

size_t Bluetooth_TxPending(){
    return tx_count();
}

void Bluetooth_TxPoll(){
    size_t pending = tx_count();
    if (!pending) return;

    size_t avail = BT_SERIAL.availableForWrite();
    if (!avail) return;

    size_t to_send = (pending < avail) ? pending : avail;
    uint32_t tail = bt_tx_tail & BT_TX_MASK;

    size_t first = BT_TX_BUF_SZ - tail;
    if (first > to_send) first = to_send;
    if (first) {
        size_t n1 = BT_SERIAL.write(&bt_tx_buf[tail], first);
        bt_tx_tail = (bt_tx_tail + n1) & BT_TX_MASK;
        to_send -= n1;
        if (n1 < first) return; // UART HW buffer filled sooner
    }

    if (to_send) {
        size_t n2 = BT_SERIAL.write(&bt_tx_buf[0], to_send);
        bt_tx_tail = (bt_tx_tail + n2) & BT_TX_MASK;
    }
}

void Bluetooth_SendLine(const char* s){
    if (s) Bluetooth_SendRaw((const uint8_t*)s, strlen(s));
    static const uint8_t crlf[2] = {'\r','\n'};
    Bluetooth_SendRaw(crlf, 2);
}

void Bluetooth_SendRaw(const uint8_t* p, size_t n){
    if (!n || !p) return;
    (void)tx_enqueue(p, n); // enqueue as much as possible; overflow is dropped
}

// Receive functions
int Bluetooth_Available(){
    return BT_SERIAL.available();
}

char Bluetooth_Read(){
    return (char)BT_SERIAL.read();
}

void Bluetooth_SendString(const char* s){
    if (!s) return;
    Bluetooth_SendRaw((const uint8_t*)s, strlen(s));
}

void Bluetooth_SendFloat(float val, int precision){
    char buf[32];
    dtostrf(val, 0, precision, buf);
    Bluetooth_SendString(buf);
}

void Bluetooth_SendNewLine(){
    static const uint8_t crlf[2] = {'\r','\n'};
    Bluetooth_SendRaw(crlf, 2);
}
