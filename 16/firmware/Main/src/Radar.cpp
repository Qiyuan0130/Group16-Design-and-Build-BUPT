// Radar.cpp
#include <Arduino.h>
#include <string.h>
#include <stdint.h>

#include "Radar.h"
#include "Bluetooth.h"
#include "Encoder.h"
#include "IMU.h"

// Provided by main.cpp
void GetOdometryData(float* x, float* y);

// NUCLEO-F446RE UART5  PD2=RX  PC12=TX
HardwareSerial RPSerial(PD2, PC12);

static volatile bool g_radar_tx_enabled = true; // default: on
void Radar_EnableTx(bool enable){ g_radar_tx_enabled = enable; }
bool Radar_IsTxEnabled(void){ return g_radar_tx_enabled; }

// -------- Utils --------
static inline void uart_clear_in(HardwareSerial& s, uint32_t ms=5){
    uint32_t t0 = millis();
    while (millis()-t0 < ms) { while (s.available()) (void)s.read(); delay(1); }
}

static bool read_exact(HardwareSerial& s, uint8_t* buf, size_t n, uint32_t timeout_ms){
    uint32_t t0 = millis(); size_t k = 0;
    while (k < n && (millis()-t0) < timeout_ms){
        if (s.available()) buf[k++] = (uint8_t)s.read();
    }
    return k == n;
}

static void send2(HardwareSerial& s, uint8_t a, uint8_t b){
    uint8_t c[2] = {a,b}; s.write(c,2); s.flush();
}

static bool read_descriptor(HardwareSerial& s, const char* tag, uint32_t timeout_ms=200){
    uint8_t d[7];
    if (!read_exact(s, d, 7, timeout_ms)) {
        Serial.print(tag); Serial.println(" desc timeout");
        return false;
    }
    bool ok = (d[0]==0xA5 && d[1]==0x5A);
    Serial.print(tag); Serial.print(" desc: ");
    for (int i=0;i<7;i++){ if (d[i]<16) Serial.print('0'); Serial.print(d[i], HEX); Serial.print(' '); }
    Serial.println(ok ? " [OK]" : " [BAD]");
    return ok;
}

/* ===== Standard node parsing (5 bytes/node) =====
   b0: bit0=start, bit1=~start; quality = b0>>2
   b1 LSB=1 is angle parity; angle_q6 = ((b2<<8 | b1)>>1)
   distance dist_q2 = (b4<<8 | b3)
*/
static bool parse_std5(const uint8_t* p, bool& start, uint8_t& qual, float& ang_deg, float& dist_mm){
    const uint8_t b0=p[0], b1=p[1], b2=p[2], b3=p[3], b4=p[4];
    const bool s  = (b0 & 0x01) != 0;
    const bool ns = (b0 & 0x02) != 0;
    if (s == ns) return false;              // start complement check
    if ((b1 & 0x01) == 0) return false;     // angle parity bit must be 1

    qual = (uint8_t)(b0 >> 2);

    const uint16_t raw = ((uint16_t)b2 << 8) | b1;
    uint16_t angle_q6  = (raw >> 1) & 0x7FFF;  // drop LSB and clamp 15-bit
    if (angle_q6 >= 360u*64u) return false;    // range check
    const uint16_t dist_q2 = ((uint16_t)b4 << 8) | b3;

    ang_deg = (float)angle_q6 * (1.0f/64.0f);
    if (ang_deg >= 360.0f) ang_deg -= 360.0f;
    dist_mm = (float)dist_q2 * 0.25f;

    start = s;
    return true;
}

static inline bool pass_filter(uint8_t q, float d_mm){
    return (q >= RADAR_MIN_QUALITY) &&
           (d_mm >= RADAR_MIN_DIST_MM) &&
           (d_mm <= RADAR_MAX_DIST_MM);
}

void Radar_init(){
    Serial.println("[RPLIDAR] init begin");
    RPSerial.begin(460800);   // 8N1
    delay(10);
    uart_clear_in(RPSerial, 10);

    send2(RPSerial, 0xA5, 0x25);  // STOP
    delay(20);
    send2(RPSerial, 0xA5, 0x40);  // RESET
    delay(600);
    uart_clear_in(RPSerial, 10);

    send2(RPSerial, 0xA5, 0x52);  // GET_HEALTH
    (void)read_descriptor(RPSerial, "[HEALTH]");
    uint8_t health[3];
    if (read_exact(RPSerial, health, 3, 150)){
        Serial.print("[HEALTH] status="); Serial.print(health[0]);
        Serial.print(" err="); Serial.println((uint16_t)health[1] | ((uint16_t)health[2]<<8));
    } else {
        Serial.println("[HEALTH] read failed");
    }

    // standard sampling: SCAN(0x20)
    send2(RPSerial, 0xA5, 0x20);
    (void)read_descriptor(RPSerial, "[SCAN]");
    Serial.println("[RPLIDAR] standard sampling started");
}

// ===== Read and emit (bucket 0.5°, keep near+far, non-blocking) =====
void Radar_read(bool toSerial){
    static uint8_t  buf[4096];
    static size_t   len = 0;
    static uint32_t lastReport = 0;

    // stats
    static uint32_t good = 0, bad = 0, kept = 0, dropped = 0, skipped = 0;

    // TX throttling and thresholds
    const uint16_t  WR_MIN         = 32;
    const uint32_t  DUE_MS         = 20;    // 兜底刷新
    const uint16_t  PENDING_MAX    = 1536;
    static uint32_t last_flush_ms  = 0;

    // Angle buckets: 0.5° per bucket
    #define BUCKET_CDEG 50u
    #define NBUCKETS (36000u/BUCKET_CDEG)
    static uint16_t ang_near[NBUCKETS], dist_near[NBUCKETS];
    static uint8_t  q_near[NBUCKETS], seen_near[NBUCKETS];
    static uint16_t ang_far[NBUCKETS],  dist_far[NBUCKETS];
    static uint8_t  q_far[NBUCKETS],  seen_far[NBUCKETS];
    static bool     rev_active = false;

    // Multi-point frame 0x11 builder
    const uint8_t TYPE_POINTS = 0x11;
    const uint8_t MAX_PER_FRAME = 50; // (1+1+5*n) <= 255
    static uint8_t pkt[2 + 1 + 1 + 1 + 5*MAX_PER_FRAME + 1];
    static int     w = 5;
    static uint8_t cnt = 0;

    auto pkt_begin = [&](){ if (cnt==0){ pkt[0]=0xAA; pkt[1]=0x55; pkt[3]=TYPE_POINTS; pkt[4]=0; w=5; } };
    auto pkt_send_prefix = [&](uint8_t n, uint32_t now){
        if (n==0 || cnt==0) return false;
        uint8_t lenb = (uint8_t)(1 + 1 + 5*n);
        pkt[2] = lenb; pkt[4] = n;
        uint8_t sum = 0; for (int k=0;k<lenb;k++) sum += pkt[3+k];
        pkt[2 + 1 + lenb] = (uint8_t)(-sum);
        size_t total = 2 + 1 + lenb + 1; // 6 + 5*n

        size_t wr = Bluetooth_TxWritable();
        if (Bluetooth_TxPending() >= PENDING_MAX || wr < total) return false;
        Bluetooth_TxPoll();
        if (Bluetooth_TxWritable() < total) return false;
        Bluetooth_SendRaw(pkt, total);
        last_flush_ms = now;

        if (cnt > n){ memmove(&pkt[5], &pkt[5 + 5*n], 5*(cnt - n)); }
        cnt -= n; w = 5 + 5*cnt; return true;
    };
    auto pkt_push_point = [&](uint16_t a_cdeg, uint16_t d_mm, uint8_t q, uint32_t now){
        pkt_begin();
        pkt[w+0]=(uint8_t)(a_cdeg & 0xFF);
        pkt[w+1]=(uint8_t)(a_cdeg >> 8);
        pkt[w+2]=(uint8_t)(d_mm & 0xFF);
        pkt[w+3]=(uint8_t)(d_mm >> 8);
        pkt[w+4]=q; w+=5; cnt++;
        if (cnt >= MAX_PER_FRAME) { (void)pkt_send_prefix(cnt, now); }
    };
    auto flush_buckets = [&](uint32_t now){
        // 角度升序：先近点再远点（若两者不同）
        for (uint32_t k=0;k<NBUCKETS; ++k){
            if (seen_near[k]){
                pkt_push_point(ang_near[k], dist_near[k], q_near[k], now);
                seen_near[k]=0;
            }
            if (seen_far[k]){
                if (!(seen_near[k]==0 && ang_far[k]==ang_near[k] && dist_far[k]==dist_near[k])){
                    pkt_push_point(ang_far[k], dist_far[k], q_far[k], now);
                }
                seen_far[k]=0;
            }
        }
        // 把包里剩余的也尽量发出去
        while (cnt){
            size_t wr = Bluetooth_TxWritable();
            if (Bluetooth_TxPending() >= PENDING_MAX || wr < 6) break;
            uint8_t nfit = (uint8_t)((wr - 6) / 5); if (nfit==0) break;
            uint8_t n = cnt < nfit ? cnt : nfit; if (n > MAX_PER_FRAME) n = MAX_PER_FRAME;
            if (!pkt_send_prefix(n, now)) break;
        }
    };

    // absorb input
    while (RPSerial.available() && len < sizeof(buf)) { buf[len++] = (uint8_t)RPSerial.read(); }

    // consume nodes
    size_t i = 0;
    while (len - i >= 5){
        bool start; uint8_t q; float ang, dist;
        if (parse_std5(&buf[i], start, q, ang, dist)){
            good++; i += 5; if (!pass_filter(q, dist)) { dropped++; continue; } kept++;

            if (toSerial && g_radar_tx_enabled){
                uint32_t ac = (uint32_t)(ang * 100.0f + 0.5f); if (ac >= 36000u) ac -= 36000u;
                uint32_t bi = ac / BUCKET_CDEG; if (bi >= NBUCKETS) bi = NBUCKETS-1;
                uint16_t d = (uint16_t)min( (int)65535, (int)(dist + 0.5f) );
                // 更新近点（更近优先，距离相等取质量高）
                if (!seen_near[bi] || d < dist_near[bi] || (d==dist_near[bi] && q > q_near[bi])){
                    seen_near[bi]=1; ang_near[bi]=(uint16_t)ac; dist_near[bi]=d; q_near[bi]=q;
                }
                // 更新远点（更远优先，距离相等取质量高）
                if (!seen_far[bi]  || d > dist_far[bi]  || (d==dist_far[bi]  && q > q_far[bi])){
                    seen_far[bi]=1;  ang_far[bi]=(uint16_t)ac;  dist_far[bi]=d;  q_far[bi]=q;
                }

                uint32_t now = millis();
                bool revolve = start && rev_active; if (start && !rev_active) rev_active = true;
                bool due = (now - last_flush_ms) >= DUE_MS;
                if (revolve || due){ flush_buckets(now); last_flush_ms = now; }
            }
        }else{ bad++; i += 1; }
    }

    // pack remaining
    if (i && i < len){ memmove(buf, buf + i, len - i); len -= i; } else if (i == len){ len = 0; }

    // fallback flush
    if (toSerial && g_radar_tx_enabled){ uint32_t now2 = millis(); if ((now2 - last_flush_ms) >= 40){ flush_buckets(now2); last_flush_ms = now2; } }

    // reset stats every second
    if ((millis() - lastReport) >= 1000){ lastReport = millis(); good = bad = kept = dropped = skipped = 0; }
}
