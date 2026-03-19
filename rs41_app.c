/*
 * rs41_app.c — Flipper Zero RS41 radiosonde tracker
 *
 * Uses the built-in CC1101 (Sub-GHz radio) to receive and decode
 * Vaisala RS41 radiosondes on 400–406 MHz.
 *
 * Controls:
 *   UP / DOWN  : change receive frequency (±0.1 MHz)
 *   OK         : start / stop reception
 *   BACK       : exit
 *
 * CC1101 configuration:
 *   Modulation  : 2-FSK
 *   Data rate   : 4800 bps  (MDMCFG3/4: DRATE_M=131, DRATE_E=7)
 *   Deviation   : ≈ 2381 Hz  (DEVIATN: E=0, M=4)
 *   BW filter   : ≈ 58 kHz   (CHANBW_E=3, CHANBW_M=3)
 *   Sync word   : 0x086D     (first 2 bytes of the bit-reversed RS41 sync)
 *   Packet mode : infinite length (PKTCTRL0 = 0x02)
 *
 * After sync detection the CC1101 fills its RX FIFO with the remaining
 * 6 sync bytes + 312 payload bytes = 318 bytes total.  The radio thread
 * polls RXBYTES via furi_hal_subghz_rx_pipe_not_empty() / read_packet()
 * and accumulates them.  When 318 bytes are collected the 312-byte payload
 * (offset +6) is passed to rs41_decode().
 */

#include <furi.h>
#include <gui/gui.h>
#include <gui/elements.h>
#include <input/input.h>
#include <furi_hal.h>
#include <furi_hal_spi.h>
#include <furi_hal_spi_config.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rs41_decoder.h"

/* ── Direct CC1101 FIFO access ───────────────────────────────────────────── */
/*
 * furi_hal_subghz_read_packet() (via cc1101_read_fifo in firmware) silently
 * consumes the first byte from the FIFO as a length indicator (capped at 64),
 * then reads only that many bytes — losing one byte per call and reading a
 * random (payload-dependent) amount.  For a 318-byte infinite-mode frame this
 * completely garbles the data.
 *
 * Work-around: read RXBYTES directly, then burst-read exactly that many bytes
 * using furi_hal_spi_bus_trx / furi_hal_spi_bus_rx while holding CS via
 * furi_hal_spi_acquire / furi_hal_spi_release.
 *
 * CC1101 register addresses (with READ=0x80, BURST=0x40):
 *   RXBYTES status register : 0x80|0x40|0x3B = 0xFB
 *   FIFO burst-read         : 0x80|0x40|0x3F = 0xFF
 *   Bit 7 of RXBYTES        : RXFIFO_OVERFLOW flag
 */
#define CC1101_RXBYTES_ADDR   0xFB
#define CC1101_FIFO_BURST_RD  0xFF
#define CC1101_RXFIFO_OVERFLOW 0x80u

/* Return raw RXBYTES register: bit7=overflow, bits6:0=bytes available */
static uint8_t cc1101_rxbytes_direct(void) {
    uint8_t cmd[2] = {CC1101_RXBYTES_ADDR, 0x00};
    uint8_t resp[2] = {0, 0};
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_bus_trx(&furi_hal_spi_bus_handle_subghz, cmd, resp, 2, 50);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    return resp[1];
}

/* Burst-read exactly n bytes from the CC1101 RX FIFO into dst */
static void cc1101_read_fifo_direct(uint8_t* dst, uint8_t n) {
    if(n == 0) return;
    uint8_t cmd = CC1101_FIFO_BURST_RD;
    uint8_t status;
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    /* Send burst-read FIFO command; first byte back = CC1101 status */
    furi_hal_spi_bus_trx(&furi_hal_spi_bus_handle_subghz, &cmd, &status, 1, 50);
    /* CS stays asserted (within acquire/release); clock out n FIFO bytes */
    furi_hal_spi_bus_rx(&furi_hal_spi_bus_handle_subghz, dst, n, 500);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

/* ── CC1101 preset ───────────────────────────────────────────────────────── */
/*
 * Register pairs {address, value}, terminated by {0x00, 0x00}.
 * Frequency is set separately via furi_hal_subghz_set_frequency_and_path().
 *
 * Sync word bytes (SYNC1:SYNC0 = 0x08:0x6D) are the CC1101-visible
 * (MSB-first) form of the RS41 on-air sync word:
 *   RS41 air (LSB-first) : 10 B6 CA 11 22 96 12 F8
 *   Bit-reversed         : 08 6D 53 88 44 69 48 1F
 *
 * The CC1101 consumes the matched 2-byte sync; the remaining 6 sync bytes
 * plus 312 payload bytes (318 total) appear in the RX FIFO.
 *
 * Format of CC1101_RS41_PRESET:
 *   {reg, val} pairs terminated by {0x00, 0x00}, followed by 8-byte PA table.
 *   furi_hal_subghz_load_custom_preset() reads reg/val pairs until reg==0x00,
 *   then loads the next 8 bytes as the PA table (confirmed from firmware).
 */
static const uint8_t CC1101_RS41_PRESET[] = {
    0x02, 0x0D,  /* IOCFG0   : GDO0 asserts when sync found, de-asserts at end  */
    0x03, 0x47,  /* FIFOTHR  : RX threshold = 32 bytes                           */
    0x04, 0x08,  /* SYNC1    : sync high byte (bit-reversed RS41)                */
    0x05, 0x6D,  /* SYNC0    : sync low  byte                                     */
    0x07, 0x04,  /* PKTCTRL1 : no address check                                   */
    0x08, 0x02,  /* PKTCTRL0 : infinite packet length, no CRC                     */
    0x0B, 0x06,  /* FSCTRL1  : IF ≈ 152 kHz                                       */
    0x0C, 0x00,  /* FSCTRL0  : no frequency offset                                */
    0x10, 0xF7,  /* MDMCFG4  : BW ≈ 58 kHz (CHANBW_E=3,M=3), DRATE_E=7          */
    0x11, 0x83,  /* MDMCFG3  : DRATE_M=131 → 4800.18 bps @ 26 MHz               */
    0x12, 0x02,  /* MDMCFG2  : 2-FSK, no Manchester, 16/16 sync                  */
    0x13, 0x22,  /* MDMCFG1  : no FEC, 4 preamble bytes                           */
    0x14, 0xF8,  /* MDMCFG0  : channel spacing mantissa                           */
    0x15, 0x04,  /* DEVIATN  : E=0, M=4 → ≈ 2381 Hz deviation                   */
    0x17, 0x30,  /* MCSM1    : stay in RX after packet                            */
    0x18, 0x18,  /* MCSM0    : auto-calibrate on IDLE→RX                          */
    0x19, 0x1D,  /* FOCCFG   : freq-offset compensation                           */
    0x1A, 0x1C,  /* BSCFG    : bit-sync compensation                              */
    0x1B, 0xC7,  /* AGCCTRL2 : max gain settings                                  */
    0x1C, 0x00,  /* AGCCTRL1                                                       */
    0x1D, 0xB2,  /* AGCCTRL0                                                       */
    0x21, 0xB6,  /* FREND1   : front-end RX config                                */
    0x22, 0x10,  /* FREND0   : front-end TX config (PA index 0)                   */
    0x00, 0x00,  /* Terminator (reg==0 stops the loader loop)                     */
    /* 8-byte PA table — required by furi_hal_subghz_load_custom_preset().
     * Only PA[0] matters for RX; 0xC0 = nominal output, rest unused.          */
    0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* ── Constants ───────────────────────────────────────────────────────────── */
#define FRAME_FIFO_LEN   318   /* 6 remaining sync bytes + 312 payload */
#define FREQ_STEP_MHZ    0.1f
#define FREQ_MIN_MHZ   400.1f
#define FREQ_MAX_MHZ   406.0f
#define FREQ_DEFAULT   403.0f
#define RX_TIMEOUT_MS  2500   /* restart if no new FIFO bytes for this long */

/* ── App state ───────────────────────────────────────────────────────────── */
typedef struct {
    /* GUI */
    Gui*              gui;
    ViewPort*         viewport;
    FuriMessageQueue* input_queue;
    NotificationApp*  notifications;

    /* Threading */
    FuriMutex*  mutex;
    FuriThread* radio_thread;
    volatile bool radio_running; /* set false to stop the radio thread */

    /* Radio config (written by main thread, read by radio thread) */
    volatile float freq_mhz;
    bool           rx_active;   /* true = RX thread is started */

    /* Latest decoded sonde data — access under mutex */
    Rs41Frame frame;
    bool      has_data;
    uint32_t  last_frame_tick;
} AppCtx;

#define TAG "RS41"

/* ── Radio thread ────────────────────────────────────────────────────────── */

static void radio_start_rx(AppCtx* app) {
    furi_hal_subghz_reset();
    furi_hal_subghz_flush_rx();
    furi_hal_subghz_load_custom_preset(CC1101_RS41_PRESET);
    uint32_t actual = furi_hal_subghz_set_frequency_and_path((uint32_t)(app->freq_mhz * 1.0e6f));
    FURI_LOG_I(TAG, "RX start: requested %.3f MHz, CC1101 tuned to %lu Hz",
               (double)app->freq_mhz, (unsigned long)actual);
    furi_hal_subghz_rx();
}

static int32_t radio_thread_fn(void* ctx) {
    AppCtx* app = (AppCtx*)ctx;

    uint8_t  fifo_buf[FRAME_FIFO_LEN];
    uint16_t buf_idx       = 0;
    uint32_t last_byte_ms  = 0;
    bool     collecting    = false;
    float    last_freq     = 0.0f;
    uint32_t frames_rx     = 0;   /* raw frames captured */
    uint32_t frames_ok     = 0;   /* successfully decoded */

    FURI_LOG_I(TAG, "Radio thread started");
    radio_start_rx(app);
    last_freq = app->freq_mhz;

    while(app->radio_running) {
        /* If frequency changed, restart RX */
        float cur_freq = app->freq_mhz;
        if(cur_freq != last_freq) {
            FURI_LOG_I(TAG, "Freq change: %.3f -> %.3f MHz",
                       (double)last_freq, (double)cur_freq);
            last_freq  = cur_freq;
            collecting = false;
            buf_idx    = 0;
            radio_start_rx(app);
        }

        /* Read RXBYTES directly — avoids furi_hal_subghz_read_packet which
         * silently consumes the first FIFO byte as a length indicator.      */
        uint8_t rxbytes_reg = cc1101_rxbytes_direct();

        /* FIFO overflow: flush and restart */
        if(rxbytes_reg & CC1101_RXFIFO_OVERFLOW) {
            FURI_LOG_W(TAG, "FIFO overflow at buf=%u — restarting RX", (unsigned)buf_idx);
            collecting = false;
            buf_idx    = 0;
            radio_start_rx(app);
            furi_delay_ms(2);
            continue;
        }

        uint8_t available = rxbytes_reg & 0x7F;

        if(available == 0) {
            if(collecting) {
                if(furi_get_tick() - last_byte_ms > RX_TIMEOUT_MS) {
                    FURI_LOG_W(TAG, "Frame timeout after %u bytes — restarting RX",
                               (unsigned)buf_idx);
                    collecting = false;
                    buf_idx    = 0;
                    radio_start_rx(app);
                }
            }
            furi_delay_ms(2);
            continue;
        }

        /* Bytes available in FIFO */
        if(!collecting) {
            FURI_LOG_I(TAG, "Sync found — collecting frame (%u bytes already in FIFO)",
                       (unsigned)available);
            collecting   = true;
        }
        last_byte_ms = furi_get_tick();

        /* How many to pull: available (≤64) but no more than remaining space */
        uint16_t space   = FRAME_FIFO_LEN - buf_idx;
        uint8_t  to_read = available;
        if((uint16_t)to_read > space) to_read = (uint8_t)space;

        cc1101_read_fifo_direct(fifo_buf + buf_idx, to_read);
        buf_idx += to_read;

        FURI_LOG_D(TAG, "Read %u FIFO bytes → buf %u/%u",
                   (unsigned)to_read, (unsigned)buf_idx, FRAME_FIFO_LEN);

        if(buf_idx >= FRAME_FIFO_LEN) {
            frames_rx++;
            int16_t rssi = (int16_t)furi_hal_subghz_get_rssi();
            FURI_LOG_I(TAG, "Frame #%lu captured (%u bytes), RSSI=%d dBm",
                       (unsigned long)frames_rx, (unsigned)buf_idx, (int)rssi);

            /* Log the 6 FIFO sync bytes (should be 53 88 44 69 48 1F) */
            FURI_LOG_I(TAG, "Sync bytes: %02X %02X %02X %02X %02X %02X",
                       fifo_buf[0], fifo_buf[1], fifo_buf[2],
                       fifo_buf[3], fifo_buf[4], fifo_buf[5]);

            /* Log raw bytes 0-15 of payload */
            const uint8_t* p = fifo_buf + 6;
            FURI_LOG_I(TAG, "Raw[0..15]: %02X %02X %02X %02X %02X %02X %02X %02X"
                             " %02X %02X %02X %02X %02X %02X %02X %02X",
                       p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],
                       p[8],p[9],p[10],p[11],p[12],p[13],p[14],p[15]);

            /* Log raw bytes 44-57 — frame type (byte 48) and first block start */
            const uint8_t* q = fifo_buf + 6 + 44;
            FURI_LOG_I(TAG, "Raw[44..57]: %02X %02X %02X %02X %02X %02X %02X %02X"
                             " %02X %02X %02X %02X %02X %02X",
                       q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],
                       q[8],q[9],q[10],q[11],q[12],q[13]);

            Rs41Frame decoded;
            bool ok = rs41_decode(fifo_buf + 6, rssi, &decoded);
            if(ok) {
                frames_ok++;
                FURI_LOG_I(TAG, "DECODE OK #%lu: id=%.8s lat=%.4f lon=%.4f alt=%.0f sats=%u T=%.1f P=%.1f",
                           (unsigned long)frames_ok,
                           decoded.id, (double)decoded.lat, (double)decoded.lon,
                           (double)decoded.alt, (unsigned)decoded.sats,
                           (double)decoded.temp, (double)decoded.pres);

                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->frame           = decoded;
                app->has_data        = true;
                app->last_frame_tick = furi_get_tick();
                furi_mutex_release(app->mutex);

                view_port_update(app->viewport);
                notification_message(app->notifications, &sequence_blink_green_10);
            } else {
                FURI_LOG_W(TAG, "Decode FAILED (frame #%lu) id='%.8s' has_gps=%d",
                           (unsigned long)frames_rx, decoded.id, (int)decoded.has_gps);
            }

            /* Restart for next frame */
            collecting = false;
            buf_idx    = 0;
            radio_start_rx(app);
        }
    }

    FURI_LOG_I(TAG, "Radio thread stopping. frames_rx=%lu ok=%lu",
               (unsigned long)frames_rx, (unsigned long)frames_ok);
    furi_hal_subghz_idle();
    furi_hal_subghz_sleep();
    return 0;
}

/* ── Display helpers ─────────────────────────────────────────────────────── */

/* Format a coordinate: "50.0745N" or "014.4189E" */
static void fmt_coord(char* buf, size_t sz, float val, char pos, char neg) {
    char sign     = (val >= 0.0f) ? pos : neg;
    float abs_val = fabsf(val);
    int   deg     = (int)abs_val;
    int   frac    = (int)((abs_val - (float)deg) * 10000.0f);
    if(frac > 9999) frac = 9999;
    if(frac < 0)    frac = 0;
    /* Clamp deg so gcc can prove the buffer is large enough */
    if(deg > 180)   deg  = 180;
    snprintf(buf, sz, "%d.%04d%c", deg, frac, sign);
}

static void draw_cb(Canvas* canvas, void* ctx) {
    AppCtx* app = (AppCtx*)ctx;

    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    /* ── Header ──────────────────────────────────────────────────────── */
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 8, "RS41");

    char freq_str[12];
    snprintf(freq_str, sizeof(freq_str), "%.1f MHz", (double)app->freq_mhz);
    canvas_draw_str(canvas, 38, 8, freq_str);

    if(app->rx_active)
        canvas_draw_str(canvas, 106, 8, "[RX]");

    canvas_draw_line(canvas, 0, 10, 127, 10);

    /* ── Content ─────────────────────────────────────────────────────── */
    if(!app->rx_active && !app->has_data) {
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 32, AlignCenter, AlignCenter,
                                "Press OK to start");
        canvas_draw_str_aligned(canvas, 64, 44, AlignCenter, AlignCenter,
                                "UP/DN: change freq");
        return;
    }

    if(app->rx_active && !app->has_data) {
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 30, AlignCenter, AlignCenter,
                                "Searching...");
        canvas_draw_str_aligned(canvas, 64, 42, AlignCenter, AlignCenter,
                                "UP/DN: change freq");
        canvas_draw_str_aligned(canvas, 64, 54, AlignCenter, AlignCenter,
                                "OK: stop  BACK: exit");
        return;
    }

    /* Has data — show sonde info */
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    Rs41Frame f = app->frame;
    furi_mutex_release(app->mutex);

    canvas_set_font(canvas, FontSecondary);

    /* Row 1: sonde ID + RSSI */
    char row1[32];
    snprintf(row1, sizeof(row1), "ID:%-8s %ddBm", f.id, f.rssi);
    canvas_draw_str(canvas, 0, 20, row1);

    if(f.has_gps) {
        /* Row 2: latitude / longitude */
        char lat_s[20], lon_s[20], row2[44];
        fmt_coord(lat_s, sizeof(lat_s), f.lat, 'N', 'S');
        fmt_coord(lon_s, sizeof(lon_s), f.lon, 'E', 'W');
        snprintf(row2, sizeof(row2), "%-9s %-9s", lat_s, lon_s);
        canvas_draw_str(canvas, 0, 29, row2);

        /* Row 3: altitude + temperature */
        char row3[32];
        if(f.temp > -998.0f)
            snprintf(row3, sizeof(row3), "Alt:%5.0fm T:%+.1fC",
                     (double)f.alt, (double)f.temp);
        else
            snprintf(row3, sizeof(row3), "Alt:%5.0fm  --.-C", (double)f.alt);
        canvas_draw_str(canvas, 0, 38, row3);

        /* Row 4: pressure + satellites */
        char row4[32];
        if(f.pres > 0.0f)
            snprintf(row4, sizeof(row4), "P:%6.1fhPa Sats:%u",
                     (double)f.pres, (unsigned)f.sats);
        else
            snprintf(row4, sizeof(row4), "P:   ---hPa Sats:%u",
                     (unsigned)f.sats);
        canvas_draw_str(canvas, 0, 47, row4);
    } else {
        /* GPS not yet decoded */
        canvas_draw_str(canvas, 0, 29, "No GPS fix yet");
        if(f.temp > -998.0f) {
            char tmp[24];
            snprintf(tmp, sizeof(tmp), "T: %+.1f C", (double)f.temp);
            canvas_draw_str(canvas, 0, 38, tmp);
        }
        if(f.pres > 0.0f) {
            char pstr[24];
            snprintf(pstr, sizeof(pstr), "P: %.1f hPa", (double)f.pres);
            canvas_draw_str(canvas, 0, 47, pstr);
        }
    }

    /* Status hint at bottom */
    canvas_draw_str(canvas, 0, 62, "OK:stop UP/DN:freq BACK:exit");
}

/* ── Input callback (called from GUI thread) ─────────────────────────────── */
static void input_cb(InputEvent* event, void* ctx) {
    FuriMessageQueue* queue = ctx;
    furi_message_queue_put(queue, event, 0);
}

/* ── App entry point ─────────────────────────────────────────────────────── */
int32_t rs41_app(void* p) {
    UNUSED(p);

    AppCtx* app = malloc(sizeof(AppCtx));
    memset(app, 0, sizeof(AppCtx));

    app->freq_mhz         = FREQ_DEFAULT;
    app->rx_active        = false;
    app->has_data         = false;
    app->last_frame_tick  = 0;

    /* Mutex for sonde state */
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    /* Input queue */
    app->input_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    /* ViewPort */
    app->viewport = view_port_alloc();
    view_port_draw_callback_set(app->viewport, draw_cb, app);
    view_port_input_callback_set(app->viewport, input_cb, app->input_queue);

    /* GUI */
    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->viewport, GuiLayerFullscreen);

    /* Notifications */
    app->notifications = furi_record_open(RECORD_NOTIFICATION);

    /* ── Main event loop ────────────────────────────────────────────── */
    bool running = true;
    while(running) {
        InputEvent event;
        FuriStatus status = furi_message_queue_get(app->input_queue, &event, 100);

        if(status == FuriStatusOk && event.type == InputTypeShort) {
            switch(event.key) {

            case InputKeyBack:
                running = false;
                break;

            case InputKeyOk:
                if(!app->rx_active) {
                    /* Start radio thread */
                    app->radio_running = true;
                    app->rx_active     = true;
                    app->radio_thread  = furi_thread_alloc_ex(
                        "RS41Radio", 4096, radio_thread_fn, app);
                    furi_thread_start(app->radio_thread);
                } else {
                    /* Stop radio thread */
                    app->radio_running = false;
                    furi_thread_join(app->radio_thread);
                    furi_thread_free(app->radio_thread);
                    app->radio_thread = NULL;
                    app->rx_active    = false;
                }
                view_port_update(app->viewport);
                break;

            case InputKeyUp:
                if(app->freq_mhz < FREQ_MAX_MHZ - 0.001f) {
                    app->freq_mhz += FREQ_STEP_MHZ;
                    if(app->freq_mhz > FREQ_MAX_MHZ) app->freq_mhz = FREQ_MAX_MHZ;
                }
                view_port_update(app->viewport);
                break;

            case InputKeyDown:
                if(app->freq_mhz > FREQ_MIN_MHZ + 0.001f) {
                    app->freq_mhz -= FREQ_STEP_MHZ;
                    if(app->freq_mhz < FREQ_MIN_MHZ) app->freq_mhz = FREQ_MIN_MHZ;
                }
                view_port_update(app->viewport);
                break;

            default:
                break;
            }
        }
    }

    /* ── Cleanup ─────────────────────────────────────────────────────── */
    if(app->rx_active) {
        app->radio_running = false;
        furi_thread_join(app->radio_thread);
        furi_thread_free(app->radio_thread);
    }

    gui_remove_view_port(app->gui, app->viewport);
    view_port_enabled_set(app->viewport, false);
    view_port_free(app->viewport);

    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);

    furi_message_queue_free(app->input_queue);
    furi_mutex_free(app->mutex);
    free(app);

    return 0;
}
