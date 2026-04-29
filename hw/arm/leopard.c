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
#define NOR_SIZE  0x100

static struct {
    uint8_t  prg[6];     /* PRGDATA0..5 */
    uint8_t  shreg[3];   /* SHREG0..2   */
    uint32_t cnt;
    uint8_t  radr[3];
    uint8_t  rdata;
    uint8_t  *flash;     /* mmap'd ctf-firmware.bin */
    size_t   flash_size;
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
            mdio.phy_reg[phy][reg] = data;
            mdio.last_data = data;
            opname = "wr";
        } else if (op == 2) {           /* standard C22 read */
            mdio.last_data = mdio.phy_reg[phy][reg];
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
/* Track whether each watched PC range has been observed at any sample tick. */
static bool leopard_seen_lanstart;
static bool leopard_seen_ifexec;
static void leopard_pc_sample_cb(void *opaque)
{
    CPUState *cs = qemu_get_cpu(0);
    if (cs) {
        ARMCPU *acpu = ARM_CPU(cs);
        uint32_t pc = acpu->env.regs[15];
        uint32_t lr = acpu->env.regs[14];
        uint32_t sp = acpu->env.regs[13];
        if (leopard_pc_sample_n < 200) {
            fprintf(stderr, "[pc-sample] pc=%#x lr=%#x sp=%#x\n", pc, lr, sp);
        }
        leopard_pc_sample_n++;
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
        if (pc == 0x403bcf98) {
            static int n; if (++n <= 30)
                fprintf(stderr, "[watch] ppe_add hit #%d r0=%#x lr=%#x sp=%#x\n",
                        n, acpu->env.regs[0], lr, sp);
        }
        if (pc >= 0x405b5800 && pc < 0x405b5848) {
            static int n; if (++n <= 30)
                fprintf(stderr, "[watch] stub pc=%#x lr=%#x sp=%#x\n",
                        pc, lr, sp);
        }
    }
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(leopard_pc_sample_timer, ns + 50 * 1000); /* 50 us */
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
#define LEOPARD_FE_SIZE        0x1000
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
typedef struct LeopardFEState LeopardFEState;
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

    bool     enabled_logged;

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
};

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
    /* ARP auto-reply: the firmware's IP layer doesn't actually have an
     * IP assigned (or the bridge<->IP-stack glue is incomplete in our
     * synthetic build), so it never replies to ARP requests for its
     * own IP.  slirp learns guest MACs only by observing outbound
     * traffic; with no firmware ARP reply, slirp never delivers the
     * SYN to the firmware's MAC, and host->guest TCP times out.
     *
     * Stand in for the firmware's ARP layer: when we see an ARP
     * request for 192.168.1.1, generate the reply ourselves and
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
        uint32_t our_ip = (192u<<24) | (168u<<16) | (1u<<8) | 1u;
        uint32_t tgt_ip = ((uint32_t)buf[38]<<24) | ((uint32_t)buf[39]<<16) |
                          ((uint32_t)buf[40]<<8)  | (uint32_t)buf[41];
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
            uint32_t adrh = s->mac_l;
            uint32_t adrl = s->mac_h;
            uint8_t fw_mac[6];
            fw_mac[0] = (adrh >> 8) & 0xff;
            fw_mac[1] = (adrh >> 0) & 0xff;
            fw_mac[2] = (adrl >> 24) & 0xff;
            fw_mac[3] = (adrl >> 16) & 0xff;
            fw_mac[4] = (adrl >> 8) & 0xff;
            fw_mac[5] = (adrl >> 0) & 0xff;
            /* dst = sender of request */
            memcpy(reply + 0, buf + 6, 6);
            memcpy(reply + 6, fw_mac, 6);
            reply[12] = 0x08; reply[13] = 0x06;
            reply[14] = 0x00; reply[15] = 0x01;
            reply[16] = 0x08; reply[17] = 0x00;
            reply[18] = 6; reply[19] = 4;
            reply[20] = 0x00; reply[21] = 0x02;       /* reply */
            memcpy(reply + 22, fw_mac, 6);
            reply[26] = 192; reply[27] = 168; reply[28] = 1; reply[29] = 1;
            memcpy(reply + 32, buf + 22, 6);          /* tgt HW = orig sender HW */
            memcpy(reply + 38, buf + 28, 4);          /* tgt IP = orig sender IP */
            fprintf(stderr, "[fe] ARP auto-reply: %02x:%02x:%02x:%02x:%02x:%02x is 192.168.1.1\n",
                    fw_mac[0], fw_mac[1], fw_mac[2], fw_mac[3], fw_mac[4], fw_mac[5]);
            /* Try both: direct send AND deferred send.  Slirp may
             * filter re-entrant sends; the deferred path catches that. */
            qemu_send_packet(qemu_get_queue(s->nic), reply, sizeof(reply));
            memcpy(leopard_arp_reply, reply, sizeof(reply));
            leopard_arp_nic = s->nic;
            timer_mod(leopard_arp_send_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
            return size;  /* swallow the request */
        }
    }

    if (!(s->glo_cfg & 4)) return 0;
    if (!s->rx_base || !s->rx_max) return 0;
    if (size > 1600) return size;        /* drop oversize */

    hwaddr base = leopard_fe_dma_addr(s->rx_base);
    uint32_t idx = s->rx_drx_idx;
    uint32_t d[4];
    leopard_fe_read_desc(base, idx, d);
    if (rx_log < 12) {
        fprintf(stderr, "[fe] RX desc[%u]: d0=%#x d1=%#x d2=%#x d3=%#x ba=%#llx\n",
                idx, d[0], d[1], d[2], d[3],
                (unsigned long long)leopard_fe_dma_addr(d[0]));
    }

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
        if (rx_log < 12) {
            fprintf(stderr, "[fe] RX auto-post idx=%u buf=%#x\n",
                    idx, buf_pa);
        }
    }

    if (d[1] & 0x80000000u) {
        /* HW already wrote here, software hasn't consumed; drop. */
        if (rx_log < 12) fprintf(stderr, "[fe] RX drop: DDONE already set\n");
        return 0;
    }
    hwaddr ba = leopard_fe_dma_addr(d[0]);
    address_space_write(&address_space_memory, ba,
                        MEMTXATTRS_UNSPECIFIED, buf, size);
    /* MTK PDMA RX descriptor.  Different MTK SoC generations encode the
     * packet length in different bit fields of d[1], and we don't know
     * a priori which field this firmware reads.  Set the length in
     * BOTH bits 29:16 (PLEN1) and bits 13:0 (PLEN0) — the previous
     * encoding (29:16 only) caused this firmware's etherPacketAdj to
     * see m_len=0 and reject every packet as "less than 14".
     *
     *   d[1] bit 31     = DDONE (HW filled)
     *   d[1] bit 30     = LS0   (last segment of packet)
     *   d[1] bits 29:16 = PLEN1 / PLEN0-alt (segment 1 length)
     *   d[1] bits 13:0  = PLEN0 (segment 0 length, primary on this gen)
     *   d[2] = VLAN tag / hash / RSS info (left zero — no VLAN)
     *   d[3] bits 22:19 = SPORT (source switch port + 1, 1..4 for LAN)
     *
     * SPORT must be non-zero or the firmware's RX driver treats this as
     * an invalid descriptor and drops the packet.  Use port 1 (= eth1)
     * which is always part of the LAN bridge per boot UART. */
    uint32_t len14 = (uint32_t)(size & 0x3fff);
    d[1] = 0x80000000u                  /* DDONE */
         | 0x40000000u                  /* LS0 - single-segment packet */
         | (len14 << 16)                /* PLEN1 */
         | len14;                       /* PLEN0 */
    d[2] = 0;
    d[3] = (1u << 19);                /* SPORT = 1 (= eth1) */
    leopard_fe_write_desc(base, idx, d);
    s->rx_drx_idx = (idx + 1) % s->rx_max;

    s->int_status |= FE_INT_RX_DONE_INT0;
    leopard_fe_update_irq(s);
    return size;
}

static uint64_t leopard_fe_read(void *opaque, hwaddr off, unsigned size)
{
    LeopardFEState *s = opaque;
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
        s->int_status &= ~(uint32_t)val;       /* W1C */
        leopard_fe_update_irq(s);
        break;
    case FE_PDMA_INT_MASK:
        s->int_mask = val;
        leopard_fe_update_irq(s);
        break;
    default: {
        /* Store the value so subsequent reads return it (the firmware
         * polls some PPE bits and stalls if they're stuck at 0). */
        unsigned o = (unsigned)off & 0xfff;
        s->fe_misc[o >> 2] = (uint32_t)val;
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

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&leopard_fe_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
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
