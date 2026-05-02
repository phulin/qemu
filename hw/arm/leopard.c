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
static uint32_t leopard_debug_read32(uint32_t addr);
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
        } else if (leopard_pc_sample_n % 4000 == 0) {
            /* After warmup, print every 4000th sample (~200ms at 50us)
             * to expose late-boot hang points. */
            fprintf(stderr, "[pc-sample-late n=%d] pc=%#x lr=%#x sp=%#x\n",
                    leopard_pc_sample_n, pc, lr, sp);
        }
        leopard_pc_sample_n++;
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

static uint16_t leopard_get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t leopard_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
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
    };
    uint32_t cfg = leopard_debug_read32(0x4071e6c0);
    /*
     * The web stack independently classifies accepted sockets as local
     * vs remote management by testing the peer address against this LAN
     * address global and the firmware's mask at 0x40689490.  The target
     * image's default management subnet is 192.168.0.0/24.  Do not write
     * 0x40689490 here; in the ARP path it is also used as a callback/list
     * slot, and poisoning it with a netmask crashes tNetTask.
     */
    leopard_debug_write32(0x40689488, 0x0100a8c0);
    if (!cfg || leopard_debug_read32(cfg + 0x18)) {
        leopard_debug_write32(0x4066bb28, 0);
        return;
    }

    /*
     * IP input walks (*(cfg+0x18)+0x10) as an in_ifaddr-style list:
     *   node[0]   -> sockaddr, byte 1 is AF_INET (2)
     *   node[2]   must be non-zero
     *   sockaddr+4 holds the IPv4 address as a host-endian u32
     *   node[0x18] is the next pointer
     * Populate only those fields so the real IP/TCP path sees
     * 192.168.0.1 as a local address.
     */
    leopard_debug_write32(cfg + 0x18, FAKE_IF_ROOT);
    leopard_debug_write32(FAKE_IF_ROOT + 0x10, FAKE_ADDR_NODE);
    leopard_debug_write32(FAKE_IF_ROOT + 0x2c, 0);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x00, FAKE_SOCKADDR);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x08, 1);
    leopard_debug_write32(FAKE_ADDR_NODE + 0x60, 0);
    leopard_debug_write32(FAKE_SOCKADDR + 0x00, 0x00000200);
    leopard_debug_write32(FAKE_SOCKADDR + 0x04, 0x0100a8c0);
    leopard_debug_write32(0x4066bb28, 0);
    fprintf(stderr,
            "[ip-bind] synthesized AF_INET 192.168.0.1 root=%#x node=%#x\n",
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
    if (size >= 54 && buf[12] == 0x08 && buf[13] == 0x00 &&
        (buf[14] >> 4) == 4 && buf[23] == 6) {
        const uint8_t *ip = buf + 14;
        uint8_t ihl = (ip[0] & 0x0f) * 4;
        if (ihl >= 20 && size >= 14 + ihl + 20) {
            const uint8_t *tcp = ip + ihl;
            uint16_t ip_len = leopard_get_be16(ip + 2);
            uint8_t thl = (tcp[12] >> 4) * 4;
            uint16_t data_len = ip_len >= ihl + thl ? ip_len - ihl - thl : 0;
            fprintf(stderr,
                    "[fe] RX tcp %u.%u.%u.%u>%u.%u.%u.%u %u>%u "
                    "flags=%#x seq=%#x ack=%#x win=%u iplen=%u datalen=%u\n",
                    buf[26], buf[27], buf[28], buf[29],
                    buf[30], buf[31], buf[32], buf[33],
                    leopard_get_be16(tcp), leopard_get_be16(tcp + 2),
                    tcp[13], leopard_get_be32(tcp + 4),
                    leopard_get_be32(tcp + 8), leopard_get_be16(tcp + 14),
                    ip_len, data_len);
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
     * request for 192.168.0.1, generate the reply ourselves and
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
        uint32_t our_ip = (192u<<24) | (168u<<16) | (0u<<8) | 1u;
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
            uint8_t fw_mac[6];
            leopard_fe_get_fw_mac(s, fw_mac);
            /* dst = sender of request */
            memcpy(reply + 0, buf + 6, 6);
            memcpy(reply + 6, fw_mac, 6);
            reply[12] = 0x08; reply[13] = 0x06;
            reply[14] = 0x00; reply[15] = 0x01;
            reply[16] = 0x08; reply[17] = 0x00;
            reply[18] = 6; reply[19] = 4;
            reply[20] = 0x00; reply[21] = 0x02;       /* reply */
            memcpy(reply + 22, fw_mac, 6);
            reply[28] = 192; reply[29] = 168; reply[30] = 0; reply[31] = 1;
            memcpy(reply + 32, buf + 22, 6);          /* tgt HW = orig sender HW */
            memcpy(reply + 38, buf + 28, 4);          /* tgt IP = orig sender IP */
            fprintf(stderr, "[fe] ARP auto-reply: %02x:%02x:%02x:%02x:%02x:%02x is 192.168.0.1\n",
                    fw_mac[0], fw_mac[1], fw_mac[2], fw_mac[3], fw_mac[4], fw_mac[5]);
            qemu_send_packet(qemu_get_queue(s->nic), reply, sizeof(reply));
            return size;
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
            if (len && len <= 1500 && data_ptr && s->peer_mac_valid) {
                uint8_t frame[1514];
                uint8_t fw_mac[6];
                hwaddr ba = leopard_dram_ptr(data_ptr);

                leopard_fe_get_fw_mac(s, fw_mac);
                memcpy(frame, s->peer_mac, 6);
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
                if (leopard_fe_tx_dedup(s, frame, len + 14)) {
                    fprintf(stderr,
                            "[fe] synthetic L3 TX dedup mbuf=%#x data=%#x len=%u\n",
                            mbuf, data_ptr, len + 14);
                } else if (s->nic) {
                    qemu_send_packet(qemu_get_queue(s->nic), frame, len + 14);
                }
            } else {
                fprintf(stderr, "[fe] synthetic L3 skip mbuf=%#x len=%u data=%#x peer=%d\n",
                        mbuf, len, data_ptr, s->peer_mac_valid);
            }
        }
        if (o == 0xf80 || o == 0xf84 ||
            o == 0xfa0 || o == 0xfa4 || o == 0xfa8 || o == 0xfb0 || o == 0xfb4 ||
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
            switch (o) {
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
