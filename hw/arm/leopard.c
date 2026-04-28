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
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "system/reset.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/cpu.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "target/arm/gtimer.h"

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

/* --- microsecond counter at TIMER_BASE+0x48 -----------------------------
 * U-Boot reads a free-running counter for udelay()/get_timer().  We expose
 * the host monotonic clock in microseconds at offset 0x48 and treat the rest
 * of the page as RAZ/WI.
 */
static uint64_t leopard_timer_read(void *opaque, hwaddr off, unsigned size)
{
    if (off == 0x48) {
        return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000;
    }
    return 0;
}

static void leopard_timer_write(void *opaque, hwaddr off,
                                uint64_t val, unsigned size) { }

static const MemoryRegionOps leopard_timer_ops = {
    .read = leopard_timer_read,
    .write = leopard_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

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
        DeviceState *gic = qdev_new(TYPE_ARM_GIC);
        SysBusDevice *gicbus = SYS_BUS_DEVICE(gic);
        /* PPI numbers (offset within the 16 PPI slots, INTID = 16 + ppi) */
        const int timer_ppi[] = {
            [GTIMER_PHYS] = 14,  /* INTID 30 */
            [GTIMER_VIRT] = 11,  /* INTID 27 */
            [GTIMER_HYP]  = 10,  /* INTID 26 */
            [GTIMER_SEC]  = 13,  /* INTID 29 */
        };
        int num_spi = 128;
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

    /* Free-running us counter (overlays sysctrl-lo at 0x10004000) */
    memory_region_init_io(timer, NULL, &leopard_timer_ops, NULL,
                          "leopard.timer", LEOPARD_TIMER_SIZE);
    memory_region_add_subregion_overlap(sysmem, LEOPARD_TIMER_BASE,
                                        timer, 1);

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

    /* Start periodic I/O kick timer */
    leopard_io_kick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         leopard_io_kick_cb, NULL);
    timer_mod(leopard_io_kick_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000);

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
