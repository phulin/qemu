/*
 * TP-Link Archer C7 v2 / QCA9558 machine model.
 *
 * This is an ath79/QCA955x board model for the vendor Linux/MIPS firmware.
 * It intentionally starts with the SoC blocks the 2.6.31 BSP reaches during
 * early boot; Ethernet, PCIe Wi-Fi, and USB are added as concrete device
 * models rather than by patching guest code.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "exec/tswap.h"
#include "exec/cpu-common.h"
#include "hw/char/serial-mm.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/mips/mips.h"
#include "hw/misc/unimp.h"
#include "net/checksum.h"
#include "net/net.h"
#include "qom/object.h"
#include "system/memory.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/mips/cpu.h"

#define TYPE_ARCHER_C7_MACHINE MACHINE_TYPE_NAME("archer-c7-v2")
#define TYPE_QCA955X_GMAC "qca955x-gmac"
OBJECT_DECLARE_SIMPLE_TYPE(Qca955xGmacState, QCA955X_GMAC)

#define C7_RAM_SIZE            (128 * MiB)
#define C7_RAM_KSEG0_BASE      0x80000000ULL
#define C7_KERNEL_LOAD_PADDR   0x00002000ULL
#define C7_KERNEL_ENTRY        0x801c8040ULL
#define C7_ENVP_PADDR          0x02000000
#define C7_ENVP_VADDR          cpu_mips_phys_to_kseg0(NULL, C7_ENVP_PADDR)
#define C7_ENVP_NB_ENTRIES     16
#define C7_ENVP_ENTRY_SIZE     256

#define QCA955X_APB_BASE       0x18000000ULL
#define QCA955X_DDR_BASE       (QCA955X_APB_BASE + 0x00000000)
#define QCA955X_UART_BASE      (QCA955X_APB_BASE + 0x00020000)
#define QCA955X_USB_CTRL_BASE  (QCA955X_APB_BASE + 0x00030000)
#define QCA955X_GPIO_BASE      (QCA955X_APB_BASE + 0x00040000)
#define QCA955X_PLL_BASE       (QCA955X_APB_BASE + 0x00050000)
#define QCA955X_RESET_BASE     (QCA955X_APB_BASE + 0x00060000)
#define QCA955X_GMAC_BASE      (QCA955X_APB_BASE + 0x00070000)
#define QCA955X_WMAC_BASE      (QCA955X_APB_BASE + 0x00100000)
#define QCA955X_SPI_BASE       0x1f000000ULL
#define QCA955X_SPI_SIZE       0x01000000ULL
#define QCA955X_WDT_BASE       0x1d000000ULL
#define QCA955X_WDT_SIZE       0x00001000ULL
#define QCA955X_OTP_BASE       0x18130000ULL
#define QCA955X_OTP_SIZE       0x00002000ULL

#define QCA955X_PCI_MEM_BASE0  0x10000000ULL
#define QCA955X_PCI_MEM_BASE1  0x12000000ULL
#define QCA955X_PCI_CFG_BASE0  0x14000000ULL
#define QCA955X_PCI_CFG_BASE1  0x16000000ULL
#define QCA955X_PCI_CFG_SIZE   0x00001000ULL
#define QCA955X_PCI_CRP_BASE0  (QCA955X_APB_BASE + 0x000c0000)
#define QCA955X_PCI_CRP_BASE1  (QCA955X_APB_BASE + 0x00250000)
#define QCA955X_PCI_CRP_SIZE   0x00001000ULL
#define QCA955X_PCI_CTRL_BASE0 (QCA955X_APB_BASE + 0x000f0000)
#define QCA955X_PCI_CTRL_BASE1 (QCA955X_APB_BASE + 0x00280000)
#define QCA955X_PCI_CTRL_SIZE  0x00000100ULL
#define QCA955X_GE0_BASE       0x19000000ULL
#define QCA955X_GE1_BASE       0x1a000000ULL
#define QCA955X_EHCI0_BASE     0x1b000000ULL
#define QCA955X_EHCI1_BASE     0x1b400000ULL

#define C7_FLASH_UPGRADE_OFF   0x20000ULL
#define C7_FLASH_CONFIG_OFF    0x00fa0000ULL
#define C7_FLASH_CONFIG_SIZE   0x00050000ULL
#define C7_FLASH_ART_OFF       0x00ff0000ULL
#define C7_FLASH_ART_SIZE      0x00010000ULL
#define QCA955X_REV_ID         0x00001130U
#define QCA955X_BOOTSTRAP_40MHZ BIT(4)

#define QCA955X_PLL_CPU_CONFIG_REG         0x00
#define QCA955X_PLL_DDR_CONFIG_REG         0x04
#define QCA955X_PLL_CLK_CTRL_REG           0x08
#define QCA955X_PLL_ETH_XMII_CONTROL_REG   0x28
#define QCA955X_PLL_ETH_SGMII_CONTROL_REG  0x48
#define QCA955X_PLL_ETH_SGMII_SERDES_REG   0x4c

#define QCA955X_DDR_REG_FLUSH_GE0          0x9c
#define QCA955X_DDR_REG_FLUSH_GE1          0xa0

#define QCA955X_GMAC_GLUE_SIZE             0x00000100
#define QCA955X_GMAC_GLUE_SGMII_STATUS     0x18
#define QCA955X_GMAC_GLUE_SGMII_CTRL       0x1c
#define QCA955X_GMAC_GLUE_SGMII_SERDES     0x34
#define QCA955X_GMAC_GLUE_SGMII_RES_CAL    0x58
#define QCA955X_GMAC_GLUE_SGMII_LOCK       BIT(15)
#define QCA955X_GMAC_GLUE_RES_CAL_DONE     0x0000000f

#define QCA955X_RESET_REG_MISC_INT_STATUS  0x10
#define QCA955X_RESET_REG_MISC_INT_ENABLE  0x14
#define QCA955X_RESET_REG_CPU_INT_STATUS   0x18
#define QCA955X_RESET_REG_GLOBAL_INT       0x20
#define QCA955X_RESET_REG_RESET_MODULE     0x1c
#define QCA955X_RESET_REG_REV_ID           0x90
#define QCA955X_RESET_REG_EXT_INT_STATUS   0xac
#define QCA955X_RESET_REG_BOOTSTRAP        0xb0

#define AR71XX_GPIO_REG_OE                 0x00
#define AR71XX_GPIO_REG_IN                 0x04
#define AR71XX_GPIO_REG_OUT                0x08

#define AR71XX_SPI_REG_FS                  0x00
#define AR71XX_SPI_REG_CTRL                0x04
#define AR71XX_SPI_REG_IOC                 0x08
#define AR71XX_SPI_REG_RDS                 0x0c
#define AR71XX_SPI_FS_GPIO                 BIT(0)
#define AR71XX_SPI_IOC_DO                  BIT(0)
#define AR71XX_SPI_IOC_CLK                 BIT(8)
#define AR71XX_SPI_IOC_CS0                 BIT(16)
#define AR71XX_SPI_IOC_CS1                 BIT(17)
#define AR71XX_SPI_IOC_CS2                 BIT(18)

#define SPI_NOR_CMD_READ                   0x03
#define SPI_NOR_CMD_FAST_READ              0x0b
#define SPI_NOR_CMD_RDSR                   0x05
#define SPI_NOR_CMD_RDID                   0x9f

#define QCA955X_CPU_IRQ_MISC               6
#define QCA955X_MISC_IRQ_UART              BIT(3)

#define QCA955X_GMAC_MMIO_SIZE             0x10000
#define QCA955X_GMAC_REG_MII_MGMT_CFG      0x20
#define QCA955X_GMAC_REG_MII_MGMT_CMD      0x24
#define QCA955X_GMAC_REG_MII_MGMT_ADDRESS  0x28
#define QCA955X_GMAC_REG_MII_MGMT_CTRL     0x2c
#define QCA955X_GMAC_REG_MII_MGMT_STATUS   0x30
#define QCA955X_GMAC_REG_MII_MGMT_IND      0x34
#define QCA955X_GMAC_REG_DMA_TX_CTRL       0x180
#define QCA955X_GMAC_REG_DMA_TX_DESC       0x184
#define QCA955X_GMAC_REG_DMA_TX_STATUS     0x188
#define QCA955X_GMAC_REG_DMA_RX_CTRL       0x18c
#define QCA955X_GMAC_REG_DMA_RX_DESC       0x190
#define QCA955X_GMAC_REG_DMA_RX_STATUS     0x194
#define QCA955X_GMAC_REG_DMA_INTR_MASK     0x198
#define QCA955X_GMAC_REG_DMA_INTR          0x19c
#define QCA955X_GMAC_REG_FIFO_CFG0         0x48
#define QCA955X_GMAC_REG_FIFO_CFG1         0x4c
#define QCA955X_GMAC_REG_FIFO_CFG2         0x50
#define QCA955X_GMAC_REG_FIFO_CFG3         0x54
#define QCA955X_GMAC_REG_FIFO_CFG4         0x58
#define QCA955X_GMAC_REG_FIFO_CFG5         0x5c

#define QCA955X_GMAC_IRQ_TX                BIT(0)
#define QCA955X_GMAC_IRQ_RX                BIT(4)
#define QCA955X_GMAC_RX_STATUS_DONE        BIT(0)
#define QCA955X_GMAC_DESC_STRIDE           0x0c
#define QCA955X_GMAC_DESC_NEXT             0x08
#define QCA955X_GMAC_DESC_EMPTY            BIT(31)
#define QCA955X_GMAC_MAX_FRAME             2048
#define QCA955X_GMAC_FCS_LEN               4
#define QCA955X_S17_LAN_VID                1
#define QCA955X_S17_WAN_VID                2
#define QCA955X_S17_LAN_PORT               2
#define ETH_P_8021Q                        0x8100
#define ETH_P_IP                           0x0800
#define ETH_P_ARP                          0x0806
#define ETH_P_IPV6                         0x86dd

#define AR724X_PCI_REG_APP                 0x00
#define AR724X_PCI_REG_RESET               0x18
#define AR724X_PCI_REG_INT_STATUS          0x4c
#define AR724X_PCI_REG_INT_MASK            0x50
#define AR724X_PCI_RESET_LINK_UP           BIT(0)

#define PCI_VENDOR_ID_ATHEROS              0x168c
#define PCI_DEVICE_ID_ATHEROS_AR9580       0x0033
#define PCI_CLASS_NETWORK_OTHER            0x0280
#define PCI_STATUS_CAP_LIST                BIT(4)
#define PCI_CAP_ID_PM                      0x01
#define PCI_CAP_ID_EXP                     0x10

#define QCA955X_PHY_BMCR                   0
#define QCA955X_PHY_BMSR                   1
#define QCA955X_PHY_ID1                    2
#define QCA955X_PHY_ID2                    3
#define QCA955X_PHY_ANAR                   4
#define QCA955X_PHY_ANLPAR                 5
#define QCA955X_PHY_SPEC_STATUS            17

#define QCA955X_MII_CMD_READ               BIT(0)

#define QCA955X_S17_ADDR_PHY               0x18
#define QCA955X_S17_ADDR_REG               0
#define QCA955X_S17_DATA_PHY_BASE          0x10
#define QCA955X_S17_DATA_PHY_LAST          0x17
#define QCA955X_S17_REG_SPACE_SIZE         0x80000
#define QCA955X_S17_REG_MASK_CTRL          0x0000
#define QCA955X_S17_REG_GLOBAL_INT_STATUS  0x0020
#define QCA955X_S17_REG_PORT_STATUS_BASE   0x007c
#define QCA955X_S17_REG_VTU_FUNC1          0x0614

static const char c7_default_cmdline[] =
    "console=ttyS0,115200 root=31:02 rootfstype=squashfs "
    "init=/sbin/init "
    "mtdparts=ath-nor0:128k(u-boot),1024k(kernel),14848k(rootfs),"
    "320k(config),64k(ART) mem=128M";

typedef struct ArcherC7Prom {
    target_ulong kernel_entry;
    target_ulong stack;
    target_ulong argv;
    target_ulong envp;
    target_ulong ram_size;
} ArcherC7Prom;

typedef struct Qca955xRegs {
    MemoryRegion iomem;
    const char *name;
    qemu_irq irq;
    hwaddr irq_status_offset;
    hwaddr irq_enable_offset;
    uint32_t regs[0x2000 / 4];
    uint32_t read_or_mask[0x2000 / 4];
    uint32_t write_self_clear_mask[0x2000 / 4];
} Qca955xRegs;

typedef struct Qca955xMiscIrqLine {
    Qca955xRegs *intc;
    uint32_t status_bit;
} Qca955xMiscIrqLine;

struct Qca955xGmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;
    uint32_t unit;
    uint32_t regs[QCA955X_GMAC_MMIO_SIZE / 4];
    uint16_t phy_regs[32][32];
    uint32_t switch_regs[QCA955X_S17_REG_SPACE_SIZE / 4];
    uint32_t s17_addr;
    hwaddr rx_desc;
    hwaddr tx_desc;
    QEMUTimer *rx_file_timer;
    bool rx_file_done;
};

typedef enum C7SpiOutKind {
    C7_SPI_OUT_NONE,
    C7_SPI_OUT_ID,
    C7_SPI_OUT_STATUS,
    C7_SPI_OUT_FLASH,
} C7SpiOutKind;

typedef struct ArcherC7Spi {
    MemoryRegion iomem;
    uint8_t *flash;
    uint32_t regs[4];
    bool cs_active;
    bool last_clk;
    uint8_t cmd;
    uint8_t tx_byte;
    unsigned tx_bits;
    unsigned addr_bytes;
    uint32_t addr;
    bool fast_read_dummy;
    uint32_t rx_shift;
    C7SpiOutKind out_kind;
    unsigned out_pos;
    unsigned out_bit;
} ArcherC7Spi;

typedef struct ArcherC7Pcie {
    MemoryRegion ctrl;
    MemoryRegion crp;
    MemoryRegion cfg;
    uint32_t ctrl_regs[0x100 / 4];
    uint32_t crp_regs[0x1000 / 4];
    uint32_t cfg_regs[0x1000 / 4];
    bool present;
} ArcherC7Pcie;

static ArcherC7Prom c7_prom;

static const uint8_t c7_spi_jedec_id[] = { 0xef, 0x40, 0x18 };

static bool c7_gmac_trace_enabled(void)
{
    static int enabled = -1;

    if (enabled < 0) {
        const char *env = getenv("ARCHER_C7_GMAC_TRACE");
        enabled = env && env[0] && g_strcmp0(env, "0") != 0;
    }
    return enabled;
}

#define C7_GMAC_TRACE(...)                                      \
    do {                                                        \
        if (c7_gmac_trace_enabled()) {                          \
            fprintf(stderr, __VA_ARGS__);                       \
        }                                                       \
    } while (0)

static void c7_prom_set(uint32_t *prom_buf, int index, const char *string, ...)
    G_GNUC_PRINTF(3, 4);

static void c7_prom_set(uint32_t *prom_buf, int index, const char *string, ...)
{
    va_list ap;
    uint32_t table_addr;

    if (index >= C7_ENVP_NB_ENTRIES) {
        return;
    }

    if (string == NULL) {
        prom_buf[index] = 0;
        return;
    }

    table_addr = sizeof(uint32_t) * C7_ENVP_NB_ENTRIES +
                 index * C7_ENVP_ENTRY_SIZE;
    prom_buf[index] = tswap32(C7_ENVP_VADDR + table_addr);

    va_start(ap, string);
    vsnprintf((char *)prom_buf + table_addr, C7_ENVP_ENTRY_SIZE, string, ap);
    va_end(ap);
}

static void c7_load_prom(const char *kernel_filename, const char *cmdline)
{
    uint32_t *prom_buf;
    int prom_index = 0;
    const long prom_size =
        C7_ENVP_NB_ENTRIES * (sizeof(uint32_t) + C7_ENVP_ENTRY_SIZE);

    prom_buf = g_malloc0(prom_size);
    c7_prom_set(prom_buf, prom_index++, "%s", kernel_filename);
    c7_prom_set(prom_buf, prom_index++, "%s", cmdline);
    c7_prom_set(prom_buf, prom_index++, "memsize");
    c7_prom_set(prom_buf, prom_index++, "%u", (unsigned)C7_RAM_SIZE);
    c7_prom_set(prom_buf, prom_index++, "ememsize");
    c7_prom_set(prom_buf, prom_index++, "%u", (unsigned)C7_RAM_SIZE);
    c7_prom_set(prom_buf, prom_index++, NULL);

    rom_add_blob_fixed("archer-c7.prom", prom_buf, prom_size, C7_ENVP_PADDR);
    g_free(prom_buf);

    c7_prom.kernel_entry = C7_KERNEL_ENTRY;
    c7_prom.stack = C7_ENVP_VADDR - 64;
    c7_prom.argv = C7_ENVP_VADDR;
    c7_prom.envp = C7_ENVP_VADDR;
    c7_prom.ram_size = C7_RAM_SIZE;
}

static uint64_t qca955x_regs_read(void *opaque, hwaddr offset, unsigned size)
{
    Qca955xRegs *s = opaque;

    if (offset + size > sizeof(s->regs) || size != 4) {
        qemu_log_mask(LOG_UNIMP, "%s: unsupported read offset=0x%"
                      HWADDR_PRIx " size=%u\n", s->name, offset, size);
        return 0;
    }

    return s->regs[offset >> 2] | s->read_or_mask[offset >> 2];
}

static void qca955x_regs_update_irq(Qca955xRegs *s)
{
    uint32_t status;
    uint32_t enable;

    if (!s->irq) {
        return;
    }

    status = s->regs[s->irq_status_offset >> 2];
    enable = s->regs[s->irq_enable_offset >> 2];
    qemu_set_irq(s->irq, !!(status & enable));
}

static void qca955x_regs_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    Qca955xRegs *s = opaque;

    if (offset + size > sizeof(s->regs) || size != 4) {
        qemu_log_mask(LOG_UNIMP, "%s: unsupported write offset=0x%"
                      HWADDR_PRIx " size=%u value=0x%" PRIx64 "\n",
                      s->name, offset, size, value);
        return;
    }

    s->regs[offset >> 2] = value & ~s->write_self_clear_mask[offset >> 2];

    if (offset == s->irq_status_offset || offset == s->irq_enable_offset) {
        qca955x_regs_update_irq(s);
    }
}

static const MemoryRegionOps qca955x_regs_ops = {
    .read = qca955x_regs_read,
    .write = qca955x_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void qca955x_misc_irq_set(void *opaque, int n, int level)
{
    Qca955xMiscIrqLine *line = opaque;
    Qca955xRegs *intc = line->intc;
    uint32_t *status = &intc->regs[intc->irq_status_offset >> 2];

    if (level) {
        *status |= line->status_bit;
    } else {
        *status &= ~line->status_bit;
    }

    qca955x_regs_update_irq(intc);
}

static hwaddr qca955x_gmac_dma_addr(uint32_t addr)
{
    return addr & 0x1fffffff;
}

static uint32_t qca955x_gmac_dma_ld32(hwaddr addr)
{
    uint8_t buf[4];

    cpu_physical_memory_read(qca955x_gmac_dma_addr(addr), buf, sizeof(buf));
    return ldl_be_p(buf);
}

static void qca955x_gmac_dma_st32(hwaddr addr, uint32_t value)
{
    uint8_t buf[4];

    stl_be_p(buf, value);
    cpu_physical_memory_write(qca955x_gmac_dma_addr(addr), buf, sizeof(buf));
}

static void qca955x_gmac_update_irq(Qca955xGmacState *s)
{
    uint32_t status = s->regs[QCA955X_GMAC_REG_DMA_INTR >> 2];
    uint32_t mask = s->regs[QCA955X_GMAC_REG_DMA_INTR_MASK >> 2];

    qemu_set_irq(s->irq, !!(status & mask));
}

static void qca955x_gmac_set_irq_status(Qca955xGmacState *s, uint32_t bits)
{
    s->regs[QCA955X_GMAC_REG_DMA_INTR >> 2] |= bits;
    qca955x_gmac_update_irq(s);
}

static void qca955x_gmac_init_phy(Qca955xGmacState *s)
{
    int phy;

    for (phy = 0; phy < 32; phy++) {
        s->phy_regs[phy][QCA955X_PHY_BMCR] = 0x1140;
        s->phy_regs[phy][QCA955X_PHY_BMSR] = 0x786d;
        s->phy_regs[phy][QCA955X_PHY_ID1] = 0x004d;
        s->phy_regs[phy][QCA955X_PHY_ID2] = 0xd072;
        s->phy_regs[phy][QCA955X_PHY_ANAR] = 0x01e1;
        s->phy_regs[phy][QCA955X_PHY_ANLPAR] = 0xcde1;
        s->phy_regs[phy][QCA955X_PHY_SPEC_STATUS] = 0xac00;
    }

    /*
     * The C7 v2 has the QCA9558 GMAC connected to an external S17/QCA8337
     * switch. The switch register file is accessed through the standard S17
     * MDIO window: PHY 0x18 register 0 latches address bits [18:9], and
     * PHYs 0x10..0x17 expose 16-bit halves of 32-bit switch registers.
     */
    s->switch_regs[QCA955X_S17_REG_MASK_CTRL >> 2] = 0x00001301;
    s->switch_regs[QCA955X_S17_REG_GLOBAL_INT_STATUS >> 2] = 0x3f000000;
    for (phy = 0; phy <= 6; phy++) {
        s->switch_regs[(QCA955X_S17_REG_PORT_STATUS_BASE + phy * 4) >> 2] =
            0x0000007f;
    }
    s->s17_addr = 0;
}

static bool qca955x_gmac_s17_half_addr(Qca955xGmacState *s, unsigned phy,
                                       unsigned reg, uint32_t *byte_addr,
                                       unsigned *shift)
{
    uint32_t word_addr;
    uint32_t addr;

    if (phy < QCA955X_S17_DATA_PHY_BASE ||
        phy > QCA955X_S17_DATA_PHY_LAST) {
        return false;
    }

    word_addr = ((phy & 0x7) << 5) | (reg & 0x1f);
    addr = ((s->s17_addr & 0x3ff) << 9) | (word_addr << 1);
    if ((addr >> 2) >= ARRAY_SIZE(s->switch_regs)) {
        return false;
    }

    *byte_addr = addr;
    *shift = (addr & 0x2) ? 16 : 0;
    return true;
}

static uint32_t qca955x_gmac_s17_read_reg(Qca955xGmacState *s,
                                          uint32_t byte_addr)
{
    uint32_t reg = byte_addr & ~3u;
    uint32_t value = s->switch_regs[reg >> 2];

    switch (reg) {
    case QCA955X_S17_REG_MASK_CTRL:
        value |= 0x00001301;
        value &= ~BIT(31);
        break;
    case QCA955X_S17_REG_GLOBAL_INT_STATUS:
        value |= 0x3f000000;
        break;
    case QCA955X_S17_REG_VTU_FUNC1:
        /*
         * The S17/AR8337 table-function BUSY bit is self-clearing after the
         * requested operation completes. The driver polls this bit during
         * SSDK init and VLAN flushes.
         */
        value &= ~BIT(31);
        s->switch_regs[reg >> 2] = value;
        break;
    default:
        if (reg >= QCA955X_S17_REG_PORT_STATUS_BASE &&
            reg < QCA955X_S17_REG_PORT_STATUS_BASE + 7 * 4 &&
            ((reg - QCA955X_S17_REG_PORT_STATUS_BASE) % 4) == 0) {
            value |= 0x0000007f;
        }
        break;
    }

    return value;
}

static uint16_t qca955x_gmac_mdio_read(Qca955xGmacState *s, uint32_t address)
{
    unsigned phy = (address >> 8) & 0x1f;
    unsigned reg = address & 0x1f;
    uint32_t byte_addr;
    unsigned shift;
    uint16_t value;

    if (phy == QCA955X_S17_ADDR_PHY && reg == QCA955X_S17_ADDR_REG) {
        C7_GMAC_TRACE("c7.gmac%u mdio-read s17-addr value=%#x\n",
                      s->unit, s->s17_addr);
        return s->s17_addr;
    }

    if (qca955x_gmac_s17_half_addr(s, phy, reg, &byte_addr, &shift)) {
        value = extract32(qca955x_gmac_s17_read_reg(s, byte_addr), shift, 16);
        C7_GMAC_TRACE("c7.gmac%u mdio-read s17 reg=%#x.%u value=%#x\n",
                      s->unit, byte_addr & ~3u, shift >> 4, value);
        return value;
    }

    value = s->phy_regs[phy][reg];
    C7_GMAC_TRACE("c7.gmac%u mdio-read phy=%u reg=%u value=%#x\n",
                  s->unit, phy, reg, value);
    return value;
}

static void qca955x_gmac_mdio_write(Qca955xGmacState *s, uint32_t address,
                                    uint16_t value)
{
    unsigned phy = (address >> 8) & 0x1f;
    unsigned reg = address & 0x1f;
    uint32_t byte_addr;
    unsigned shift;
    uint32_t *switch_reg;

    if (phy == QCA955X_S17_ADDR_PHY && reg == QCA955X_S17_ADDR_REG) {
        s->s17_addr = value & 0x3ff;
        s->phy_regs[phy][reg] = value;
        C7_GMAC_TRACE("c7.gmac%u mdio-write s17-addr value=%#x\n",
                      s->unit, value);
        return;
    }

    if (qca955x_gmac_s17_half_addr(s, phy, reg, &byte_addr, &shift)) {
        switch_reg = &s->switch_regs[byte_addr >> 2];
        *switch_reg &= ~(0xffffu << shift);
        *switch_reg |= (uint32_t)value << shift;
        C7_GMAC_TRACE("c7.gmac%u mdio-write s17 reg=%#x.%u value=%#x full=%#x\n",
                      s->unit, byte_addr & ~3u, shift >> 4, value,
                      *switch_reg);
        return;
    }

    s->phy_regs[phy][reg] = value;
    C7_GMAC_TRACE("c7.gmac%u mdio-write phy=%u reg=%u value=%#x\n",
                  s->unit, phy, reg, value);
    if (reg == QCA955X_PHY_BMCR && (value & BIT(15))) {
        s->phy_regs[phy][QCA955X_PHY_BMCR] = 0x1140;
        s->phy_regs[phy][QCA955X_PHY_BMSR] = 0x786d;
        s->phy_regs[phy][QCA955X_PHY_SPEC_STATUS] = 0xac00;
    }
}

static bool qca955x_gmac_rx_ready(Qca955xGmacState *s)
{
    uint32_t ctrl;

    if (!s->rx_desc || !(s->regs[QCA955X_GMAC_REG_DMA_RX_CTRL >> 2] & 1)) {
        return false;
    }

    ctrl = qca955x_gmac_dma_ld32(s->rx_desc + 4);
    return !!(ctrl & QCA955X_GMAC_DESC_EMPTY);
}

static bool qca955x_gmac_can_receive(NetClientState *nc)
{
    Qca955xGmacState *s = qemu_get_nic_opaque(nc);

    return qca955x_gmac_rx_ready(s);
}

static hwaddr qca955x_gmac_next_desc(hwaddr desc)
{
    uint32_t next = qca955x_gmac_dma_ld32(desc + QCA955X_GMAC_DESC_NEXT);

    if (next != 0) {
        return qca955x_gmac_dma_addr(next);
    }
    return desc + QCA955X_GMAC_DESC_STRIDE;
}

static bool qca955x_gmac_is_vlan_frame(const uint8_t *buf, size_t size)
{
    return size >= 18 && lduw_be_p(buf + 12) == ETH_P_8021Q;
}

static size_t qca955x_s17_tag_lan_to_cpu(uint8_t *out, const uint8_t *in,
                                         size_t size)
{
    size_t len = MIN(size, (size_t)QCA955X_GMAC_MAX_FRAME);

    if (len < 14 || qca955x_gmac_is_vlan_frame(in, len) ||
        len + 4 > QCA955X_GMAC_MAX_FRAME) {
        memcpy(out, in, len);
        return len;
    }

    memcpy(out, in, 12);
    stw_be_p(out + 12, ETH_P_8021Q);
    /*
     * The S17 CPU-port tag uses the low bits as the physical source port.
     * The vendor GMAC receive path maps LAN switch ports onto VLAN 1 before
     * handing the frame to Linux.
     */
    stw_be_p(out + 14, QCA955X_S17_LAN_PORT);
    memcpy(out + 16, in + 12, len - 12);
    return len + 4;
}

static bool qca955x_s17_egress_cpu_to_external(uint8_t *frame, uint32_t *len)
{
    uint16_t tci;

    if (!qca955x_gmac_is_vlan_frame(frame, *len)) {
        return true;
    }

    tci = lduw_be_p(frame + 14);
    switch (tci & 0x0fff) {
    case QCA955X_S17_LAN_VID:
    case QCA955X_S17_WAN_VID:
        /*
         * The external S17 switch presents the LAN and WAN access ports as
         * untagged Ethernet. The CPU port carries VLAN-tagged frames to the
         * SoC, but frames leaving a physical access port should not be dropped
         * just because they are not LAN VID 1.
         */
        memmove(frame + 12, frame + 16, *len - 16);
        *len -= 4;
        return true;
    default:
        return false;
    }
}

static void qca955x_gmac_trace_packet(uint32_t unit, const char *dir,
                                      const uint8_t *frame, size_t len)
{
    uint16_t eth_type;
    uint16_t vid = 0xffff;
    size_t l3;

    if (!c7_gmac_trace_enabled() || len < 14) {
        return;
    }

    eth_type = lduw_be_p(frame + 12);
    l3 = 14;
    if (eth_type == ETH_P_8021Q && len >= 18) {
        vid = lduw_be_p(frame + 14) & 0x0fff;
        eth_type = lduw_be_p(frame + 16);
        l3 = 18;
    }

    if (eth_type == ETH_P_ARP && len >= l3 + 28 &&
        lduw_be_p(frame + l3 + 2) == ETH_P_IP &&
        frame[l3 + 4] == 6 && frame[l3 + 5] == 4) {
        const uint8_t *arp = frame + l3;
        uint16_t op = lduw_be_p(arp + 6);

        C7_GMAC_TRACE("c7.gmac%u %s arp vid=%d op=%u "
                      "sha=%02x:%02x:%02x:%02x:%02x:%02x spa=%u.%u.%u.%u "
                      "tha=%02x:%02x:%02x:%02x:%02x:%02x tpa=%u.%u.%u.%u\n",
                      unit, dir, vid == 0xffff ? -1 : (int)vid, op,
                      arp[8], arp[9], arp[10], arp[11], arp[12], arp[13],
                      arp[14], arp[15], arp[16], arp[17],
                      arp[18], arp[19], arp[20], arp[21], arp[22], arp[23],
                      arp[24], arp[25], arp[26], arp[27]);
        return;
    }

    if (eth_type == ETH_P_IP && len >= l3 + 20) {
        const uint8_t *ip = frame + l3;
        uint8_t ihl = (ip[0] & 0x0f) * 4;
        uint16_t ip_len = lduw_be_p(ip + 2);
        uint16_t ip_sum = 0xffff;

        if (ihl >= 20 && len >= l3 + ihl) {
            ip_sum = net_raw_checksum((uint8_t *)ip, ihl);
        }

        C7_GMAC_TRACE("c7.gmac%u %s ipv4 vid=%d proto=%u len=%u ihl=%u "
                      "ipck=%#x %u.%u.%u.%u>%u.%u.%u.%u\n",
                      unit, dir, vid == 0xffff ? -1 : (int)vid, ip[9],
                      ip_len, ihl, ip_sum,
                      ip[12], ip[13], ip[14], ip[15],
                      ip[16], ip[17], ip[18], ip[19]);
        if (ip[9] == 6 && ihl >= 20 && ip_len >= ihl + 20 &&
            len >= l3 + ip_len) {
            const uint8_t *tcp = ip + ihl;
            uint16_t tcp_len = ip_len - ihl;
            uint8_t thl = (tcp[12] >> 4) * 4;
            uint16_t tcp_sum = 0xffff;

            if (thl >= 20 && tcp_len >= thl) {
                tcp_sum = net_checksum_tcpudp(tcp_len, ip[9],
                                              (uint8_t *)ip + 12,
                                              (uint8_t *)tcp);
            }

            C7_GMAC_TRACE("c7.gmac%u %s tcp sport=%u dport=%u flags=%#x "
                          "seq=%#x ack=%#x win=%u thl=%u tcpck=%#x\n",
                          unit, dir, lduw_be_p(tcp), lduw_be_p(tcp + 2),
                          tcp[13], ldl_be_p(tcp + 4), ldl_be_p(tcp + 8),
                          lduw_be_p(tcp + 14), thl, tcp_sum);
        }
        return;
    }

    if (eth_type == ETH_P_IPV6) {
        C7_GMAC_TRACE("c7.gmac%u %s ipv6 vid=%d\n",
                      unit, dir, vid == 0xffff ? -1 : (int)vid);
    }
}

static ssize_t qca955x_gmac_receive(NetClientState *nc, const uint8_t *buf,
                                    size_t size)
{
    Qca955xGmacState *s = qemu_get_nic_opaque(nc);
    uint8_t frame[QCA955X_GMAC_MAX_FRAME];
    hwaddr desc = s->rx_desc;
    uint32_t data;
    uint32_t ctrl;
    uint32_t len;
    uint32_t dma_len;

    if (!qca955x_gmac_rx_ready(s)) {
        C7_GMAC_TRACE("c7.gmac%u rx drop-not-ready size=%zu rx_desc=%#"
                      HWADDR_PRIx " rx_ctrl=%#x intr=%#x mask=%#x\n",
                      s->unit, size, s->rx_desc,
                      s->regs[QCA955X_GMAC_REG_DMA_RX_CTRL >> 2],
                      s->regs[QCA955X_GMAC_REG_DMA_INTR >> 2],
                      s->regs[QCA955X_GMAC_REG_DMA_INTR_MASK >> 2]);
        return 0;
    }

    len = s->unit == 1 ? qca955x_s17_tag_lan_to_cpu(frame, buf, size) :
          MIN(size, (size_t)QCA955X_GMAC_MAX_FRAME);
    if (s->unit != 1) {
        memcpy(frame, buf, len);
    }
    data = qca955x_gmac_dma_ld32(desc);
    ctrl = qca955x_gmac_dma_ld32(desc + 4);
    /*
     * The ag7240 RX path subtracts the Ethernet FCS from the descriptor byte
     * count before handing the skb upward.  Model the hardware-owned count,
     * otherwise short VLAN-tagged TCP frames are truncated before IP input.
     */
    dma_len = MIN(len + QCA955X_GMAC_FCS_LEN,
                  (uint32_t)((ctrl & 0x3fff) ? (ctrl & 0x3fff) :
                             QCA955X_GMAC_MAX_FRAME));

    C7_GMAC_TRACE("c7.gmac%u rx size=%zu len=%u dma_len=%u desc=%#"
                  HWADDR_PRIx
                  " data=%#x ctrl=%#x dst=%02x:%02x:%02x:%02x:%02x:%02x"
                  " src=%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x\n",
                  s->unit, size, len, dma_len, desc, data, ctrl,
                  frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
                  frame[6], frame[7], frame[8], frame[9], frame[10],
                  frame[11], frame[12], frame[13]);
    qca955x_gmac_trace_packet(s->unit, "rx", frame, len);

    cpu_physical_memory_write(qca955x_gmac_dma_addr(data), frame, len);
    qca955x_gmac_dma_st32(desc + 4,
                          (ctrl & ~QCA955X_GMAC_DESC_EMPTY) |
                          (dma_len & 0x3fff));

    s->rx_desc = qca955x_gmac_next_desc(desc);
    s->regs[QCA955X_GMAC_REG_DMA_RX_DESC >> 2] = s->rx_desc;
    s->regs[QCA955X_GMAC_REG_DMA_RX_STATUS >> 2] =
        QCA955X_GMAC_RX_STATUS_DONE | (1U << 16);
    qca955x_gmac_set_irq_status(s, QCA955X_GMAC_IRQ_RX);
    return len;
}

static const char *qca955x_gmac_rx_file_env(Qca955xGmacState *s)
{
    const char *path = NULL;
    char env_name[32];

    snprintf(env_name, sizeof(env_name), "ARCHER_C7_GMAC%u_RX_FILE", s->unit);
    path = getenv(env_name);
    if (!path || !path[0]) {
        path = getenv("ARCHER_C7_GMAC_RX_FILE");
    }
    return path && path[0] ? path : NULL;
}

static int64_t qca955x_gmac_rx_file_delay_ms(Qca955xGmacState *s)
{
    const char *env = NULL;
    char env_name[40];
    int64_t value = 0;

    snprintf(env_name, sizeof(env_name),
             "ARCHER_C7_GMAC%u_RX_DELAY_MS", s->unit);
    env = getenv(env_name);
    if (!env || !env[0]) {
        env = getenv("ARCHER_C7_GMAC_RX_DELAY_MS");
    }
    if (env && env[0] && qemu_strtoi64(env, NULL, 0, &value) == 0 &&
        value > 0) {
        return value;
    }
    return 0;
}

static void qca955x_gmac_rx_file_timer(void *opaque)
{
    Qca955xGmacState *s = opaque;
    g_autofree gchar *contents = NULL;
    GError *err = NULL;
    gsize len = 0;
    const char *path;

    if (s->rx_file_done) {
        return;
    }

    path = qca955x_gmac_rx_file_env(s);
    if (!path) {
        s->rx_file_done = true;
        return;
    }

    if (!qca955x_gmac_rx_ready(s)) {
        timer_mod(s->rx_file_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
        return;
    }

    if (!g_file_get_contents(path, &contents, &len, &err)) {
        warn_report("c7.gmac%u could not load RX frame '%s': %s",
                    s->unit, path, err->message);
        g_error_free(err);
        s->rx_file_done = true;
        return;
    }

    if (len == 0) {
        warn_report("c7.gmac%u RX frame '%s' is empty", s->unit, path);
        s->rx_file_done = true;
        return;
    }

    C7_GMAC_TRACE("c7.gmac%u rx-file inject path=%s len=%zu\n",
                  s->unit, path, (size_t)len);
    qca955x_gmac_receive(qemu_get_queue(s->nic),
                         (const uint8_t *)contents, len);
    s->rx_file_done = true;
}

static void qca955x_gmac_tx_process(Qca955xGmacState *s)
{
    uint8_t frame[QCA955X_GMAC_MAX_FRAME];
    hwaddr desc;
    int processed = 0;

    if (!s->tx_desc || !(s->regs[QCA955X_GMAC_REG_DMA_TX_CTRL >> 2] & 1)) {
        return;
    }

    desc = s->tx_desc;
    while (processed++ < 64) {
        uint32_t data = qca955x_gmac_dma_ld32(desc);
        uint32_t ctrl = qca955x_gmac_dma_ld32(desc + 4);
        uint32_t len = ctrl & 0x3fff;

        /*
         * RX descriptors are hardware-owned while EMPTY is set, but the TX
         * path clears the same bit to hand a descriptor to hardware.  The
         * driver's TX reap path waits for hardware to set it again.
         */
        if (ctrl & QCA955X_GMAC_DESC_EMPTY) {
            break;
        }

        if (len > QCA955X_GMAC_MAX_FRAME) {
            len = QCA955X_GMAC_MAX_FRAME;
        }
        if (len >= 14 && s->nic) {
            cpu_physical_memory_read(qca955x_gmac_dma_addr(data), frame, len);
            if (s->unit == 1 &&
                !qca955x_s17_egress_cpu_to_external(frame, &len)) {
                C7_GMAC_TRACE("c7.gmac%u tx drop unknown-vlan desc=%#"
                              HWADDR_PRIx " ctrl=%#x\n",
                              s->unit, desc, ctrl);
                goto complete;
            }
            C7_GMAC_TRACE("c7.gmac%u tx len=%u desc=%#" HWADDR_PRIx
                          " data=%#x ctrl=%#x dst=%02x:%02x:%02x:%02x:%02x:%02x"
                          " src=%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x\n",
                          s->unit, len, desc, data, ctrl,
                          frame[0], frame[1], frame[2], frame[3], frame[4],
                          frame[5], frame[6], frame[7], frame[8], frame[9],
                          frame[10], frame[11], frame[12], frame[13]);
            qca955x_gmac_trace_packet(s->unit, "tx", frame, len);
            qemu_send_packet(qemu_get_queue(s->nic), frame, len);
        }

complete:
        qca955x_gmac_dma_st32(desc + 4, ctrl | QCA955X_GMAC_DESC_EMPTY);
        desc = qca955x_gmac_next_desc(desc);
        qca955x_gmac_set_irq_status(s, QCA955X_GMAC_IRQ_TX);
    }

    s->tx_desc = desc;
    s->regs[QCA955X_GMAC_REG_DMA_TX_DESC >> 2] = desc;
    s->regs[QCA955X_GMAC_REG_DMA_TX_STATUS >> 2] = 0;
}

static uint64_t qca955x_gmac_read(void *opaque, hwaddr offset, unsigned size)
{
    Qca955xGmacState *s = opaque;

    if (offset + size > sizeof(s->regs) || size != 4) {
        qemu_log_mask(LOG_UNIMP, "qca955x.gmac%u: unsupported read offset=0x%"
                      HWADDR_PRIx " size=%u\n", s->unit, offset, size);
        return 0;
    }

    if (offset == QCA955X_GMAC_REG_MII_MGMT_IND) {
        return 0;
    }

    return s->regs[offset >> 2];
}

static void qca955x_gmac_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    Qca955xGmacState *s = opaque;
    uint32_t val = value;

    if (offset + size > sizeof(s->regs) || size != 4) {
        qemu_log_mask(LOG_UNIMP, "qca955x.gmac%u: unsupported write offset=0x%"
                      HWADDR_PRIx " size=%u value=0x%" PRIx64 "\n",
                      s->unit, offset, size, value);
        return;
    }

    switch (offset) {
    case QCA955X_GMAC_REG_MII_MGMT_ADDRESS:
        s->regs[offset >> 2] = val;
        break;
    case QCA955X_GMAC_REG_MII_MGMT_CMD:
        s->regs[offset >> 2] = val;
        if (val & QCA955X_MII_CMD_READ) {
            s->regs[QCA955X_GMAC_REG_MII_MGMT_STATUS >> 2] =
                qca955x_gmac_mdio_read(
                    s, s->regs[QCA955X_GMAC_REG_MII_MGMT_ADDRESS >> 2]);
        }
        break;
    case QCA955X_GMAC_REG_MII_MGMT_CTRL:
        s->regs[offset >> 2] = val;
        qca955x_gmac_mdio_write(
            s, s->regs[QCA955X_GMAC_REG_MII_MGMT_ADDRESS >> 2], val);
        break;
    case QCA955X_GMAC_REG_DMA_TX_DESC:
        s->tx_desc = qca955x_gmac_dma_addr(val);
        s->regs[offset >> 2] = val;
        C7_GMAC_TRACE("c7.gmac%u write tx_desc=%#x dma=%#" HWADDR_PRIx "\n",
                      s->unit, val, s->tx_desc);
        break;
    case QCA955X_GMAC_REG_DMA_TX_CTRL:
        s->regs[offset >> 2] = val;
        C7_GMAC_TRACE("c7.gmac%u write tx_ctrl=%#x tx_desc=%#"
                      HWADDR_PRIx "\n", s->unit, val, s->tx_desc);
        qca955x_gmac_tx_process(s);
        break;
    case QCA955X_GMAC_REG_DMA_TX_STATUS:
        s->regs[offset >> 2] &= ~val;
        C7_GMAC_TRACE("c7.gmac%u write tx_status=%#x tx_desc=%#"
                      HWADDR_PRIx "\n", s->unit, val, s->tx_desc);
        qca955x_gmac_tx_process(s);
        break;
    case QCA955X_GMAC_REG_DMA_RX_DESC:
        s->rx_desc = qca955x_gmac_dma_addr(val);
        s->regs[offset >> 2] = val;
        C7_GMAC_TRACE("c7.gmac%u write rx_desc=%#x dma=%#" HWADDR_PRIx "\n",
                      s->unit, val, s->rx_desc);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case QCA955X_GMAC_REG_DMA_RX_CTRL:
        s->regs[offset >> 2] = val;
        C7_GMAC_TRACE("c7.gmac%u write %s=%#x rx_desc=%#" HWADDR_PRIx "\n",
                      s->unit,
                      "rx_ctrl", val, s->rx_desc);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case QCA955X_GMAC_REG_DMA_RX_STATUS:
        s->regs[offset >> 2] &= ~val;
        C7_GMAC_TRACE("c7.gmac%u write rx_status=%#x rx_desc=%#"
                      HWADDR_PRIx "\n", s->unit, val, s->rx_desc);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case QCA955X_GMAC_REG_DMA_INTR_MASK:
        s->regs[offset >> 2] = val;
        C7_GMAC_TRACE("c7.gmac%u write intr_mask=%#x intr=%#x\n",
                      s->unit, val,
                      s->regs[QCA955X_GMAC_REG_DMA_INTR >> 2]);
        qca955x_gmac_update_irq(s);
        break;
    case QCA955X_GMAC_REG_DMA_INTR:
        s->regs[offset >> 2] &= ~val;
        C7_GMAC_TRACE("c7.gmac%u ack intr=%#x remaining=%#x mask=%#x\n",
                      s->unit, val,
                      s->regs[QCA955X_GMAC_REG_DMA_INTR >> 2],
                      s->regs[QCA955X_GMAC_REG_DMA_INTR_MASK >> 2]);
        qca955x_gmac_update_irq(s);
        break;
    default:
        s->regs[offset >> 2] = val;
        break;
    }
}

static const MemoryRegionOps qca955x_gmac_ops = {
    .read = qca955x_gmac_read,
    .write = qca955x_gmac_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static NetClientInfo qca955x_gmac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = qca955x_gmac_can_receive,
    .receive = qca955x_gmac_receive,
};

static void qca955x_gmac_reset(DeviceState *dev)
{
    Qca955xGmacState *s = QCA955X_GMAC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->rx_desc = 0;
    s->tx_desc = 0;
    s->rx_file_done = false;
    s->regs[QCA955X_GMAC_REG_FIFO_CFG0 >> 2] = 0x00001fff;
    s->regs[QCA955X_GMAC_REG_FIFO_CFG1 >> 2] = 0x00001000;
    s->regs[QCA955X_GMAC_REG_FIFO_CFG2 >> 2] = 0x015500aa;
    s->regs[QCA955X_GMAC_REG_FIFO_CFG3 >> 2] = 0x01f00140;
    s->regs[QCA955X_GMAC_REG_FIFO_CFG4 >> 2] = 0x00000fff;
    s->regs[QCA955X_GMAC_REG_FIFO_CFG5 >> 2] = 0x00007ff0;
    qca955x_gmac_init_phy(s);
    qca955x_gmac_update_irq(s);
    if (s->rx_file_timer && qca955x_gmac_rx_file_env(s)) {
        timer_mod(s->rx_file_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                  qca955x_gmac_rx_file_delay_ms(s));
    }
}

static void qca955x_gmac_realize(DeviceState *dev, Error **errp)
{
    Qca955xGmacState *s = QCA955X_GMAC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &qca955x_gmac_ops, s,
                          TYPE_QCA955X_GMAC, QCA955X_GMAC_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&qca955x_gmac_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    if (qca955x_gmac_rx_file_env(s)) {
        s->rx_file_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                        qca955x_gmac_rx_file_timer, s);
    }
}

static const Property qca955x_gmac_properties[] = {
    DEFINE_PROP_UINT32("unit", Qca955xGmacState, unit, 0),
    DEFINE_NIC_PROPERTIES(Qca955xGmacState, conf),
};

static void qca955x_gmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = qca955x_gmac_realize;
    device_class_set_legacy_reset(dc, qca955x_gmac_reset);
    device_class_set_props(dc, qca955x_gmac_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo qca955x_gmac_type_info = {
    .name = TYPE_QCA955X_GMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qca955xGmacState),
    .class_init = qca955x_gmac_class_init,
};

static void qca955x_gmac_register_types(void)
{
    type_register_static(&qca955x_gmac_type_info);
}

type_init(qca955x_gmac_register_types)

static void c7_spi_transaction_reset(ArcherC7Spi *s)
{
    s->cmd = 0;
    s->tx_byte = 0;
    s->tx_bits = 0;
    s->addr_bytes = 0;
    s->addr = 0;
    s->fast_read_dummy = false;
    s->rx_shift = 0;
    s->out_kind = C7_SPI_OUT_NONE;
    s->out_pos = 0;
    s->out_bit = 0;
}

static uint8_t c7_spi_output_byte(ArcherC7Spi *s)
{
    switch (s->out_kind) {
    case C7_SPI_OUT_ID:
        if (s->out_pos < sizeof(c7_spi_jedec_id)) {
            return c7_spi_jedec_id[s->out_pos];
        }
        return 0xff;
    case C7_SPI_OUT_STATUS:
        return 0;
    case C7_SPI_OUT_FLASH:
        return s->flash[s->addr & (QCA955X_SPI_SIZE - 1)];
    case C7_SPI_OUT_NONE:
    default:
        return 0xff;
    }
}

static int c7_spi_next_miso_bit(ArcherC7Spi *s)
{
    uint8_t value = c7_spi_output_byte(s);
    int bit = (value >> (7 - s->out_bit)) & 1;

    s->out_bit++;
    if (s->out_bit == 8) {
        s->out_bit = 0;
        if (s->out_kind == C7_SPI_OUT_FLASH) {
            s->addr++;
        } else if (s->out_kind != C7_SPI_OUT_NONE) {
            s->out_pos++;
        }
    }
    return bit;
}

static void c7_spi_set_output(ArcherC7Spi *s, C7SpiOutKind kind)
{
    s->out_kind = kind;
    s->out_pos = 0;
    s->out_bit = 0;
}

static void c7_spi_process_tx_byte(ArcherC7Spi *s, uint8_t value)
{
    if (s->cmd == 0) {
        s->cmd = value;
        switch (s->cmd) {
        case SPI_NOR_CMD_RDID:
            c7_spi_set_output(s, C7_SPI_OUT_ID);
            break;
        case SPI_NOR_CMD_RDSR:
            c7_spi_set_output(s, C7_SPI_OUT_STATUS);
            break;
        case SPI_NOR_CMD_READ:
        case SPI_NOR_CMD_FAST_READ:
            s->addr = 0;
            s->addr_bytes = 0;
            s->fast_read_dummy = s->cmd == SPI_NOR_CMD_FAST_READ;
            break;
        default:
            c7_spi_set_output(s, C7_SPI_OUT_NONE);
            break;
        }
        return;
    }

    if (s->cmd == SPI_NOR_CMD_READ || s->cmd == SPI_NOR_CMD_FAST_READ) {
        if (s->addr_bytes < 3) {
            s->addr = (s->addr << 8) | value;
            s->addr_bytes++;
            if (s->addr_bytes == 3 && !s->fast_read_dummy) {
                c7_spi_set_output(s, C7_SPI_OUT_FLASH);
            }
            return;
        }

        if (s->fast_read_dummy) {
            s->fast_read_dummy = false;
            c7_spi_set_output(s, C7_SPI_OUT_FLASH);
        }
    }
}

static void c7_spi_clock_bit(ArcherC7Spi *s, int mosi)
{
    int miso = c7_spi_next_miso_bit(s);

    s->rx_shift = (s->rx_shift << 1) | miso;
    s->regs[AR71XX_SPI_REG_RDS >> 2] = s->rx_shift;

    s->tx_byte = (s->tx_byte << 1) | (mosi & 1);
    s->tx_bits++;
    if (s->tx_bits == 8) {
        c7_spi_process_tx_byte(s, s->tx_byte);
        s->tx_byte = 0;
        s->tx_bits = 0;
    }
}

static uint64_t c7_spi_direct_read(ArcherC7Spi *s, hwaddr offset, unsigned size)
{
    uint64_t value = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        value = (value << 8) |
            s->flash[(offset + i) & (QCA955X_SPI_SIZE - 1)];
    }
    return value;
}

static uint64_t c7_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    ArcherC7Spi *s = opaque;

    if ((s->regs[AR71XX_SPI_REG_FS >> 2] & AR71XX_SPI_FS_GPIO) &&
        offset < sizeof(s->regs) && size == 4) {
        return s->regs[offset >> 2];
    }

    if (offset + size <= QCA955X_SPI_SIZE && size <= 4) {
        return c7_spi_direct_read(s, offset, size);
    }

    qemu_log_mask(LOG_UNIMP, "qca955x.spi: unsupported read offset=0x%"
                  HWADDR_PRIx " size=%u\n", offset, size);
    return 0xffffffff;
}

static void c7_spi_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    ArcherC7Spi *s = opaque;
    bool new_cs_active;
    bool new_clk;

    if (!(s->regs[AR71XX_SPI_REG_FS >> 2] & AR71XX_SPI_FS_GPIO) &&
        offset >= sizeof(s->regs) && offset < QCA955X_SPI_SIZE) {
        return;
    }

    if (offset >= sizeof(s->regs) || size != 4) {
        qemu_log_mask(LOG_UNIMP, "qca955x.spi: unsupported write offset=0x%"
                      HWADDR_PRIx " size=%u value=0x%" PRIx64 "\n",
                      offset, size, value);
        return;
    }

    switch (offset) {
    case AR71XX_SPI_REG_FS:
    case AR71XX_SPI_REG_CTRL:
        s->regs[offset >> 2] = value;
        if (offset == AR71XX_SPI_REG_FS &&
            !(value & AR71XX_SPI_FS_GPIO)) {
            s->cs_active = false;
            s->last_clk = false;
            c7_spi_transaction_reset(s);
        }
        break;
    case AR71XX_SPI_REG_IOC:
        s->regs[offset >> 2] = value;
        new_cs_active = !(value & AR71XX_SPI_IOC_CS0);
        new_clk = !!(value & AR71XX_SPI_IOC_CLK);
        if (!s->cs_active && new_cs_active) {
            c7_spi_transaction_reset(s);
        }
        if (s->cs_active && !new_cs_active) {
            c7_spi_transaction_reset(s);
        }
        if (new_cs_active && !s->last_clk && new_clk) {
            c7_spi_clock_bit(s, value & AR71XX_SPI_IOC_DO);
        }
        s->cs_active = new_cs_active;
        s->last_clk = new_clk;
        break;
    case AR71XX_SPI_REG_RDS:
        s->regs[offset >> 2] = value;
        break;
    }
}

static const MemoryRegionOps c7_spi_ops = {
    .read = c7_spi_read,
    .write = c7_spi_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t c7_pcie_read_reg(uint32_t *regs, size_t regs_bytes,
                                 hwaddr offset, unsigned size)
{
    uint32_t data;

    if (offset + size > regs_bytes || size > 4) {
        return 0xffffffff;
    }

    data = regs[offset >> 2];
    if (size == 4) {
        return data;
    }

    data >>= (offset & 3) * 8;
    if (size == 2) {
        return data & 0xffff;
    }
    if (size == 1) {
        return data & 0xff;
    }

    return 0xffffffff;
}

static void c7_pcie_write_reg(uint32_t *regs, size_t regs_bytes,
                              hwaddr offset, uint64_t value, unsigned size)
{
    uint32_t *reg;
    unsigned shift;
    uint32_t mask;

    if (offset + size > regs_bytes || size > 4 || size == 0) {
        return;
    }

    reg = &regs[offset >> 2];
    if (size == 4) {
        *reg = value;
        return;
    }

    shift = (offset & 3) * 8;
    mask = size == 2 ? 0xffffu : 0xffu;
    *reg = (*reg & ~(mask << shift)) | ((value & mask) << shift);
}

static uint64_t c7_pcie_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    ArcherC7Pcie *s = opaque;

    return c7_pcie_read_reg(s->ctrl_regs, sizeof(s->ctrl_regs), offset, size);
}

static void c7_pcie_ctrl_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    ArcherC7Pcie *s = opaque;

    c7_pcie_write_reg(s->ctrl_regs, sizeof(s->ctrl_regs), offset, value, size);
}

static uint64_t c7_pcie_crp_read(void *opaque, hwaddr offset, unsigned size)
{
    ArcherC7Pcie *s = opaque;

    if (!s->present) {
        return 0xffffffff;
    }

    return c7_pcie_read_reg(s->crp_regs, sizeof(s->crp_regs), offset, size);
}

static void c7_pcie_crp_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    ArcherC7Pcie *s = opaque;

    if (!s->present) {
        return;
    }

    c7_pcie_write_reg(s->crp_regs, sizeof(s->crp_regs), offset, value, size);
}

static uint64_t c7_pcie_cfg_read(void *opaque, hwaddr offset, unsigned size)
{
    ArcherC7Pcie *s = opaque;

    if (!s->present) {
        return 0xffffffff;
    }

    return c7_pcie_read_reg(s->cfg_regs, sizeof(s->cfg_regs), offset, size);
}

static void c7_pcie_cfg_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    ArcherC7Pcie *s = opaque;

    if (!s->present) {
        return;
    }

    c7_pcie_write_reg(s->cfg_regs, sizeof(s->cfg_regs), offset, value, size);
}

static const MemoryRegionOps c7_pcie_ctrl_ops = {
    .read = c7_pcie_ctrl_read,
    .write = c7_pcie_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps c7_pcie_crp_ops = {
    .read = c7_pcie_crp_read,
    .write = c7_pcie_crp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps c7_pcie_cfg_ops = {
    .read = c7_pcie_cfg_read,
    .write = c7_pcie_cfg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void c7_pcie_init_config(ArcherC7Pcie *s)
{
    stl_le_p((uint8_t *)s->cfg_regs + 0x00,
             (PCI_DEVICE_ID_ATHEROS_AR9580 << 16) | PCI_VENDOR_ID_ATHEROS);
    stl_le_p((uint8_t *)s->cfg_regs + 0x04,
             PCI_STATUS_CAP_LIST << 16);
    stl_le_p((uint8_t *)s->cfg_regs + 0x08,
             PCI_CLASS_NETWORK_OTHER << 16);
    stl_le_p((uint8_t *)s->cfg_regs + 0x0c, 0x00000000);
    stl_le_p((uint8_t *)s->cfg_regs + 0x10, 0x00000004);
    stl_le_p((uint8_t *)s->cfg_regs + 0x2c,
             (PCI_DEVICE_ID_ATHEROS_AR9580 << 16) | PCI_VENDOR_ID_ATHEROS);
    stl_le_p((uint8_t *)s->cfg_regs + 0x34, 0x00000040);
    stl_le_p((uint8_t *)s->cfg_regs + 0x3c, 0x00000100);
    stl_le_p((uint8_t *)s->cfg_regs + 0x40,
             PCI_CAP_ID_PM | (0x50 << 8) | (0x0003 << 16));
    stl_le_p((uint8_t *)s->cfg_regs + 0x44, 0x00000000);
    stl_le_p((uint8_t *)s->cfg_regs + 0x50,
             PCI_CAP_ID_EXP | (0x0002 << 16));

    memcpy(s->crp_regs, s->cfg_regs, sizeof(s->crp_regs));
}

static void c7_pcie_create(MemoryRegion *sysmem, const char *name,
                           hwaddr ctrl_base, hwaddr crp_base,
                           hwaddr cfg_base, bool present)
{
    ArcherC7Pcie *s = g_new0(ArcherC7Pcie, 1);
    char *mr_name;

    s->present = present;
    if (present) {
        s->ctrl_regs[AR724X_PCI_REG_RESET >> 2] = AR724X_PCI_RESET_LINK_UP;
        c7_pcie_init_config(s);
    }

    mr_name = g_strdup_printf("%s.ctrl", name);
    memory_region_init_io(&s->ctrl, NULL, &c7_pcie_ctrl_ops, s, mr_name,
                          QCA955X_PCI_CTRL_SIZE);
    g_free(mr_name);
    memory_region_add_subregion(sysmem, ctrl_base, &s->ctrl);

    mr_name = g_strdup_printf("%s.crp", name);
    memory_region_init_io(&s->crp, NULL, &c7_pcie_crp_ops, s, mr_name,
                          QCA955X_PCI_CRP_SIZE);
    g_free(mr_name);
    memory_region_add_subregion(sysmem, crp_base, &s->crp);

    mr_name = g_strdup_printf("%s.cfg", name);
    memory_region_init_io(&s->cfg, NULL, &c7_pcie_cfg_ops, s, mr_name,
                          QCA955X_PCI_CFG_SIZE);
    g_free(mr_name);
    memory_region_add_subregion(sysmem, cfg_base, &s->cfg);
}

static void c7_load_file_into_flash(uint8_t *flash, const char *path,
                                    hwaddr offset, size_t max_size,
                                    const char *what)
{
    gchar *contents = NULL;
    gsize len = 0;
    GError *err = NULL;

    if (!g_file_get_contents(path, &contents, &len, &err)) {
        error_report("could not load Archer C7 %s '%s': %s",
                     what, path, err->message);
        g_error_free(err);
        exit(1);
    }

    if (len > max_size) {
        error_report("Archer C7 %s '%s' is too large: 0x%" PRIx64
                     " bytes, max 0x%zx",
                     what, path, (uint64_t)len, max_size);
        g_free(contents);
        exit(1);
    }

    memcpy(flash + offset, contents, len);
    g_free(contents);
}

static void c7_load_partition_from_env(uint8_t *flash, const char *env_name,
                                       hwaddr offset, size_t max_size,
                                       const char *what)
{
    const char *path = getenv(env_name);

    if (!path || !path[0]) {
        return;
    }

    c7_load_file_into_flash(flash, path, offset, max_size, what);
}

static Qca955xRegs *qca955x_regs_create(MemoryRegion *sysmem,
                                        const char *name,
                                        hwaddr base,
                                        hwaddr size)
{
    Qca955xRegs *s = g_new0(Qca955xRegs, 1);

    s->name = name;
    memory_region_init_io(&s->iomem, NULL, &qca955x_regs_ops,
                          s, name, size);
    memory_region_add_subregion(sysmem, base, &s->iomem);
    return s;
}

static Qca955xRegs *qca955x_create_soc_regs(MemoryRegion *sysmem,
                                            qemu_irq misc_irq)
{
    Qca955xRegs *pll;
    Qca955xRegs *reset;
    Qca955xRegs *gpio;
    Qca955xRegs *gmac_glue;

    Qca955xRegs *ddr;

    ddr = qca955x_regs_create(sysmem, "qca955x.ddr",
                              QCA955X_DDR_BASE, 0x100);
    /*
     * The GMAC driver uses the DDR flush registers to order DMA ring memory:
     * write bit 0, then poll until hardware clears it after the flush.
     */
    ddr->write_self_clear_mask[QCA955X_DDR_REG_FLUSH_GE0 >> 2] = BIT(0);
    ddr->write_self_clear_mask[QCA955X_DDR_REG_FLUSH_GE1 >> 2] = BIT(0);
    qca955x_regs_create(sysmem, "qca955x.usb-ctrl",
                        QCA955X_USB_CTRL_BASE, 0x100);

    gpio = qca955x_regs_create(sysmem, "qca955x.gpio",
                               QCA955X_GPIO_BASE, 0x100);
    gpio->regs[AR71XX_GPIO_REG_OE >> 2] = 0x00000000;
    gpio->regs[AR71XX_GPIO_REG_IN >> 2] = 0x00ffffff;
    gpio->regs[AR71XX_GPIO_REG_OUT >> 2] = 0x00000000;

    pll = qca955x_regs_create(sysmem, "qca955x.pll",
                              QCA955X_PLL_BASE, 0x100);
    /*
     * 40 MHz ref clock, CPU PLL 720 MHz, DDR PLL 600 MHz, AHB 200 MHz.
     * These values follow Linux ath79's qca955x clock formulas.
     */
    pll->regs[QCA955X_PLL_CPU_CONFIG_REG >> 2] = 0x00001480;
    pll->regs[QCA955X_PLL_DDR_CONFIG_REG >> 2] = 0x00013c00;
    pll->regs[QCA955X_PLL_CLK_CTRL_REG >> 2] = 0x01010000;
    pll->regs[QCA955X_PLL_ETH_XMII_CONTROL_REG >> 2] = 0x00000000;
    pll->regs[QCA955X_PLL_ETH_SGMII_CONTROL_REG >> 2] = 0x00000000;
    pll->regs[QCA955X_PLL_ETH_SGMII_SERDES_REG >> 2] = 0x00000007;

    reset = qca955x_regs_create(sysmem, "qca955x.reset",
                                QCA955X_RESET_BASE, 0x1000);
    reset->irq = misc_irq;
    reset->irq_status_offset = QCA955X_RESET_REG_MISC_INT_STATUS;
    reset->irq_enable_offset = QCA955X_RESET_REG_MISC_INT_ENABLE;
    reset->regs[QCA955X_RESET_REG_MISC_INT_STATUS >> 2] = 0x00000000;
    reset->regs[QCA955X_RESET_REG_MISC_INT_ENABLE >> 2] = 0x00000000;
    reset->regs[QCA955X_RESET_REG_CPU_INT_STATUS >> 2] = 0x00000000;
    reset->regs[QCA955X_RESET_REG_GLOBAL_INT >> 2] = 0x00000000;
    reset->regs[QCA955X_RESET_REG_RESET_MODULE >> 2] = 0x00000000;
    reset->regs[QCA955X_RESET_REG_REV_ID >> 2] = QCA955X_REV_ID;
    reset->regs[QCA955X_RESET_REG_EXT_INT_STATUS >> 2] = 0x00000000;
    reset->regs[QCA955X_RESET_REG_BOOTSTRAP >> 2] = QCA955X_BOOTSTRAP_40MHZ;

    gmac_glue = qca955x_regs_create(sysmem, "qca955x.gmac-glue",
                                    QCA955X_GMAC_BASE,
                                    QCA955X_GMAC_GLUE_SIZE);
    /*
     * The GMAC glue block includes SGMII PLL/resistor-calibration status
     * fields. The vendor driver writes the control registers and then polls
     * these status bits; expose the post-calibration steady state so the
     * driver can complete the same bring-up sequence it runs on hardware.
     */
    gmac_glue->read_or_mask[QCA955X_GMAC_GLUE_SGMII_STATUS >> 2] =
        QCA955X_GMAC_GLUE_SGMII_LOCK;
    gmac_glue->read_or_mask[QCA955X_GMAC_GLUE_SGMII_RES_CAL >> 2] =
        QCA955X_GMAC_GLUE_RES_CAL_DONE;
    gmac_glue->regs[QCA955X_GMAC_GLUE_SGMII_STATUS >> 2] =
        QCA955X_GMAC_GLUE_SGMII_LOCK;
    gmac_glue->regs[QCA955X_GMAC_GLUE_SGMII_RES_CAL >> 2] =
        QCA955X_GMAC_GLUE_RES_CAL_DONE;

    qca955x_regs_create(sysmem, "qca955x.wdt",
                        QCA955X_WDT_BASE, QCA955X_WDT_SIZE);
    qca955x_regs_create(sysmem, "qca955x.otp",
                        QCA955X_OTP_BASE, QCA955X_OTP_SIZE);

    return reset;
}

static void c7_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;

    cpu_reset(CPU(cpu));

    env->CP0_Status &= ~((target_ulong)1 << CP0St_ERL);
    env->CP0_Status &= ~((target_ulong)1 << CP0St_BEV);
    /*
     * QCA9558 uses a 74Kc-class core whose WAIT state wakes for pending
     * interrupts even when the current Status bits do not make the interrupt
     * immediately takeable.  Linux idles in WAIT during sleeps; without this
     * bit QEMU can leave userland nanosleep paths parked forever.
     */
    env->CP0_Config7 |= (1U << CP0C7_WII);
    env->active_tc.gpr[4] = 2;
    env->active_tc.gpr[5] = c7_prom.argv;
    env->active_tc.gpr[6] = c7_prom.ram_size;
    env->active_tc.gpr[7] = QCA955X_SPI_SIZE / MiB;
    env->active_tc.gpr[29] = c7_prom.stack;
    env->active_tc.PC = c7_prom.kernel_entry;
}

static void c7_load_kernel(MachineState *machine)
{
    const char *cmdline = machine->kernel_cmdline;
    ssize_t kernel_size;

    if (!machine->kernel_filename) {
        error_report("archer-c7-v2 requires -kernel with the decompressed "
                     "Archer C7 kernel blob");
        exit(1);
    }

    if (!cmdline || !cmdline[0]) {
        cmdline = c7_default_cmdline;
    }

    kernel_size = load_image_targphys(machine->kernel_filename,
                                      C7_KERNEL_LOAD_PADDR,
                                      C7_RAM_SIZE, NULL);
    if (kernel_size < 0) {
        error_report("could not load Archer C7 kernel '%s'",
                     machine->kernel_filename);
        exit(1);
    }

    c7_load_prom(machine->kernel_filename, cmdline);
}

static void c7_load_flash(MachineState *machine, MemoryRegion *sysmem)
{
    ArcherC7Spi *spi = g_new0(ArcherC7Spi, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);

    memory_region_init_ram(flash, NULL, "archer-c7.spi-flash",
                           QCA955X_SPI_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, QCA955X_SPI_BASE, flash);

    spi->flash = memory_region_get_ram_ptr(flash);
    memset(spi->flash, 0xff, QCA955X_SPI_SIZE);
    spi->regs[AR71XX_SPI_REG_IOC >> 2] =
        AR71XX_SPI_IOC_CS0 | AR71XX_SPI_IOC_CS1 | AR71XX_SPI_IOC_CS2;

    memory_region_init_io(&spi->iomem, NULL, &c7_spi_ops, spi,
                          "qca955x.spi-regs", sizeof(spi->regs));
    memory_region_add_subregion_overlap(sysmem, QCA955X_SPI_BASE,
                                        &spi->iomem, 1);

    if (machine->firmware) {
        gchar *contents = NULL;
        gsize len = 0;
        GError *err = NULL;
        hwaddr offset = C7_FLASH_UPGRADE_OFF;
        size_t max_size = QCA955X_SPI_SIZE - C7_FLASH_UPGRADE_OFF;

        if (!g_file_get_contents(machine->firmware, &contents, &len, &err)) {
            error_report("could not load Archer C7 flash image '%s': %s",
                         machine->firmware, err->message);
            g_error_free(err);
            exit(1);
        }

        if (len == QCA955X_SPI_SIZE) {
            offset = 0;
            max_size = QCA955X_SPI_SIZE;
        }

        if (len > max_size) {
            error_report("Archer C7 flash image '%s' is too large",
                         machine->firmware);
            g_free(contents);
            exit(1);
        }
        memcpy(spi->flash + offset, contents, len);
        g_free(contents);
    }

    c7_load_partition_from_env(spi->flash, "ARCHER_C7_CONFIG_FILE",
                               C7_FLASH_CONFIG_OFF, C7_FLASH_CONFIG_SIZE,
                               "config partition");
    c7_load_partition_from_env(spi->flash, "ARCHER_C7_ART_FILE",
                               C7_FLASH_ART_OFF, C7_FLASH_ART_SIZE,
                               "ART partition");
}

static void qca955x_gmac_create(hwaddr base, qemu_irq irq, uint32_t unit,
                                bool match_default)
{
    DeviceState *dev = qdev_new(TYPE_QCA955X_GMAC);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    bool configured;

    qdev_prop_set_uint32(dev, "unit", unit);
    configured = qemu_configure_nic_device(dev, match_default, NULL);
    if (!configured && unit == 0) {
        g_autofree char *mac = g_strdup_printf("52:54:00:12:34:%02x", unit);

        object_property_set_str(OBJECT(dev), "mac", mac, &error_abort);
    }
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, base);
    sysbus_connect_irq(sbd, 0, irq);
}

static void mips_archer_c7_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram_low = g_new(MemoryRegion, 1);
    Clock *cpuclk;
    MIPSCPU *cpu;
    CPUMIPSState *env;
    Qca955xRegs *reset;
    Qca955xMiscIrqLine *uart_irq;

    if (machine->ram_size != C7_RAM_SIZE) {
        char *sz = size_to_str(C7_RAM_SIZE);
        error_report("archer-c7-v2 requires %s RAM", sz);
        g_free(sz);
        exit(1);
    }

    cpuclk = clock_new(OBJECT(machine), "qca955x.cpu-refclk");
    clock_set_hz(cpuclk, 720000000);
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk,
                                     TARGET_BIG_ENDIAN);
    env = &cpu->env;
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
    qemu_register_reset(c7_cpu_reset, cpu);

    memory_region_add_subregion(sysmem, C7_RAM_KSEG0_BASE, machine->ram);
    memory_region_init_alias(ram_low, NULL, "archer-c7.lowmem",
                             machine->ram, 0, C7_RAM_SIZE);
    memory_region_add_subregion(sysmem, 0, ram_low);

    reset = qca955x_create_soc_regs(sysmem, env->irq[QCA955X_CPU_IRQ_MISC]);
    uart_irq = g_new0(Qca955xMiscIrqLine, 1);
    uart_irq->intc = reset;
    uart_irq->status_bit = QCA955X_MISC_IRQ_UART;
    serial_mm_init(sysmem, QCA955X_UART_BASE, 2,
                   qemu_allocate_irq(qca955x_misc_irq_set, uart_irq, 0),
                   115200, serial_hd(0), DEVICE_NATIVE_ENDIAN);

    c7_load_flash(machine, sysmem);
    c7_load_kernel(machine);

    create_unimplemented_device("qca955x.wmac",
                                QCA955X_WMAC_BASE, 0x20000);
    create_unimplemented_device("qca955x.pcie-mem0",
                                QCA955X_PCI_MEM_BASE0, 0x02000000);
    create_unimplemented_device("qca955x.pcie-mem1",
                                QCA955X_PCI_MEM_BASE1, 0x02000000);
    c7_pcie_create(sysmem, "qca955x.pcie0",
                   QCA955X_PCI_CTRL_BASE0, QCA955X_PCI_CRP_BASE0,
                   QCA955X_PCI_CFG_BASE0, false);
    c7_pcie_create(sysmem, "qca955x.pcie1",
                   QCA955X_PCI_CTRL_BASE1, QCA955X_PCI_CRP_BASE1,
                   QCA955X_PCI_CFG_BASE1, true);
    qca955x_gmac_create(QCA955X_GE1_BASE, env->irq[5], 1, true);
    qca955x_gmac_create(QCA955X_GE0_BASE, env->irq[4], 0, false);
    create_unimplemented_device("qca955x.ehci0",
                                QCA955X_EHCI0_BASE, 0x1000);
    create_unimplemented_device("qca955x.ehci1",
                                QCA955X_EHCI1_BASE, 0x1000);
}

static void mips_archer_c7_machine_init(MachineClass *mc)
{
    mc->desc = "TP-Link Archer C7 v2 (QCA9558)";
    mc->init = mips_archer_c7_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("74Kc");
    mc->default_ram_size = C7_RAM_SIZE;
    mc->default_ram_id = "archer-c7.ram";
}

DEFINE_MACHINE("archer-c7-v2", mips_archer_c7_machine_init)
