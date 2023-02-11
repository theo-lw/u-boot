#include <common.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <dm.h>
#include <dm/pinctrl.h>
#include <dm/platform_data/serial_bcm2711_spi.h>
#include <errno.h>
#include <linux/bitops.h>
#include <linux/compiler.h>
#include <serial.h>
#include <watchdog.h>

struct GPIO {
  u32 GPFSEL[6];
  u32 : 32;
  u32 GPSET0;
  u32 GPSET1;
  u32 : 32;
  u32 GPCLR0;
  u32 GPCLR1;
  u32 : 32;
  u32 GPLEV0;
  u32 GPLEV1;
  u32 : 32;
  u32 GPEDS0;
  u32 GPEDS1;
  u32 : 32;
  u32 GPREN0;
  u32 GPREN1;
  u32 : 32;
  u32 GPFEN0;
  u32 GPFEN1;
  u32 : 32;
  u32 GPHEN0;
  u32 GPHEN1;
  u32 : 32;
  u32 GPLEN0;
  u32 GPLEN1;
  u32 : 32;
  u32 GPAREN0;
  u32 GPAREN1;
  u32 : 32;
  u32 GPAFEN0;
  u32 GPAFEN1;
  u32 _unused[21];
  u32 PUP_PDN_CNTRL_REG[4];
};

struct AUX {
  u32 IRQ;
  u32 ENABLES;
};

struct SPI {
  u32 CNTL0; // Control register 0
  u32 CNTL1; // Control register 1
  u32 STAT;  // Status
  u32 PEEK;  // Peek
  u32 _unused[4];
  u32 IO_REGa;     // Data
  u32 IO_REGb;     // Data
  u32 IO_REGc;     // Data
  u32 IO_REGd;     // Data
  u32 TXHOLD_REGa; // Extended Data
  u32 TXHOLD_REGb; // Extended Data
  u32 TXHOLD_REGc; // Extended Data
  u32 TXHOLD_REGd; // Extended Data
};

#define GPIO_BASE (0xFE000000 + 0x200000)
#define AUX_BASE (GPIO_BASE + 0x15000)

static volatile struct GPIO *const
    gpio __section(".data") = (struct GPIO *)(GPIO_BASE);
static volatile struct AUX *const
    aux __section(".data") = (struct AUX *)(AUX_BASE);
static volatile struct SPI *const spi[] __section(".data") = {
    (struct SPI *)(AUX_BASE + 0x80), (struct SPI *)(AUX_BASE + 0xC0)};

struct bcm2711_spi_priv {
  volatile struct GPIO *gpio;
  volatile struct AUX *aux;
  volatile struct SPI *spi[2];
};

/*************** GPIO ***************/

#define GPIO_INPUT 0x00
#define GPIO_OUTPUT 0x01
#define GPIO_ALTFN0 0x04
#define GPIO_ALTFN1 0x05
#define GPIO_ALTFN2 0x06
#define GPIO_ALTFN3 0x07
#define GPIO_ALTFN4 0x03
#define GPIO_ALTFN5 0x02

#define GPIO_NONE 0x00
#define GPIO_PUP 0x01
#define GPIO_PDP 0x02

static void setup_gpio(volatile struct GPIO *gpio, u32 pin, u32 setting,
                       u32 resistor) {
  u32 reg = pin / 10;
  u32 shift = (pin % 10) * 3;
  u32 status = gpio->GPFSEL[reg]; // read status
  status &= ~(7u << shift);       // clear bits
  status |= (setting << shift);   // set bits
  gpio->GPFSEL[reg] = status;     // write back

  reg = pin / 16;
  shift = (pin % 16) * 2;
  status = gpio->PUP_PDN_CNTRL_REG[reg]; // read status
  status &= ~(3u << shift);              // clear bits
  status |= (resistor << shift);         // set bits
  gpio->PUP_PDN_CNTRL_REG[reg] = status; // write back
}

void init_gpio(volatile struct GPIO *gpio) {
  setup_gpio(gpio, 18, GPIO_ALTFN4, GPIO_NONE);
  setup_gpio(gpio, 19, GPIO_ALTFN4, GPIO_NONE);
  setup_gpio(gpio, 20, GPIO_ALTFN4, GPIO_NONE);
  setup_gpio(gpio, 21, GPIO_ALTFN4, GPIO_NONE);
}

#define SPI_CNTL0_DOUT_HOLD_SHIFT 12
#define SPI_CNTL0_CS_SHIFT 17
#define SPI_CNTL0_SPEED_SHIFT 20

#define SPI_CNTL0_POSTINPUT 0x00010000
#define SPI_CNTL0_VAR_CS 0x00008000
#define SPI_CNTL0_VAR_WIDTH 0x00004000
#define SPI_CNTL0_Enable 0x00000800
#define SPI_CNTL0_In_Rising 0x00000400
#define SPI_CNTL0_Clear_FIFOs 0x00000200
#define SPI_CNTL0_Out_Rising 0x00000100
#define SPI_CNTL0_Invert_CLK 0x00000080
#define SPI_CNTL0_SO_MSB_FST 0x00000040
#define SPI_CNTL0_MAX_SHIFT 0x0000003F

#define SPI_CNTL1_CS_HIGH_SHIFT 8

#define SPI_CNTL1_Keep_Input 0x00000001
#define SPI_CNTL1_SI_MSB_FST 0x00000002
#define SPI_CNTL1_Done_IRQ 0x00000040
#define SPI_CNTL1_TX_EM_IRQ 0x00000080

#define SPI_STAT_TX_FIFO_MASK 0xFF000000
#define SPI_STAT_RX_FIFO_MASK 0x00FF0000
#define SPI_STAT_TX_FULL 0x00000400
#define SPI_STAT_TX_EMPTY 0x00000200
#define SPI_STAT_RX_FULL 0x00000100
#define SPI_STAT_RX_EMPTY 0x00000080
#define SPI_STAT_BUSY 0x00000040
#define SPI_STAT_BIT_CNT_MASK 0x0000003F

void init_spi(volatile struct AUX *aux, volatile struct SPI *spi[2],
              u32 channel) {
  u32 reg = aux->ENABLES;
  reg |= (2 << channel);
  aux->ENABLES = reg;
  spi[channel]->CNTL0 = SPI_CNTL0_Clear_FIFOs;
  u32 speed = (700000000 / (2 * 0x400000)) - 1; // for maximum bitrate 0x400000
  spi[channel]->CNTL0 = (speed << SPI_CNTL0_SPEED_SHIFT) | SPI_CNTL0_VAR_WIDTH |
                        SPI_CNTL0_Enable | SPI_CNTL0_In_Rising |
                        SPI_CNTL0_SO_MSB_FST;
  spi[channel]->CNTL1 = SPI_CNTL1_SI_MSB_FST;
}

static void spi_send_recv(volatile struct SPI *spi[2], u32 channel,
                          const char *sendbuf, size_t sendlen, char *recvbuf,
                          size_t recvlen) {
  size_t sendidx = 0;
  size_t recvidx = 0;
  while (sendidx < sendlen || recvidx < recvlen) {
    u32 data = 0;
    size_t count = 0;

    // prepare write data
    for (; sendidx < sendlen && count < 24; sendidx += 1, count += 8) {
      data |= (sendbuf[sendidx] << (16 - count));
    }
    data |= (count << 24);

    // always need to write something, otherwise no receive
    while (spi[channel]->STAT & SPI_STAT_TX_FULL)
      asm volatile("yield");
    if (sendidx < sendlen) {
      spi[channel]->TXHOLD_REGa = data; // keep chip-select active, more to come
    } else {
      spi[channel]->IO_REGa = data;
    }

    // read transaction
    while (spi[channel]->STAT & SPI_STAT_RX_EMPTY)
      asm volatile("yield");
    data = spi[channel]->IO_REGa;

    // process data, if needed, assume same byte count in transaction
    size_t max = (recvlen - recvidx) * 8;
    if (count > max)
      count = max;
    for (; count > 0; recvidx += 1, count -= 8) {
      recvbuf[recvidx] = (data >> (count - 8)) & 0xFF;
    }
  }
}

/*************** SPI ***************/

#define UART_RHR 0x00      // R
#define UART_THR 0x00      // W
#define UART_IER 0x01      // R/W
#define UART_IIR 0x02      // R
#define UART_FCR 0x02      // W
#define UART_LCR 0x03      // R/W
#define UART_MCR 0x04      // R/W
#define UART_LSR 0x05      // R
#define UART_MSR 0x06      // R
#define UART_SPR 0x07      // R/W
#define UART_TXLVL 0x08    // R
#define UART_RXLVL 0x09    // R
#define UART_IODir 0x0A    // R/W
#define UART_IOState 0x0B  // R/W
#define UART_IOIntEna 0x0C // R/W
#define UART_reserved 0x0D
#define UART_IOControl 0x0E // R/W
#define UART_EFCR 0x0F      // R/W

#define UART_DLL 0x00 // R/W - only accessible when EFR[4] = 1 and MCR[2] = 1
#define UART_DLH 0x01 // R/W - only accessible when EFR[4] = 1 and MCR[2] = 1
#define UART_EFR 0x02 // ?   - only accessible when LCR is 0xBF
#define UART_TCR 0x06 // R/W - only accessible when EFR[4] = 1 and MCR[2] = 1
#define UART_TLR 0x07 // R/W - only accessible when EFR[4] = 1 and MCR[2] = 1

// UART flags
#define UART_CHANNEL_SHIFT 1
#define UART_ADDR_SHIFT 3
#define UART_READ_ENABLE 0x80
#define UART_FCR_TX_FIFO_RESET 0x04
#define UART_FCR_RX_FIFO_RESET 0x02
#define UART_FCR_FIFO_EN 0x01
#define UART_LCR_DIV_LATCH_EN 0x80
#define UART_EFR_ENABLE_ENHANCED_FNS 0x10
#define UART_IOControl_RESET 0x08

static void uart_write_register(volatile struct SPI *spi[2], size_t spiChannel,
                                size_t uartChannel, char reg, char data) {
  char req[2] = {0};
  req[0] = (uartChannel << UART_CHANNEL_SHIFT) | (reg << UART_ADDR_SHIFT);
  req[1] = data;
  spi_send_recv(spi, spiChannel, req, 2, NULL, 0);
}

static char uart_read_register(volatile struct SPI *spi[2], size_t spiChannel,
                               size_t uartChannel, char reg) {
  char req[2] = {0};
  char res[2] = {0};
  req[0] = (uartChannel << UART_CHANNEL_SHIFT) | (reg << UART_ADDR_SHIFT) |
           UART_READ_ENABLE;
  spi_send_recv(spi, spiChannel, req, 2, res, 2);
  return res[1];
}

static void uart_init_channel(volatile struct SPI *spi[2], size_t spiChannel,
                              size_t uartChannel, size_t baudRate) {
  // set baud rate
  uart_write_register(spi, spiChannel, uartChannel, UART_LCR,
                      UART_LCR_DIV_LATCH_EN);
  u32 bauddiv = 14745600 / (baudRate * 16);
  uart_write_register(spi, spiChannel, uartChannel, UART_DLH,
                      (bauddiv & 0xFF00) >> 8);
  uart_write_register(spi, spiChannel, uartChannel, UART_DLL,
                      (bauddiv & 0x00FF));

  // set serial byte configuration: 8 bit, no parity, stop bits
  uart_write_register(spi, spiChannel, uartChannel, UART_LCR,
                      0x3 | (uartChannel << 2));

  // clear and enable fifos, (wait since clearing fifos takes time)
  uart_write_register(spi, spiChannel, uartChannel, UART_FCR,
                      UART_FCR_RX_FIFO_RESET | UART_FCR_TX_FIFO_RESET |
                          UART_FCR_FIFO_EN);
  for (int i = 0; i < 65535; ++i)
    asm volatile("yield");
}

void init_uart(volatile struct SPI *spi[2], u32 spiChannel) {
  uart_write_register(spi, spiChannel, 0, UART_IOControl,
                      UART_IOControl_RESET); // resets both channels
  uart_init_channel(spi, spiChannel, 0, 115200);
  uart_init_channel(spi, spiChannel, 1, 2400);
}

char uart_tell_read(volatile struct SPI *spi[2], size_t spiChannel,
                    size_t uartChannel) {
  return uart_read_register(spi, spiChannel, uartChannel, UART_RXLVL);
}

char uart_tell_write(volatile struct SPI *spi[2], size_t spiChannel,
                     size_t uartChannel) {
  return uart_read_register(spi, spiChannel, uartChannel, UART_TXLVL);
}

char uart_read(volatile struct SPI *spi[2], size_t spiChannel,
               size_t uartChannel) {
  return uart_read_register(spi, spiChannel, uartChannel, UART_RHR);
}

void uart_write(volatile struct SPI *spi[2], size_t spiChannel,
                size_t uartChannel, char c) {
  uart_write_register(spi, spiChannel, uartChannel, UART_THR, c);
}

char uart_getc(volatile struct SPI *spi[2], size_t spiChannel,
               size_t uartChannel) {
  while (uart_read_register(spi, spiChannel, uartChannel, UART_RXLVL) == 0)
    asm volatile("yield");
  char result = uart_read(spi, spiChannel, uartChannel);
  return result;
}

void uart_putc(volatile struct SPI *spi[2], size_t spiChannel,
               size_t uartChannel, char c) {
  while (uart_read_register(spi, spiChannel, uartChannel, UART_TXLVL) == 0)
    asm volatile("yield");
  uart_write(spi, spiChannel, uartChannel, c);
}

static int bcm2711_spi_serial_probe(struct udevice *dev) {
  struct bcm2711_spi_serial_plat *plat = dev_get_plat(dev);
  struct bcm2711_spi_priv *priv = dev_get_priv(dev);
  fdt_addr_t addr;

  addr = dev_read_addr(dev);
  if (addr == FDT_ADDR_T_NONE)
    return -EINVAL;
  plat->base = addr;
  priv->gpio = (struct GPIO *)(plat->base);
  priv->aux = (struct AUX *)(plat->base + 0x15000);
  priv->spi[0] = (struct SPI *)(plat->base + 0x15000 + 0x80);
  priv->spi[1] = (struct SPI *)(plat->base + 0x15000 + 0xC0);
  return 0;
}

static int bcm2711_spi_serial_pending(struct udevice *dev, bool input) {
  struct bcm2711_spi_priv *priv = dev_get_priv(dev);
  if (input) {
    schedule();
    return uart_tell_read(priv->spi, 0, 0);
  } else {
    return uart_tell_write(priv->spi, 0, 0) == 0 ? 1 : 0;
  }
}

static int bcm2711_spi_serial_setbrg(struct udevice *dev, int) {
  struct bcm2711_spi_priv *priv = dev_get_priv(dev);
  init_gpio(priv->gpio);
  init_spi(priv->aux, priv->spi, 0);
  init_uart(priv->spi, 0);
  return 0;
}

void DEBUG_init(void) {
  init_gpio(gpio);
  init_spi(aux, spi, 0);
  init_uart(spi, 0);
}

void DEBUG_putc(char ch) { uart_putc(spi, 0, 0, ch); }

static int bcm2711_spi_serial_getc(struct udevice *dev) {
  struct bcm2711_spi_priv *priv = dev_get_priv(dev);
  u32 data;

  /* Wait until there is data in the FIFO */
  data = uart_getc(priv->spi, 0, 0);

  return (int)data;
}

static int bcm2711_spi_serial_putc(struct udevice *dev, const char data) {
  struct bcm2711_spi_priv *priv = dev_get_priv(dev);

  uart_putc(priv->spi, 0, 0, data);

  return 0;
}

static const struct dm_serial_ops bcm2711_spi_serial_ops = {
    .putc = bcm2711_spi_serial_putc,
    .pending = bcm2711_spi_serial_pending,
    .getc = bcm2711_spi_serial_getc,
    .setbrg = bcm2711_spi_serial_setbrg,
};

static const struct udevice_id bcm2711_spi_serial_id[] = {
    {.compatible = "brcm,bcm2711-spi-uart"}, {}};

U_BOOT_DRIVER(serial_bcm2711_spi) = {
    .name = "serial_bcm2711_spi",
    .id = UCLASS_SERIAL,
    .of_match = of_match_ptr(bcm2711_spi_serial_id),
    .plat_auto = sizeof(struct bcm2711_spi_serial_plat),
    .probe = bcm2711_spi_serial_probe,
    .ops = &bcm2711_spi_serial_ops,
#if !CONFIG_IS_ENABLED(OF_CONTROL) || IS_ENABLED(CONFIG_OF_BOARD)
    .flags = DM_FLAG_PRE_RELOC,
#endif
    .priv_auto = sizeof(struct bcm2711_spi_priv),
};
