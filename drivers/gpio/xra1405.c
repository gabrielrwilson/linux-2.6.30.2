/**
 * XRA1405 16-bit GPIO expander using a SPI interface
 *
 * This driver registers the XRA1405 GPIO Expander as a set of GPIO into the
 * system using the GPIO abstraction layer. The abstraction controller will
 * assign a base address and then the number of defined GPIO pins will be
 * maped to the device, thus 16 more GPIOs identifiers will be added using
 * unsigned integer to the system in the range base..(base+GPIO_COUNT).
 *
 * The driver provides and registers functions to access the GPIOs using the
 * abstraction layer, resolving the access by itself using the SPI interface.
 * It also provides function to configure the GPIOs as input or output.
 *
 * The definition of the gpio_chip  GPIO controller can be found in
 * <linux/gpio/driver.h>.
 *
 * Copyright (C) 2015 - Polysat, Cal Poly
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
//#include <linux/of_irq.h>
//#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/processor.h>
#include <asm/hw_irq.h>
#include <linux/irqnr.h>
#include <linux/spi/xra1405.h>

#ifndef CONFIG_GENERIC_HARDIRQS
#warn not CONFIG_GENERIC_HARDIRQS
#endif

#define XRA_TYPE_1405       0

#define XRA1405_READ(r)     (0x80 | ((r) << 1))
#define XRA1405_WRITE(r)    (0x00 | ((r) << 1))

#define XRA1405_REG_COUNT   0x16    /* Total number of registers (0x00 to 0x15) */

#define XRA1405_REG_GSR     0x00    /* GPIO State Register */
#define XRA1405_REG_OCR     0x02    /* Output Control Register */
#define XRA1405_REG_PIR     0x04    /* Input Polarity Inversion Register */
#define XRA1405_REG_GCR     0x06    /* GPIO Configuration Register */
#define XRA1405_REG_PUR     0x08    /* Input Internal Pull-up Enable Register */
#define XRA1405_REG_IER     0x0A    /* Input Interrupt Enable Register */
#define XRA1405_REG_TSCR    0x0C    /* Output Three-State Control Register */
#define XRA1405_REG_ISR     0x0E    /* Input Interrupt Status Register */
#define XRA1405_REG_REIR    0x10    /* Input Rising Edge Interrupt Enable Register */
#define XRA1405_REG_FEIR    0x12    /* Input Falling Edge Interrupt Enable Register */
#define XRA1405_REG_IFR     0x14    /* Input Filter Enable Register */

/* Define more significative byte register position for each paramter */
#define XRA1405_REG_GSR_MSB  XRA1405_REG_GSR+1 
#define XRA1405_REG_OCR_MSB  XRA1405_REG_OCR+1
#define XRA1405_REG_PIR_MSB  XRA1405_REG_PIR+1
#define XRA1405_REG_GCR_MSB  XRA1405_REG_GCR+1
#define XRA1405_REG_PUR_MSB  XRA1405_REG_PUR+1
#define XRA1405_REG_IER_MSB  XRA1405_REG_IER+1
#define XRA1405_REG_TSCR_MSB XRA1405_REG_TSCR+1
#define XRA1405_REG_ISR_MSB  XRA1405_REG_ISR+1
#define XRA1405_REG_REIR_MSB XRA1405_REG_REIR+1
#define XRA1405_REG_FEIR_MSB XRA1405_REG_FEIR+1
#define XRA1405_REG_IFR_MSB  XRA1405_REG_IFR+1

/* Define cache positions that are half of the register number */
#define XRA1405_CACHE_GSR   XRA1405_REG_GSR/2
#define XRA1405_CACHE_OCR   XRA1405_REG_OCR/2
#define XRA1405_CACHE_PIR   XRA1405_REG_PIR/2
#define XRA1405_CACHE_GCR   XRA1405_REG_GCR/2
#define XRA1405_CACHE_PUR   XRA1405_REG_PUR/2
#define XRA1405_CACHE_IER   XRA1405_REG_IER/2
#define XRA1405_CACHE_TSCR  XRA1405_REG_TSCR/2
#define XRA1405_CACHE_ISR   XRA1405_REG_ISR/2
#define XRA1405_CACHE_REIR  XRA1405_REG_REIR/2
#define XRA1405_CACHE_FEIR  XRA1405_REG_FEIR/2
#define XRA1405_CACHE_IFR   XRA1405_REG_IFR/2

/* Total number of cache registers (16 bit) */
#define XRA1405_CACHE_COUNT XRA1405_REG_COUNT/2

/* Buffer size for asynchronous communications
 * Note that there is no need for more than 2 bytes per channel **/
#define XRA1405_MAX_BUF     8

#define XRA1405_MODULE_NAME "xra1405"  /* The module name to show in lsmod */
#define XRA1405_GPIO_NAME   "GPIO-XRA" /* The name to show on /proc/interrupts */

#define MAKE_ALIAS(base, n) base #n

struct xra1405 {
    u16         cache[XRA1405_CACHE_COUNT]; // Cache of the chip registers (in pairs)
    u32         irq_rise_mask;
    u32         irq_fall_mask;
    u32         irq_enabled_mask;
    u32         irq_allocated;
    u32         irq_soft_mask;
    int         irq;
    struct irq_domain   *irq_domain;

    /* Mutex definition */
    struct mutex        lock;
    // struct mutex        irq_lock;

    struct gpio_chip    chip;

    /* tx and rx buffers */
    struct spi_message   spi_msg;
    struct spi_transfer  spi_xfer[2];
    int			padding[64];
    u8                   tx_buff[3];
    u8                   rx_buff[3];

    // const struct xra1405_ops    *ops;
    void    *data;
    struct timeval *shared_irq_time;
};

/**
 * This lock class tells lockdep that TPIO irqs are in a different category
 * than their parents, so it won't report false recursion.
 */
// static struct lock_class_key gpio_lock_class;

static int xra1405_read16(struct xra1405 *xra, unsigned reg)
{
    u8 tx[1];
    int status;

    /* Use the macro to compute the read command byte */
    tx[0] = XRA1405_READ(reg);
    /* Write 8 bits and read 8 bits synchronously */
    status = spi_w8r16(xra->data, tx[0]);
    /* Return the status received, either the value readed or an error */
    return status;
}

static int xra1405_read(struct xra1405 *xra, unsigned reg)
{
    u8 tx[1];
    int status;

    /* Use the macro to compute the read command byte */
    tx[0] = XRA1405_READ(reg);
    /* Write 8 bits and read 8 bits synchronously */
    status = spi_w8r8(xra->data, tx[0]);
    /* Return the status received, either the value readed or an error */
    return status;
}

static int xra1405_write(struct xra1405 *xra, unsigned reg, unsigned val)
{
    u8 tx[2];

    tx[0] = XRA1405_WRITE(reg);
    tx[1] = val;
    /* Just write to the bus and don't try to read anything back */
    return spi_write(xra->data, tx, sizeof(tx));
}

static int xra1405_write16(struct xra1405 *xra, unsigned reg, u16 val)
{
    int retval;
    retval = xra1405_write(xra, reg, (val & 0x00FF));
    if(retval)
        return retval;

    retval = xra1405_write(xra, reg+1, (val & 0xFF00) >> 8);

    return retval;
}

/**
 * \brief Read all registers and store the value in the local cache
 */
static int xra1405_cache_regs(struct xra1405 *xra)
{
    u8 tmp = 0;
    u8 tx[1], rx[1], i;
    int status;

    /* Read each register and store them in the cache */
    for (i=0; i < XRA1405_REG_COUNT; i++) {
        /* Generate the read command */
        tx[0] = XRA1405_READ(i);
        status = spi_write_then_read(xra->data, tx, sizeof(tx),
                rx, sizeof(rx));
        if (status >= 0) {
            if (i%2 == 0) {
                /* Store the value in a temporary 8 bit variable */
                tmp = rx[0];
            } else {
                /* Store the 16bit value in the proper cache variable */
                xra->cache[(i-1)/2] = (rx[0] << 8) | (tmp & 0x00FF);
                /* Print out initial debug information */
                dev_dbg(xra->chip.dev, "%s: Initial value for 0x%02X is 0x%04X\n", 
                        XRA1405_MODULE_NAME, ((i-1)/2 << 1), xra->cache[(i-1)/2]);
            }
        } else {
            return -EINVAL;
        }
    }

    return status;
}

static void setup_async_spi(struct xra1405 *xra, u8 rd, int len)
{
    spi_message_init(&xra->spi_msg);
    xra->spi_msg.context = xra;

    memset(&xra->spi_xfer, 0, sizeof(xra->spi_xfer));

    xra->tx_buff[0] = XRA1405_READ(rd);
    xra->spi_xfer[0].tx_buf = xra->tx_buff;
    xra->spi_xfer[0].len = 1;
    spi_message_add_tail(&xra->spi_xfer[0], &xra->spi_msg);

    xra->rx_buff[0] = 0xBB;
    xra->rx_buff[1] = 0xCC;
    xra->rx_buff[2] = 0xDD;
    xra->spi_xfer[1].rx_buf = xra->rx_buff;
    xra->spi_xfer[1].len = len;
    spi_message_add_tail(&xra->spi_xfer[1], &xra->spi_msg);
}

/**
 * \brief Low level function to set or unset a bit into a register
 */
static int __xra1405_bit(struct xra1405 *xra, unsigned reg, unsigned offset,
       int set)
{
    int status;
    /* Get the 16-bit value we have cached */
    unsigned val = xra->cache[reg];

    /* Set or unset the value in the local cache as requested */
    if (set) {
        val |= BIT(offset);
        xra->cache[reg] = val;
    } else {
        val &= ~BIT(offset);
        xra->cache[reg] = val;
    }

    /**
     * Since we are treating the cache as 16 bit registers we need to compute
     * whenever the offset is indeed directed to the first or the second re-
     * gister.
     */
    if (offset < 8) {
        val = (xra->cache[reg] & 0xFF); // Read the 8 LSB
        reg *= 2;
    } else {
        val = (xra->cache[reg] & 0xFF00) >> 8; // Read the 8 MSB
        reg *= 2;
        reg++; // Write the value to the next register
    }

    /* Send the write command to the SPI chip */
    status = xra1405_write(xra, reg, val);

    return status;
}

/**
 * \brief Configures the specified GPIO as input
 */
static int xra1405_direction_input(struct gpio_chip *chip, unsigned offset)
{
    struct xra1405 *xra = container_of(chip, struct xra1405, chip);
    int status;

    // printk("direction input\n");

    mutex_lock(&xra->lock);

    /* Write the GPIO Configuration Register */
    status = __xra1405_bit(xra, XRA1405_CACHE_GCR, offset, 1); 
    if (xra->irq_allocated) {
        __xra1405_bit(xra, XRA1405_CACHE_REIR, offset, 1); 
        __xra1405_bit(xra, XRA1405_CACHE_FEIR, offset, 1); 
        __xra1405_bit(xra, XRA1405_CACHE_IFR, offset, 0); 
        __xra1405_bit(xra, XRA1405_CACHE_IER, offset, 1); 
    }

    mutex_unlock(&xra->lock);
    return status;
}

/**
 * \brief Get the input value from the GPIO
 */
static int xra1405_get(struct gpio_chip *chip, unsigned offset)
{
    struct xra1405 *xra = container_of(chip, struct xra1405, chip);
    int reg = XRA1405_REG_GSR;
    int status = 0;

    // printk("get\n");
    
    if (offset > 7)
        reg++; // We need to read the next register instead

    mutex_lock(&xra->lock);

    if (!xra->irq_allocated) {
       /* Read the status of the GSR register (actual value of the GPIO) */
        status = xra1405_read(xra, reg);
        if (status >= 0) {
            //printk("Read gpio %d: %02X\n", offset, status);
            /* Store the value readed into the cache */
            if (offset < 8)
                xra->cache[XRA1405_CACHE_GSR] = (xra->cache[XRA1405_CACHE_GSR] & 0xFF00) | status;
            else
                  xra->cache[XRA1405_CACHE_GSR] = (status << 8) | (xra->cache[XRA1405_CACHE_GSR] & 0xFF);
        
            status = !!(status & BIT(offset));
        }
    }

    mutex_unlock(&xra->lock);

    if (status >= 0)
        return !!(xra->cache[XRA1405_CACHE_GSR] & BIT(offset));

    return 0;
}

/**
 * \brief Configures the specified offset as output
 */
static int xra1405_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
    struct xra1405 *xra = container_of(chip, struct xra1405, chip);
    int status;

    mutex_lock(&xra->lock);

    if (xra->irq_allocated & BIT(offset))
        status = -EBUSY;
    /* Write the GPIO Configuration register */
    else {
        status = __xra1405_bit(xra, XRA1405_CACHE_GCR, offset, 0);
        __xra1405_bit(xra, XRA1405_CACHE_OCR, offset, value);
    }

    mutex_unlock(&xra->lock);

    return status;
}

static void xra1405_set(struct gpio_chip *chip, unsigned offset, int value)
{
    struct xra1405 *xra = container_of(chip, struct xra1405, chip);
    
    mutex_lock(&xra->lock);
    // Send the value to the Output Control Register
    if (xra->cache[XRA1405_CACHE_GCR] & BIT(offset))
        __xra1405_bit(xra, XRA1405_CACHE_OCR, offset, value);

    mutex_unlock(&xra->lock);
}

static void xra1405_isr_gsr_complete(void *context)
{
    struct xra1405 *xra = context;
    int i, handled_count = 0;

    if (xra->spi_msg.status < 0)
        goto err_data;

    xra->cache[XRA1405_CACHE_GSR] = (xra->rx_buff[1] << 8) | xra->rx_buff[0];

    // printk("ISR: %04X, GSR: %04X\n", xra->cache[XRA1405_CACHE_ISR],
    //                                 xra->cache[XRA1405_CACHE_GSR]);

    /* Fire any nested IRQ if it is enabled */
    for (i=0; i < xra->chip.ngpio; i++) {
        if (BIT(i) & xra->cache[XRA1405_CACHE_ISR] &&
                !(BIT(i) & xra->irq_soft_mask)) {
            generic_handle_irq(xra->chip.base + i);
            handled_count++;
        }
    }

    /* Un-mask the IRQ */
    enable_irq(xra->irq);

    return;

err_data:
    enable_irq(xra->irq);
    dev_err(xra->chip.dev,
            "Error during IRQ handler while reading the registers\n");
}

static void xra1405_isr_complete(void *context)
{
    struct xra1405 *xra = context;
    struct spi_device *spi = xra->data;

    xra->cache[XRA1405_CACHE_ISR] |= (xra->rx_buff[0] << 8);
    /* Prepare the message to request the register */
    setup_async_spi(xra, XRA1405_REG_GSR, 2);
    xra->spi_msg.complete = xra1405_isr_gsr_complete;
    spi_async(spi, &xra->spi_msg);
}

static void xra1405_isr_lsb_complete(void *context)
{
    struct xra1405 *xra = context;
    struct spi_device *spi = xra->data;

    xra->cache[XRA1405_CACHE_ISR] = xra->rx_buff[0];
    /* Prepare the message to request the register */
    setup_async_spi(xra, XRA1405_REG_ISR_MSB, 1);
    xra->spi_msg.complete = xra1405_isr_complete;
    spi_async(spi, &xra->spi_msg);
}

/* IRQ Related functionality */
static irqreturn_t xra1405_irq(int irq, void *data)
{
    struct xra1405 *xra = data;
    struct spi_device *spi = xra->data;

    /* Mask the IRQ to prevent recursion */
    disable_irq_nosync(xra->irq);

    if (xra->shared_irq_time)
        do_gettimeofday(xra->shared_irq_time);

    /* Send the read request for the Interrupt Status Register to identify the
     * IRQ number, but do not read it until the next read message  */
    setup_async_spi(xra, XRA1405_REG_ISR, 1);
    xra->spi_msg.complete = xra1405_isr_lsb_complete;
    spi_async(spi, &xra->spi_msg);

    return IRQ_HANDLED;
}

#if 0
static int xra1405_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
    struct xra1405 *xra = container_of(chip, struct xra1405, chip);
    return xra->chip.base + offset;
}
#endif

static void xra1405_irq_mask (unsigned int irq)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    unsigned int pos = irq - xra->chip.base;

    // printk("Mask %d\n", irq);
    xra->irq_soft_mask |= BIT(pos);
}

static void xra1405_irq_unmask (unsigned int irq)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    unsigned int pos = irq - xra->chip.base;

    xra->irq_soft_mask &= ~BIT(pos);
    // printk("Unmask %d\n", irq);
}

static int xra1405_irq_set_type (unsigned int irq, unsigned int type)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    unsigned int pos = irq - xra->chip.base;
    int status = 0;

    set_irq_handler(irq, &handle_level_irq);
    if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
        __xra1405_bit(xra, XRA1405_CACHE_FEIR, pos, 1); 
        __xra1405_bit(xra, XRA1405_CACHE_REIR, pos, 1); 
        __xra1405_bit(xra, XRA1405_CACHE_IFR, pos, 0); 
        // xra->cache[XRA1405_CACHE_REIR] &= ~BIT(pos);
        // xra->cache[XRA1405_CACHE_FEIR] &= ~BIT(pos);
        xra->irq_rise_mask |= BIT(pos);
        xra->irq_fall_mask |= BIT(pos);
        // printk("Setup %d RISING | FALLING\n", irq);
    } else if (type & IRQ_TYPE_EDGE_RISING) {
        __xra1405_bit(xra, XRA1405_CACHE_FEIR, pos, 0); 
        __xra1405_bit(xra, XRA1405_CACHE_REIR, pos, 1); 
        __xra1405_bit(xra, XRA1405_CACHE_IFR, pos, 0); 
        // xra->cache[XRA1405_CACHE_REIR] |= BIT(pos);
        // xra->cache[XRA1405_CACHE_FEIR] &= ~BIT(pos);
        xra->irq_rise_mask |= BIT(pos);
        xra->irq_fall_mask &= ~BIT(pos);
        // printk("Setup %d RISING\n", irq);
    } else if (type & IRQ_TYPE_EDGE_FALLING) {
        __xra1405_bit(xra, XRA1405_CACHE_FEIR, pos, 1); 
        __xra1405_bit(xra, XRA1405_CACHE_REIR, pos, 0); 
        __xra1405_bit(xra, XRA1405_CACHE_IFR, pos, 0); 
        // xra->cache[XRA1405_CACHE_FEIR] |= BIT(pos);
        // xra->cache[XRA1405_CACHE_REIR] &= ~BIT(pos);
        xra->irq_rise_mask &= ~BIT(pos);
        xra->irq_fall_mask |= BIT(pos);
        // printk("Setup %d FALLING\n", irq);
    } else if (type & IRQ_TYPE_NONE) {
        /* Just to make the driver support the configuration, but the chip
         * doesn't have a NONE, maybe configure it as default? */
        __xra1405_bit(xra, XRA1405_CACHE_IER, pos, 0); 
        __xra1405_bit(xra, XRA1405_CACHE_FEIR, pos, 0); 
        __xra1405_bit(xra, XRA1405_CACHE_REIR, pos, 0); 
        __xra1405_bit(xra, XRA1405_CACHE_IFR, pos, 0); 

        // xra->cache[XRA1405_CACHE_FEIR] &= ~BIT(pos);
        // xra->cache[XRA1405_CACHE_REIR] &= ~BIT(pos);
        xra->irq_rise_mask &= ~BIT(pos);
        xra->irq_fall_mask &= ~BIT(pos);
        /* Disable the interrution as well */
        // xra->cache[XRA1405_CACHE_IER] &= ~BIT(pos);
        // printk("Setup %d NONE\n", irq);
    } else
        return -EINVAL;

    return status;
}

static void xra1405_irq_enable(unsigned int irq)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    int pos = irq - xra->chip.base;

    __xra1405_bit(xra, XRA1405_CACHE_IER, pos, 1); 
    // printk("_enable %d\n", pos);
}

static void xra1405_irq_disable(unsigned int irq)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    int pos = irq - xra->chip.base;

    __xra1405_bit(xra, XRA1405_CACHE_IER, pos, 0); 
    // printk("_disable %d\n", pos);
}

static unsigned int xra1405_irq_startup(unsigned int irq)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    unsigned int pos = irq - xra->chip.base;
    unsigned int irq_was_allocated;
    int i;

    // printk("_startup %u\n", pos);

    mutex_lock(&xra->lock);
    irq_was_allocated = xra->irq_allocated;
    xra->irq_allocated |= BIT(pos);
    __xra1405_bit(xra, XRA1405_CACHE_GCR, pos, 1); 

    if (!irq_was_allocated) {
        for (i = 0; i < xra->chip.ngpio; i++) {
            if (xra->cache[XRA1405_CACHE_GCR] & BIT(i) &&
                                        !(xra->irq_allocated & BIT(i))) {
                __xra1405_bit(xra, XRA1405_CACHE_REIR, i, 1); 
                __xra1405_bit(xra, XRA1405_CACHE_FEIR, i, 1); 
                __xra1405_bit(xra, XRA1405_CACHE_IFR, i, 0); 
                __xra1405_bit(xra, XRA1405_CACHE_IER, i, 1); 
            }
        }
    }

    __xra1405_bit(xra, XRA1405_CACHE_IER, pos, 1); 
    xra->irq_soft_mask &= ~BIT(pos);

    mutex_unlock(&xra->lock);

    return 0;
}

static void xra1405_irq_shutdown(unsigned int irq)
{
    struct xra1405 *xra = (struct xra1405*)get_irq_chip_data(irq);
    unsigned int pos = irq - xra->chip.base;
    unsigned int irq_was_allocated;
    int i;

    // printk("_shutdown %u\n", pos);

    mutex_lock(&xra->lock);
    irq_was_allocated = xra->irq_allocated;
    xra->irq_allocated &= ~BIT(pos);
    __xra1405_bit(xra, XRA1405_CACHE_IER, pos, 0); 

    if (irq_was_allocated && !xra->irq_allocated) {
        for (i = 0; i < xra->chip.ngpio; i++) {
            if (xra->cache[XRA1405_CACHE_GCR] & BIT(i)) {
                __xra1405_bit(xra, XRA1405_CACHE_REIR, i, 0); 
                __xra1405_bit(xra, XRA1405_CACHE_FEIR, i, 0); 
                __xra1405_bit(xra, XRA1405_CACHE_IFR, i, 0); 
                __xra1405_bit(xra, XRA1405_CACHE_IER, i, 0); 
            }
        }
    }

    mutex_unlock(&xra->lock);
}

/* Define the IRQ chip structure with the functions we have created */
static struct irq_chip xra1405_irq_chip = {
    .name = XRA1405_GPIO_NAME,
    .mask = xra1405_irq_mask,
    .unmask = xra1405_irq_unmask,
    .enable = xra1405_irq_enable,
    .disable = xra1405_irq_disable,
    .set_type = xra1405_irq_set_type,
    .startup = xra1405_irq_startup,
    .shutdown = xra1405_irq_shutdown,
};

static int xra1405_irq_setup(struct xra1405 *xra)
{
    struct gpio_chip *chip = &xra->chip;
    int err, irq, j;

    // mutex_init(&xra->irq_lock);

    err = request_irq(xra->irq, xra1405_irq,
            IRQF_TRIGGER_LOW | IRQF_DISABLED, XRA1405_MODULE_NAME, xra);

    if (err != 0) {
        dev_err(chip->dev, "unable to request IRQ#%d: %d\n",
                xra->irq, err);
        return err;
    }

    for (j = 0; j < xra->chip.ngpio; j++) {
        irq = xra->chip.base + j;
        set_irq_chip_data(irq, xra);
        set_irq_chip(irq, &xra1405_irq_chip);
        xra->irq_soft_mask |= BIT(j);
#ifdef CONFIG_ARM
        set_irq_flags(irq, IRQF_VALID);
#else
        irq_set_noprobe(irq);
#endif
    }

    return 0;
}

static void xra1405_irq_teardown(struct xra1405 *xra)
{
    unsigned int i;
    int irq;

    for (i = 0; i < xra->chip.ngpio; i++) {
        irq = xra->chip.base + i;
        set_irq_flags(irq, 0);
    }

    /**
     * We are not calling free_irq here because the call to release_nodes will
     * call it later and we will receive a Warning "Trying to free already-free
     * IRQ #"
     */

    free_irq(xra->irq, xra);
}

static int xra1405_probe_one(struct xra1405 *xra, struct device *dev,
        void *data, u32 base)
{
    int status, i;
    char **names;

    mutex_init(&xra->lock);

    xra->data = data;

    xra->chip.direction_input = xra1405_direction_input;
    xra->chip.get = xra1405_get;
    xra->chip.direction_output = xra1405_direction_output;
    xra->chip.set = xra1405_set;
    xra->chip.dbg_show = NULL;
#ifdef CONFIG_OF
    xra->chip.of_gpio_n_cells = 2;
    xra->chip.of_node = dev->of_node;
#endif

    xra->chip.base = base;
    xra->chip.can_sleep = true;
    xra->chip.dev = dev;
    xra->chip.owner = THIS_MODULE;

    /* Set GPIO names */
    names = devm_kzalloc(dev, sizeof(char *) * xra->chip.ngpio,
            GFP_KERNEL);

    if (!names) {
        status = -ENOMEM;
        goto fail;
    }

    for (i=0; i < xra->chip.ngpio; i++)
        names[i] = kasprintf(GFP_KERNEL, "gpio%d", i + xra->chip.base);

    xra->chip.names = (char **)names;

    /* Force all GPIOs to be inputs at the beginning */
    status = xra1405_write16(xra, XRA1405_REG_GCR, 0xFFFF);
    if (status < 0)
        goto fail;

    /* Force all pull-ups to be disabled */
    status = xra1405_write16(xra, XRA1405_REG_PUR, 0x0000);
    if (status < 0)
        goto fail;

    /* Configure interrupts at all GPIOs to be falling edge */
    status = xra1405_write16(xra, XRA1405_REG_FEIR, 0xFFFF);
    if (status < 0)
        goto fail;

    /* Force all input filter to be enabled by default */
    status = xra1405_write16(xra, XRA1405_REG_IFR, 0xFFFF);
    if (status < 0)
        goto fail;

    /* Enable interrupts for GPIOs from the Device Tree  */
    status = xra1405_write16(xra, XRA1405_REG_IER, 0);
    if (status < 0)
        goto fail;

    /* Configure the interrupts that will be generated in the rising edge */
    status = xra1405_write16(xra, XRA1405_REG_REIR, 0);
    if (status < 0)
        goto fail;

    /* Configure the interrupts that will be generated in the falling edge */
    status = xra1405_write16(xra, XRA1405_REG_FEIR, 0);
    if (status < 0)
        goto fail;

    /* Read all registers to store them into the cache */
    status = xra1405_cache_regs(xra);
    if (status < 0)
        goto fail;

    /* Register the chip as GPIO */
    status = gpiochip_add(&xra->chip);
    if (status < 0)
        goto fail;

    if (xra->irq) {
        status = xra1405_irq_setup(xra);
        if (status) {
            if (status != -EINVAL)
                xra1405_irq_teardown(xra);
            goto fail;
        } 
    }

fail:
    if (status < 0) {
        dev_dbg(dev, "can't setup chip --> %d\n", status);
    }
    return status;

}

static int __devinit xra1405_probe(struct spi_device *spi)
{
    struct xra1405_platform_data    *pdata;
    struct xra1405      *xra;
    int                             status;
    u32                             base = 160;

    /* Try to look for the device within SPI bus */
    pdata = (struct xra1405_platform_data*)spi->dev.platform_data;
    if (pdata && gpio_is_valid(pdata->base))
       base = pdata->base;

    /* bits_per_word cannot be configured in platform data */
    spi->bits_per_word = 8;

    status = spi_setup(spi);
    if (status < 0)
        return -EINVAL;

    xra = kzalloc(sizeof(*xra), GFP_KERNEL);
    if (!xra)
        return -ENOMEM;

    spi_set_drvdata(spi, xra);

    /* Set the irq as the same as the SPI irq */
    xra->irq = spi->irq;
    if (pdata)
        xra->shared_irq_time = pdata->shared_irq_time;

    /* Register the number of GPIO the chip has */
    xra->chip.ngpio = 16;

    status = xra1405_probe_one(xra, &spi->dev, spi, base);
    if (status < 0)
        goto fail;

    /* Everything was OK, show the HW information */
    printk(KERN_NOTICE "%s: registered EXAR 16-bit GPIO expander with IRQ %d\n",
            XRA1405_MODULE_NAME, xra->irq);
    return 0;

fail:
    if (gpiochip_remove(&xra->chip))
        ;
    kfree(xra);
    return status;
}

static int __devexit xra1405_remove(struct spi_device *spi)
{
    struct xra1405 *xra = spi_get_drvdata(spi);
    int status;

    if (xra->irq)
        xra1405_irq_teardown(xra);

    status = gpiochip_remove(&xra->chip);
    if (status == 0)
        kfree(xra);
    
    return status;
}

static struct spi_driver xra1405_driver = {
    .probe    = xra1405_probe,
    .remove   = __devexit_p(xra1405_remove),
    .driver   = {
        .name           = XRA1405_MODULE_NAME,
        .owner          = THIS_MODULE,
    },
};

static void xra1405_spi_exit (void)
{
    spi_unregister_driver(&xra1405_driver);
}

static int __init xra1405_init (void)
{
    return spi_register_driver(&xra1405_driver);
}
module_init(xra1405_init);

static void __exit xra1405_exit (void)
{
    xra1405_spi_exit();
}
module_exit(xra1405_exit);

MODULE_ALIAS(MAKE_ALIAS("platform:", XRA1405_MODULE_NAME));
MODULE_DESCRIPTION("XRA1405 16-bit GPIO expander with an SPI interface");
MODULE_AUTHOR("Cal Poly Polysat");
MODULE_AUTHOR("Andres Villa <andresvilla@gmail.com>");
MODULE_LICENSE("GPL");

