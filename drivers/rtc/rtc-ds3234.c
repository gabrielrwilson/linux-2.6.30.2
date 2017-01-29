/* rtc-ds3234.c
 *
 * Driver for Dallas Semiconductor (DS3234) SPI RTC with Integrated Crystal
 * and SRAM.
 *
 * Copyright (C) 2008 MIMOMax Wireless Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spi/spi.h>
#include <linux/bcd.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <asm/atomic.h>
#include <linux/kobj_map.h>
#include <linux/rtc/ds3234.h>

/**
 * The max number of pules that can occur
 * without a read before the circular buffer
 * overflows.
 */
#define PPS_MAX_TIMEVALS 20

/**
 * The interrupt number for each pps device.
 */
#define PPS0_INTERRUPT 168
#define PPS_DEVICE_NAME "pps-rtc"
#define DS3234_DRIVER_NAME "rtc-DS3234"


#define DS3234_REG_SECONDS	0x00
#define DS3234_REG_MINUTES	0x01
#define DS3234_REG_HOURS	0x02
#define DS3234_REG_DAY		0x03
#define DS3234_REG_DATE		0x04
#define DS3234_REG_MONTH	0x05
#define DS3234_REG_YEAR		0x06
#define DS3234_REG_CENTURY	(1 << 7) /* Bit 7 of the Month register */
#define DS3234_REG_A1M1		0x07
#define DS3234_REG_A1M2		0x08
#define DS3234_REG_A1M3		0x09
#define DS3234_REG_A1M4		0x0A
#define DS3234_REG_A2M1		0x0B
#define DS3234_REG_A2M2		0x0C
#define DS3234_REG_A2M3		0x0D
#define DS3234_REG_CONTROL	0x0E
#define DS3234_REG_CONT_STAT	0x0F
#define DS3234_REG_TEMP_MSB	0x11
#define DS3234_REG_TEMP_LSB	0x12

#define DS3234_CTRL_A1IE	0x01
#define DS3234_CTRL_A2IE	0x02
#define DS3234_CTRL_INTCN	0x04
#define DS3234_CTRL_CONV	0x20
#define DS3234_STAT_BUSY	0x04

struct PPSDevice {
   struct cdev cdev;
   int nreaders; /* The number of processes using the device */
   wait_queue_head_t read_q;
   struct DS3234IrqData data_buff[PPS_MAX_TIMEVALS]; /* Circular buffer of data for pps occurences */
   volatile int tv_head;
   struct mutex mutex;
   int devno;
   struct rtc_device *rtc;
   unsigned irq;
   struct timeval *shared_irq_time;
   struct spi_message   spi_msg;
   struct spi_transfer  spi_xfer[2];
   u8                   tx_buff_reg[4];
   u8                   tx_buff[32];
   u8                   rx_buff[32];
   struct spi_device *spi;
   struct RTCTempReading temp;
   int update_rtc_time;
   struct rtc_time update_time;
   unsigned long update_time_t;
   struct class *class;
   struct device *device;
};

static struct PPSDevice *gpps_device = NULL;

/**
 * Struct to track each user's position in the 
 * circular buffer. If the circular buffer 
 * overflows, the user is not notified, the data
 * is just overwritten. 
 */ 
struct PPSUser {
   struct PPSDevice *dev;
   int tv_tail;
   int wants_interrupts;
};

static ssize_t pps_read(struct file *, char __user *, size_t, loff_t *);
static int pps_open(struct inode *, struct file *);
static int pps_release(struct inode *, struct file *);
static unsigned int pps_poll(struct file *filp, poll_table *wait);
static irqreturn_t rtc_interrupt_handler(int, void *);
static int pps_init(struct PPSDevice *);
static void pps_exit(struct PPSDevice *);
static int pps_ioctl(struct inode *inode,
    struct file *file, unsigned int ioctl_num,
    unsigned long ioctl_param);

struct file_operations pps_fops = {
   .owner = THIS_MODULE,
   .read = &pps_read,
   .write = NULL,
   .poll = &pps_poll,
   .llseek = no_llseek,
   .open = &pps_open,
   .release = &pps_release,
   .ioctl = &pps_ioctl,
};

static int ds3234_set_reg(struct device *dev, unsigned char address,
				unsigned char data)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[2];

	/* MSB must be '1' to indicate write */
	buf[0] = address | 0x80;
	buf[1] = data;

	return spi_write_then_read(spi, buf, 2, NULL, 0);
}

static int ds3234_get_reg(struct device *dev, unsigned char address,
				unsigned char *data)
{
	struct spi_device *spi = to_spi_device(dev);

	*data = address & 0x7f;

	return spi_write_then_read(spi, data, 1, data, 1);
}

static int ds3234_read_time(struct device *dev, struct rtc_time *dt)
{
	int err;
	unsigned char buf[8];
	struct spi_device *spi = to_spi_device(dev);

	buf[0] = 0x00; /* Start address */

	err = spi_write_then_read(spi, buf, 1, buf, 8);
	if (err != 0)
		return err;

	/* Seconds, Minutes, Hours, Day, Date, Month, Year */
	dt->tm_sec	= bcd2bin(buf[0]);
	dt->tm_min	= bcd2bin(buf[1]);
	dt->tm_hour	= bcd2bin(buf[2] & 0x3f);
	dt->tm_wday	= bcd2bin(buf[3]) - 1; /* 0 = Sun */
	dt->tm_mday	= bcd2bin(buf[4]);
	dt->tm_mon	= bcd2bin(buf[5] & 0x1f) - 1; /* 0 = Jan */
	dt->tm_year 	= bcd2bin(buf[6] & 0xff) + 100; /* Assume 20YY */

	return rtc_valid_tm(dt);
}

static int ds3234_set_time(struct device *dev, struct rtc_time *dt)
{
	ds3234_set_reg(dev, DS3234_REG_SECONDS, bin2bcd(dt->tm_sec));
	ds3234_set_reg(dev, DS3234_REG_MINUTES, bin2bcd(dt->tm_min));
	ds3234_set_reg(dev, DS3234_REG_HOURS, bin2bcd(dt->tm_hour) & 0x3f);

	/* 0 = Sun */
	ds3234_set_reg(dev, DS3234_REG_DAY, bin2bcd(dt->tm_wday + 1));
	ds3234_set_reg(dev, DS3234_REG_DATE, bin2bcd(dt->tm_mday));

	/* 0 = Jan */
	ds3234_set_reg(dev, DS3234_REG_MONTH, bin2bcd(dt->tm_mon + 1));

	/* Assume 20YY although we just want to make sure not to go negative. */
	if (dt->tm_year > 100)
		dt->tm_year -= 100;

	ds3234_set_reg(dev, DS3234_REG_YEAR, bin2bcd(dt->tm_year));

	return 0;
}

static const struct rtc_class_ops ds3234_rtc_ops = {
	.read_time	= ds3234_read_time,
	.set_time	= ds3234_set_time,
};

static int __devinit ds3234_probe(struct spi_device *spi)
{
	unsigned char tmp;
	int res;
	struct PPSDevice *pps_device;
	struct DS3234PlatData *pdata;

	pdata = (struct DS3234PlatData*)spi->dev.platform_data;
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	spi_setup(spi);

	res = ds3234_get_reg(&spi->dev, DS3234_REG_SECONDS, &tmp);
	if (res != 0)
		return res;

        /* Configure alarm 1 to interrupt every second */
	// ds3234_set_reg(&spi->dev, DS3234_REG_A1M1, 0);
	// ds3234_set_reg(&spi->dev, DS3234_REG_A1M2, 0);
	// ds3234_set_reg(&spi->dev, DS3234_REG_A1M3, 0);
	// ds3234_set_reg(&spi->dev, DS3234_REG_A1M4, 0);

	/* Control settings
	 *
	 * CONTROL_REG
	 * BIT 7	6	5	4	3	2	1	0
	 *     EOSC	BBSQW	CONV	RS2	RS1	INTCN	A2IE	A1IE
	 *
	 *     0	0	0	1	1	1	0	0
	 *
	 * CONTROL_STAT_REG
	 * BIT 7	6	5	4	3	2	1	0
	 *     OSF	BB32kHz	CRATE1	CRATE0	EN32kHz	BSY	A2F	A1F
	 *
	 *     1	0	0	0	1	0	0	0
	 */
	ds3234_get_reg(&spi->dev, DS3234_REG_CONTROL, &tmp);
	ds3234_set_reg(&spi->dev, DS3234_REG_CONTROL, tmp & 0x1c);
	// ds3234_set_reg(&spi->dev, DS3234_REG_CONTROL, (tmp & 0x1c) | DS3234_CTRL_A1IE | DS3234_CTRL_INTCN);

	ds3234_get_reg(&spi->dev, DS3234_REG_CONT_STAT, &tmp);
	ds3234_set_reg(&spi->dev, DS3234_REG_CONT_STAT, tmp & 0x88);

	/* Print our settings */
	ds3234_get_reg(&spi->dev, DS3234_REG_CONTROL, &tmp);
	dev_info(&spi->dev, "Control Reg: 0x%02x\n", tmp);

	ds3234_get_reg(&spi->dev, DS3234_REG_CONT_STAT, &tmp);
	dev_info(&spi->dev, "Ctrl/Stat Reg: 0x%02x\n", tmp);

	pps_device = kmalloc(sizeof(*pps_device), GFP_KERNEL);
	if (!pps_device)
		return -ENOMEM;
	memset(pps_device, 0, sizeof(*pps_device));
        gpps_device = pps_device;

	pps_device->spi = spi;
	pps_device->irq = PPS0_INTERRUPT;
	if (pdata) {
		if (pdata->irq)
			pps_device->irq = pdata->irq;
		pps_device->shared_irq_time = pdata->shared_irq_time;
	}
	pps_device->rtc = rtc_device_register("ds3234",
				&spi->dev, &ds3234_rtc_ops, THIS_MODULE);
	if (IS_ERR(pps_device->rtc))
		return PTR_ERR(pps_device->rtc);

	dev_set_drvdata(&spi->dev, pps_device);

	if (pps_device->irq > 0) {

		res = pps_init(pps_device);
        	if (res < 0)
        		return res;
	}

	return 0;
}

static int __devexit ds3234_remove(struct spi_device *spi)
{
	struct PPSDevice *pps_device =
                (struct PPSDevice*)dev_get_drvdata(&spi->dev);

	if (pps_device) {
		ds3234_set_reg(&pps_device->spi->dev, DS3234_REG_CONTROL, DS3234_CTRL_INTCN);
		rtc_device_unregister(pps_device->rtc);
		if (pps_device->irq > 0)
			pps_exit(pps_device);
		kfree(pps_device);
	}

	return 0;
}

static struct spi_driver ds3234_driver = {
	.driver = {
		.name	 = "ds3234",
		.owner	= THIS_MODULE,
	},
	.probe	 = ds3234_probe,
	.remove = __devexit_p(ds3234_remove),
};

static __init int ds3234_init(void)
{
	return spi_register_driver(&ds3234_driver);
}
module_init(ds3234_init);

static __exit void ds3234_exit(void)
{
	spi_unregister_driver(&ds3234_driver);
}
module_exit(ds3234_exit);

static inline void pps_incr_idx(int buff_size, volatile int *index, int delta)
{
   if (*index + delta >= buff_size)
      *index = 0; /* wrap */
   else
      *index += delta;
}

static void rtc_irq_write_complete(void *context)
{
   struct PPSDevice *dev = (struct PPSDevice*) context;
   dev->spi_msg.complete = NULL;
   //printk("Alarm program complete\n");
}

static void rtc_irq_read_complete(void *context)
{
   struct PPSDevice *dev = (struct PPSDevice*) context;
   unsigned int sec, minute, hour, day, mon, year;
   unsigned long unixTime;
   struct rtc_time tm;
   unsigned long nextTrigger = 0;
   int reg_base = DS3234_REG_A1M1;
   int cmd_len = 9;

   // printk("Read callback 0x%02X, 0x%02X\n", dev->rx_buff[0x0E],
         // dev->rx_buff[DS3234_REG_CONT_STAT]);
   // printk("  A1: %02X, %02X, %02X, %02X\n", dev->rx_buff[7], dev->rx_buff[9], dev->rx_buff[9], dev->rx_buff[10]);
#if 0
	/* Seconds, Minutes, Hours, Day, Date, Month, Year */
	dt->tm_sec	= bcd2bin(buf[0]);
	dt->tm_min	= bcd2bin(buf[1]);
	dt->tm_hour	= bcd2bin(buf[2] & 0x3f);
	dt->tm_wday	= bcd2bin(buf[3]) - 1; /* 0 = Sun */
	dt->tm_mday	= bcd2bin(buf[4]);
	dt->tm_mon	= bcd2bin(buf[5] & 0x1f) - 1; /* 0 = Jan */
	dt->tm_year 	= bcd2bin(buf[6] & 0xff) + 100; /* Assume 20YY */

	return rtc_valid_tm(dt);
#endif
   /* Convert RTC time into seconds since epoch */
   sec	= bcd2bin(dev->rx_buff[DS3234_REG_SECONDS]);
   minute	= bcd2bin(dev->rx_buff[DS3234_REG_MINUTES]);
   hour	= bcd2bin(dev->rx_buff[DS3234_REG_HOURS] & 0x3f);
   day	= bcd2bin(dev->rx_buff[DS3234_REG_DATE]);
   mon	= bcd2bin(dev->rx_buff[DS3234_REG_MONTH] & 0x1f); /* 0 = Jan */
   year	= bcd2bin(dev->rx_buff[DS3234_REG_YEAR] & 0xff) + 2000; /* Assume 20YY */
   unixTime = mktime(year, mon, day, hour, minute, sec);

   // Pass the RTC EPOCH time and notify waiting processes
   dev->data_buff[dev->tv_head].rtc_epoch_time = unixTime;
   dev->data_buff[dev->tv_head].new_epoch_time = unixTime;
   dev->data_buff[dev->tv_head].rtc_updated = 0;

   // save the temperature reading
   dev->temp.reading = (dev->rx_buff[DS3234_REG_TEMP_MSB] << 8) |
                        dev->rx_buff[DS3234_REG_TEMP_LSB];

   // ds3234_set_reg(&spi->dev, DS3234_REG_CONT_STAT, 0x00);
   rtc_time_to_tm(nextTrigger, &tm);
   if (dev->update_rtc_time) {
      // printk("Updating RTC clock to %lu\n", dev->update_time_t);
      dev->data_buff[dev->tv_head].new_epoch_time = dev->update_time_t;
      dev->data_buff[dev->tv_head].rtc_updated = 1;

      dev->tx_buff[DS3234_REG_SECONDS] = bin2bcd(dev->update_time.tm_sec);
      dev->tx_buff[DS3234_REG_MINUTES] = bin2bcd(dev->update_time.tm_min);
      dev->tx_buff[DS3234_REG_HOURS] = bin2bcd(dev->update_time.tm_hour) & 0x3f;

      /* 0 = Sun */
      dev->tx_buff[DS3234_REG_DAY] = bin2bcd(dev->update_time.tm_wday + 1);
      dev->tx_buff[DS3234_REG_DATE] = bin2bcd(dev->update_time.tm_mday);

      /* 0 = Jan */
      dev->tx_buff[DS3234_REG_MONTH] = bin2bcd(dev->update_time.tm_mon + 1);

      /* Assume 20YY although we just want to make sure not to go negative. */
      if (dev->update_time.tm_year > 100)
         dev->update_time.tm_year -= 100;

      dev->tx_buff[DS3234_REG_YEAR] = bin2bcd(dev->update_time.tm_year);

      dev->update_rtc_time = 0;
      cmd_len += reg_base - DS3234_REG_SECONDS;
      reg_base = DS3234_REG_SECONDS;
   }
   pps_incr_idx(PPS_MAX_TIMEVALS, &dev->tv_head, 1);
   wake_up_interruptible(&dev->read_q);

   dev->tx_buff[DS3234_REG_A1M1] = 0x80;
   dev->tx_buff[DS3234_REG_A1M2] = 0x80;
   // dev->tx_buff[0x09] = bin2bcd(tm.tm_hour) & 0x3f;
   dev->tx_buff[DS3234_REG_A1M3] = 0x80;
   // dev->tx_buff[0x0A] = bin2bcd(tm.tm_mday);
   dev->tx_buff[DS3234_REG_A1M4] = 0x80;
   dev->tx_buff[DS3234_REG_A2M1] = 0x80;
   dev->tx_buff[DS3234_REG_A2M2] = 0x80;
   dev->tx_buff[DS3234_REG_A2M3] = 0x80;
   dev->tx_buff[DS3234_REG_CONTROL] = (dev->rx_buff[DS3234_REG_CONTROL] & 0xF8) | DS3234_CTRL_A1IE | DS3234_CTRL_INTCN | DS3234_CTRL_A2IE;
   dev->tx_buff[DS3234_REG_CONT_STAT] = dev->rx_buff[DS3234_REG_CONTROL] & 0xFC;

   spi_message_init(&dev->spi_msg);
   dev->spi_msg.context = dev;
   memset(&dev->spi_xfer, 0, sizeof(dev->spi_xfer));

   // Write 1 address byte plus all data
   dev->tx_buff[reg_base-1] = reg_base | 0x80;
   dev->spi_xfer[0].tx_buf = &dev->tx_buff[reg_base-1];
   dev->spi_xfer[0].len = cmd_len + 1;
   spi_message_add_tail(&dev->spi_xfer[0], &dev->spi_msg);

   // Set completion handler and run
   dev->spi_msg.complete = rtc_irq_write_complete;
   spi_async(dev->spi, &dev->spi_msg);
}

static irqreturn_t rtc_interrupt_handler(int irq, void *dev_id)
{
   struct PPSDevice *dev = (struct PPSDevice*) dev_id;
   
   /* Cast to stop volatile warning */
   if (dev->shared_irq_time) {
      *(struct timeval *)&dev->data_buff[dev->tv_head].tv =
                                  *dev->shared_irq_time;
   }
   else
      do_gettimeofday((struct timeval *)&dev->data_buff[dev->tv_head].tv);
   dev->temp.time = dev->data_buff[dev->tv_head].tv;

   // Pass the RTC EPOCH time and notify waiting processes
   spi_message_init(&dev->spi_msg);
   dev->spi_msg.context = dev;
   memset(&dev->spi_xfer, 0, sizeof(dev->spi_xfer));

   // Write 1 address byte
   dev->tx_buff[0] = 0;
   dev->spi_xfer[0].tx_buf = dev->tx_buff;
   dev->spi_xfer[0].len = 1;
   spi_message_add_tail(&dev->spi_xfer[0], &dev->spi_msg);

   // Read 19 data bytes
   dev->spi_xfer[1].rx_buf = dev->rx_buff;
   dev->spi_xfer[1].len = 20;
   spi_message_add_tail(&dev->spi_xfer[1], &dev->spi_msg);

   // Set completion handler and run
   dev->spi_msg.complete = rtc_irq_read_complete;
   spi_async(dev->spi, &dev->spi_msg);
   
   return IRQ_HANDLED;
}

static ssize_t pps_read(struct file *filp, char __user *buff, size_t len, loff_t *f_pos) 
{
   struct PPSUser *user = filp->private_data;
   struct PPSDevice *dev = user->dev;
   int len0, data_size = sizeof(dev->data_buff[0]), n_timevals;

   if (len < data_size) {
      return -EFAULT;
   }

   if (mutex_lock_interruptible(&dev->mutex))
      return -ERESTARTSYS;
  
   while (dev->tv_head == user->tv_tail) {
      mutex_unlock(&dev->mutex);
      if (filp->f_flags & O_NONBLOCK)
         return -EAGAIN;
      if (wait_event_interruptible(dev->read_q, (dev->tv_head != user->tv_tail)))
         return -ERESTARTSYS; /* Recieved a signal */
      if (mutex_lock_interruptible(&dev->mutex))
         return -ERESTARTSYS;
      if (!user->wants_interrupts)
         user->tv_tail = dev->tv_head;
   }

   /* Number of bytes available to read stored in len0 */
   len0 = data_size * (dev->tv_head - user->tv_tail);
   if (len0 < 0) /* wrapped */
      len0 = data_size * (PPS_MAX_TIMEVALS - user->tv_tail);
   if (len0 < len) len = len0;
   
   len -= (len % data_size);
   n_timevals = len/data_size;
   
   if (copy_to_user(buff, (char *)&dev->data_buff[user->tv_tail], len)) {
      mutex_unlock(&dev->mutex);
      return -EFAULT;
   }

   pps_incr_idx(PPS_MAX_TIMEVALS, &user->tv_tail, n_timevals);

   mutex_unlock(&dev->mutex);

   return len;
}

static int start_interrupts(struct PPSDevice *dev)
{
   int res;

   if (dev->nreaders == 0) {
      // dev->tv_head = 0;
      //if (dev->irq >= 160 && dev->irq < 165)
      //			request_module("xra1405");
      res = request_irq(dev->irq, &rtc_interrupt_handler, IRQF_TRIGGER_FALLING, "rtc-ds3234", dev);
      if (res < 0) {
         printk(KERN_WARNING "Unable to enable pps interrupt\n");
         return -EBUSY;
      }

      // Set up interrupt for next second boundary
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A1M1, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A1M2, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A1M3, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A1M4, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A2M1, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A2M2, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_A2M3, 0x80);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_CONT_STAT, 0x00);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_CONTROL, DS3234_CTRL_A1IE | DS3234_CTRL_A2IE | DS3234_CTRL_INTCN);
   }

   dev->nreaders++;

   return 0;
}

static void stop_interrupts(struct PPSDevice *dev)
{
   dev->nreaders--;

   if (dev->nreaders == 0) {
      if (dev->irq > 0)
         free_irq(dev->irq, dev);
      ds3234_set_reg(&dev->spi->dev, DS3234_REG_CONTROL,
                         DS3234_CTRL_INTCN);
   }
}

static int pps_open(struct inode *inode, struct file *filp)
{
   struct PPSDevice *dev;
   struct PPSUser *user;

   dev = container_of(inode->i_cdev, struct PPSDevice, cdev);

   if (!inode->i_cdev) {
      printk("Warning: Using global pps struct!\n");
      dev = gpps_device;
   }

   user = kmalloc(sizeof(struct PPSUser), GFP_KERNEL);
   if (!user)
      return -ENOMEM;
   memset(user, 0, sizeof(struct PPSUser));

   user->dev = dev;
   user->tv_tail = dev->tv_head;
   filp->private_data = user;

   if (mutex_lock_interruptible(&dev->mutex)) {
      kfree(user);
      return -ERESTARTSYS;
   }

   user->wants_interrupts = 1;
   start_interrupts(dev);
   mutex_unlock(&dev->mutex);

   return nonseekable_open(inode, filp);   
}

static int pps_release(struct inode *inode, struct file *filp)
{
   struct PPSUser *user = filp->private_data;
   struct PPSDevice *dev = user->dev;
   
   if (mutex_lock_interruptible(&dev->mutex))
      return -ERESTARTSYS;

   if (user->wants_interrupts)
      stop_interrupts(dev);
   user->wants_interrupts = 0;

   mutex_unlock(&dev->mutex);
   kfree(user);

   return 0;
}

static int pps_ioctl(struct inode *inode,
    struct file *filp, unsigned int ioctl_num,
    unsigned long ioctl_param)
{
   struct PPSUser *user = filp->private_data;
   struct PPSDevice *dev = user->dev;
   u8 tmp;

   switch(ioctl_num) {
      case IOCTL_RTC_START_PPS:
         if (!user->wants_interrupts) {
            user->tv_tail = dev->tv_head;
            user->wants_interrupts = 1;
            start_interrupts(dev);
         }
         break;

      case IOCTL_RTC_STOP_PPS:
         if (user->wants_interrupts) {
            user->tv_tail = dev->tv_head;
            user->wants_interrupts = 0;
            stop_interrupts(dev);
         }
         break;

      case IOCTL_RTC_GET_TEMP:
         return copy_to_user((void*)ioctl_param, &dev->temp, sizeof(dev->temp));

      case IOCTL_RTC_TEMP_COMP:
	 ds3234_get_reg(&dev->spi->dev, DS3234_REG_CONT_STAT, &tmp);
         if (!(tmp & DS3234_STAT_BUSY)) {
	    ds3234_get_reg(&dev->spi->dev, DS3234_REG_CONTROL, &tmp);
	    ds3234_set_reg(&dev->spi->dev, DS3234_REG_CONTROL, tmp | DS3234_CTRL_CONV);
         }
         break;

      case IOCTL_RTC_SET_NEXT_TIME:
         if (ioctl_param < 946684800) // 1/1/2000
            return -EINVAL;

         dev->update_time_t = ioctl_param;
         rtc_time_to_tm(dev->update_time_t, &dev->update_time);
	 // printk("New time: %02d:%02d:%02d ", dev->update_time.tm_hour, dev->update_time.tm_min, dev->update_time.tm_sec);

	 // printk("%02d/%02d/%04d\n", dev->update_time.tm_mon + 1, dev->update_time.tm_min, dev->update_time.tm_year);

	 /* 0 = Sun */
         // printk("   wday: %d\n", dev->update_time.tm_wday + 1);
         dev->update_rtc_time = 1;
         break;

      default:
         return -EINVAL;
   }

   return 0;
}

static unsigned int pps_poll(struct file *filp, poll_table *wait)
{
   struct PPSUser *user = filp->private_data;
   struct PPSDevice *dev = user->dev;
   int head = dev->tv_head;
   unsigned int mask = 0;

   poll_wait(filp, &dev->read_q, wait);

   if (user->wants_interrupts && head != user->tv_tail)
      mask |= POLLIN | POLLRDNORM;

   return mask;
}

static void pps_exit(struct PPSDevice *pps_device)
{
   // printk("PPS Exit!\n");
   if (pps_device->device) {
      device_destroy(pps_device->class, pps_device->devno);
      pps_device->device = NULL;
   }

   if (pps_device->class) {
      class_destroy(pps_device->class);
      pps_device->class = NULL;
   }
   if (pps_device->devno) {
      cdev_del(&pps_device->cdev);
      unregister_chrdev_region(pps_device->devno, 1);
   }
}

static int pps_init(struct PPSDevice *pps_device)
{
   dev_t dev;
   int result = 0;
   int res;
   // void *p = NULL;
  
   dev = MKDEV(0,0);
   res = alloc_chrdev_region(&dev, 0, 1, PPS_DEVICE_NAME);
   if (res < 0) {
      printk(KERN_WARNING "pps: can't get device number %d\n", res);
      return result;
   }
   pps_device->devno = dev;

   pps_device->nreaders = 0;
   init_waitqueue_head(&pps_device->read_q);
   mutex_init(&pps_device->mutex);

   pps_device->class = class_create(THIS_MODULE, "rtc-pps");
   if (!pps_device->class) {
      printk(KERN_ALERT "DS3234 class_create() failed\n");
      goto fail;
   }

   pps_device->device = device_create(pps_device->class, NULL,
                                 pps_device->devno, NULL, "ds3234");
   if (!pps_device->device) {
      printk(KERN_ALERT "DS3234 device_create() failed\n");
      goto fail;
   }
   dev_set_drvdata(pps_device->device, pps_device);

   cdev_init(&pps_device->cdev, &pps_fops);
   pps_device->cdev.owner = THIS_MODULE;
   pps_device->cdev.dev = dev;
   res = cdev_add(&pps_device->cdev, dev, 1);
   if (res < 0) {
      printk("Error %d adding pps device\n", res);
      goto fail;
   }

#if 0
   printk("cdev init: 0x%p\n", &pps_device->cdev);
   p = container_of(&pps_device->cdev, struct cdev, kobj);
   printk("   kobj: 0x%p\n", p);

   printk("cdev reverence: 0x%p\n", p);
#endif

   return 0;

fail:
   pps_exit(pps_device);
   return result;
}

MODULE_DESCRIPTION("DS3234 SPI RTC driver");
MODULE_AUTHOR("Dennis Aberilla <denzzzhome@yahoo.com>");
MODULE_LICENSE("GPL");
