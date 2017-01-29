#ifndef RTC_DS3234_H
#define RTC_DS3234_H

#define RTC_MAJOR_NUM 251

struct RTCTempReading {
   uint16_t reading;
   struct timeval time;
};

#define IOCTL_RTC_START_PPS _IO(RTC_MAJOR_NUM, 0)
#define IOCTL_RTC_STOP_PPS _IO(RTC_MAJOR_NUM, 1)
#define IOCTL_RTC_GET_TEMP _IOW(RTC_MAJOR_NUM, 2, struct RTCTempReading*)
#define IOCTL_RTC_TEMP_COMP _IO(RTC_MAJOR_NUM, 3)
#define IOCTL_RTC_SET_NEXT_TIME _IOR(RTC_MAJOR_NUM, 4, unsigned long)

struct DS3234PlatData {
   unsigned irq;
   struct timeval *shared_irq_time;
};

struct DS3234IrqData {
   struct timeval tv;
   unsigned long rtc_epoch_time;
   unsigned long new_epoch_time;
   unsigned char rtc_updated;
};

#endif
