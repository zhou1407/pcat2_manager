#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <poll.h>
#include <time.h>

typedef enum
{
    PCAT_PMU_MANAGER_COMMAND_HEARTBEAT = 0x1,
    PCAT_PMU_MANAGER_COMMAND_HEARTBEAT_ACK = 0x2,
    PCAT_PMU_MANAGER_COMMAND_PMU_HW_VERSION_GET = 0x3,
    PCAT_PMU_MANAGER_COMMAND_PMU_HW_VERSION_GET_ACK = 0x4,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_VERSION_GET = 0x5,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_VERSION_GET_ACK = 0x6,
    PCAT_PMU_MANAGER_COMMAND_STATUS_REPORT = 0x7,
    PCAT_PMU_MANAGER_COMMAND_STATUS_REPORT_ACK = 0x8,
    PCAT_PMU_MANAGER_COMMAND_DATE_TIME_SYNC = 0x9,
    PCAT_PMU_MANAGER_COMMAND_DATE_TIME_SYNC_ACK = 0xA,
    PCAT_PMU_MANAGER_COMMAND_SCHEDULE_STARTUP_TIME_SET = 0xB,
    PCAT_PMU_MANAGER_COMMAND_SCHEDULE_STARTUP_TIME_SET_ACK = 0xC,
    PCAT_PMU_MANAGER_COMMAND_PMU_REQUEST_SHUTDOWN = 0xD,
    PCAT_PMU_MANAGER_COMMAND_PMU_REQUEST_SHUTDOWN_ACK = 0xE,
    PCAT_PMU_MANAGER_COMMAND_HOST_REQUEST_SHUTDOWN = 0xF,
    PCAT_PMU_MANAGER_COMMAND_HOST_REQUEST_SHUTDOWN_ACK = 0x10,
    PCAT_PMU_MANAGER_COMMAND_PMU_REQUEST_FACTORY_RESET = 0x11,
    PCAT_PMU_MANAGER_COMMAND_PMU_REQUEST_FACTORY_RESET_ACK = 0x12,
    PCAT_PMU_MANAGER_COMMAND_WATCHDOG_TIMEOUT_SET = 0x13,
    PCAT_PMU_MANAGER_COMMAND_WATCHDOG_TIMEOUT_SET_ACK = 0x14,
    PCAT_PMU_MANAGER_COMMAND_CHARGER_ON_AUTO_START = 0x15,
    PCAT_PMU_MANAGER_COMMAND_CHARGER_ON_AUTO_START_ACK = 0x16,
    PCAT_PMU_MANAGER_COMMAND_VOLTAGE_THRESHOLD_SET = 0x17,
    PCAT_PMU_MANAGER_COMMAND_VOLTAGE_THRESHOLD_SET_ACK = 0x18,
    PCAT_PMU_MANAGER_COMMAND_NET_STATUS_LED_SETUP = 0x19,
    PCAT_PMU_MANAGER_COMMAND_NET_STATUS_LED_SETUP_ACK = 0x1A,
    PCAT_PMU_MANAGER_COMMAND_POWER_ON_EVENT_GET = 0x1B,
    PCAT_PMU_MANAGER_COMMAND_POWER_ON_EVENT_GET_ACK = 0x1C,
}PCatPMUManagerCommandType;

static inline uint16_t pcat_twd_compute_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t i;
    uint32_t j;

    for(i=0;i<len;i++)
    {
        crc ^= data[i];
        for(j=0;j<8;j++)
        {
            if(crc & 1)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static inline int64_t pcat_twd_get_monotonic_time()
{
    int64_t result;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    result = ((int64_t)ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;

    return result;
}

int main(int argc, char *argv[])
{
    int fd;
    struct termios options;
    int rspeed = B115200;
    int speed = 115200;
    const char *serial_device = "/dev/ttyS4";
    uint8_t cbuffer[4096];
    uint8_t rbuffer[256];
    size_t cbuffer_size = 0, cbuffer_used;
    ssize_t rsize;
    uint8_t buffer[13] = {0xA5, 0x01, 0x81, 0x0, 0x0, 0x3, 0x0,
        0x1, 0x0, 0x0, 0x0, 0x0, 0x5A};
    uint16_t frame_num = 0;
    uint16_t crc, rcrc;
    struct pollfd rfd;
    int ret;
    int64_t ftime, now;
    const uint8_t *p, *extra_data;
    size_t i;
    size_t remaining_size;
    uint16_t expect_len, extra_data_len;
    uint8_t src, dst;
    int need_ack;
    uint16_t rframe_num;
    uint8_t command;

    if(argc > 1)
    {
        serial_device = argv[1];
    }
    if(argc > 2)
    {
        if(sscanf(argv[2], "%d", &speed) < 1)
        {
            speed = 115200;
        }
    }

    /* daemon(1, 1); */

    fd = open(serial_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0)
    {
        fprintf(stderr, "Failed to open serial port %s: %s",
            serial_device, strerror(errno));

        return 1;
    }

    switch(speed)
    {
        case 4800:
        {
            rspeed = B4800;
            break;
        }
        case 9600:
        {
            rspeed = B9600;
            break;
        }
        case 19200:
        {
            rspeed = B19200;
            break;
        }
        case 38400:
        {
            rspeed = B38400;
            break;
        }
        case 57600:
        {
            rspeed = B57600;
            break;
        }
        case 115200:
        {
            rspeed = B115200;
            break;
        }
        default:
        {
            fprintf(stderr, "Invalid serial speed, "
                "set to default speed at %u.", 115200);
            break;
        }
    }

    tcgetattr(fd, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, rspeed);
    cfsetospeed(&options, rspeed);
    options.c_cflag &= ~CSIZE;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
        INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    rfd.fd = fd;
    rfd.events = POLLIN;

    ftime = pcat_twd_get_monotonic_time();

    while(1)
    {
        ret = poll(&rfd, 1, 100000);

        if(ret > 0)
        {
            while((rsize=read(fd, rbuffer, 256))>0)
            {
                if(cbuffer_size > 3840)
                {
                    cbuffer_size = 0;
                }

                memcpy(cbuffer + cbuffer_size, rbuffer, rsize);
                cbuffer_size += rsize;
                p = NULL;
                cbuffer_used = 0;
                remaining_size = cbuffer_size;

                for(i=0;i<cbuffer_size;i++)
                {
                    if(cbuffer[i]==0xA5)
                    {
                        p = cbuffer + i;
                        remaining_size = cbuffer_size - i;
                        cbuffer_used = i;

                        if(remaining_size < 13)
                        {
                            break;
                        }

                        expect_len = p[5] + ((uint16_t)p[6] << 8);
                        if(expect_len < 3 || expect_len > 65532)
                        {
                            cbuffer_used = i;
                            continue;
                        }

                        if(expect_len + 10 > remaining_size)
                        {
                            cbuffer_used = i;
                            continue;
                        }

                        if(p[9+expect_len]!=0x5A)
                        {
                            cbuffer_used = i;
                            continue;
                        }

                        crc = p[7+expect_len] +
                            ((uint16_t)p[8+expect_len] << 8);
                        rcrc = pcat_twd_compute_crc16(p+1, 6+expect_len);

                        if(crc!=rcrc)
                        {
                            i += 9 + expect_len;
                            cbuffer_used = i + 1;
                            continue;
                        }

                        src = p[1];
                        dst = p[2];
                        rframe_num = p[3] + ((uint16_t)p[4] << 8);
                        command = p[7] + ((uint16_t)p[8] << 8);
                        extra_data_len = expect_len - 3;
                        if(expect_len > 3)
                        {
                            extra_data = p + 9;
                        }
                        else
                        {
                            extra_data = NULL;
                        }
                        need_ack = (p[6 + expect_len]!=0);

                        switch(command)
                        {
                            case PCAT_PMU_MANAGER_COMMAND_PMU_REQUEST_SHUTDOWN:
                            {
                                system("reboot -p");
                                break;
                            }
                            default:
                            {
                                break;
                            }
                        }

                        i += 9 + expect_len;
                        cbuffer_used = i + 1;
                    }
                }

                if(p!=NULL)
                {
                    if(cbuffer_size > 0 && cbuffer_used > 0)
                    {
                        memmove(cbuffer, cbuffer + cbuffer_used,
                            cbuffer_size - cbuffer_used);
                        cbuffer_size -= cbuffer_used;
                    }
                }
                else
                {
                    cbuffer_size = 0;
                }
            }
        }

        now = pcat_twd_get_monotonic_time();
        if(now > ftime + 1000000)
        {
            ftime = now;

            buffer[3] = frame_num & 0xFF;
            buffer[4] = (frame_num >> 8) & 0xFF;
            frame_num++;

            crc = pcat_twd_compute_crc16(buffer + 1, 9);
            buffer[10] = crc & 0xFF;
            buffer[11] = (crc >> 8) & 0xFF;

            write(fd, buffer, 13);
        }
    }

    close(fd);

    return 0;
}
