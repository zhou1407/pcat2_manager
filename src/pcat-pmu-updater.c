#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <glib.h>
#include <zlib.h>

#define PCAT_PMU_MANAGER_PM_DEV "/dev/pcat-pm-ctl"
#define PCAT_PMU_MANAGER_COMMAND_TIMEOUT 1000000L
#define PCAT_PMU_MANAGER_COMMAND_QUEUE_MAX 128

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
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_STATUS = 0xC9,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_REQUEST = 0xCB,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_INFO_SEND = 0xCD,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_INFO_SEND_ACK = 0xCE,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_FINISH = 0xCF,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_FINISH_ACK = 0xD0,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_SEND = 0xD3,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_SEND_ACK = 0xD4,
    PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_READY = 0xD5,
}PCatPMUManagerCommandType;

typedef enum
{
    PCAT_PMU_UPDATER_STEP_START = 0,
    PCAT_PMU_UPDATER_STEP_WAIT,
    PCAT_PMU_UPDATER_STEP_ERROR,
    PCAT_PMU_UPDATER_STEP_PMU_FW_VERSION_QUERY,
    PCAT_PMU_UPDATER_STEP_PMU_FW_INFO_SEND,
    PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_READY,
    PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_WORKING,
    PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_FINISH,
    PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_END,
    PCAT_PMU_UPDATER_STEP_LAST
}PcatPMUUpdaterStepType;

typedef struct _PCatPMUManagerCommandData
{
    GByteArray *buffer;
    gsize written_size;
    guint16 command;
    gboolean need_ack;
    guint retry_count;
    guint16 frame_num;
    gint64 timestamp;
    gboolean firstrun;
}PCatPMUManagerCommandData;

typedef struct _PCatPMUUpdaterData
{
    GMainLoop *main_loop;

    guint check_timeout_id;

    int dev_fd;
    GIOChannel *dev_channel;
    guint dev_read_source;
    guint dev_write_source;
    GByteArray *dev_read_buffer;
    gchar *board_hwmon_device_path;

    PCatPMUManagerCommandData *dev_write_current_command_data;
    GQueue *dev_write_command_queue;

    gchar *pmu_fw_version;

    PcatPMUUpdaterStepType step;

    gchar fw_version[15];
    GByteArray *fw_data;
    gsize fw_data_sent;
    gint fw_update_flag;

    gchar *dtb_compatible;
    gsize dtb_compatible_len;
}PCatPMUUpdaterData;

static PCatPMUUpdaterData g_pcat_pmu_updater_data = {0};

static const gchar *g_pcat_pmu_dtb_compatible_name[] = {
    "ariaboard,photonicat2",
    NULL
};

static gboolean pcat_pmu_pm_dev_open(PCatPMUUpdaterData *pmu_data);
static void pcat_pmu_pm_dev_close(PCatPMUUpdaterData *pmu_data);
static void pcat_pmu_updater_pmu_fw_data_send_internal(
    PCatPMUUpdaterData *pu_data);

static gboolean g_pcat_pmu_updater_cmd_force = FALSE;
static gboolean g_pcat_pmu_updater_cmd_get_pwu_fw_version = FALSE;

static GOptionEntry g_pcat_pmu_updater_cmd_entries[] =
{
    { "force", 'F', 0, G_OPTION_ARG_NONE, &g_pcat_pmu_updater_cmd_force,
        "Force update PMU firmware (ignore version checking).", NULL },
    { "pmu-fw-version-get", 'P', 0, G_OPTION_ARG_NONE,
        &g_pcat_pmu_updater_cmd_get_pwu_fw_version,
        "Get current PMU firmware version", NULL },
    { NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, NULL }
};

static void pcat_pmu_manager_command_data_free(
    PCatPMUManagerCommandData *data)
{
    if(data==NULL)
    {
        return;
    }

    if(data->buffer!=NULL)
    {
        g_byte_array_unref(data->buffer);
    }

    g_free(data);
}

static inline guint16 pcat_pmu_compute_crc16(const guint8 *data,
    gsize len)
{
    guint16 crc = 0xFFFF;
    gsize i;
    guint j;

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

static gboolean pcat_pmu_pm_dev_write_watch_func(GIOChannel *source,
    GIOCondition condition, gpointer user_data)
{
    PCatPMUUpdaterData *pmu_data = (PCatPMUUpdaterData *)user_data;
    gssize wsize = 0;
    guint remaining_size;
    gboolean ret = FALSE;
    gint64 now;
    GByteArray *buffer;

    now = g_get_monotonic_time();

    do
    {
        if(pmu_data->dev_write_current_command_data==NULL)
        {
            pmu_data->dev_write_current_command_data =
                g_queue_pop_head(pmu_data->dev_write_command_queue);
        }
        if(pmu_data->dev_write_current_command_data==NULL)
        {
            break;
        }
        if(!pmu_data->dev_write_current_command_data->firstrun &&
            pmu_data->dev_write_current_command_data->written_size==0)
        {
            if(now <= pmu_data->dev_write_current_command_data->timestamp +
                PCAT_PMU_MANAGER_COMMAND_TIMEOUT)
            {
                break;
            }
            else if(
                pmu_data->dev_write_current_command_data->retry_count==0)
            {
                pcat_pmu_manager_command_data_free(
                    pmu_data->dev_write_current_command_data);
                pmu_data->dev_write_current_command_data = NULL;

                continue;
            }
        }

        buffer = pmu_data->dev_write_current_command_data->buffer;

        if(buffer->len <=
           pmu_data->dev_write_current_command_data->written_size)
        {
            if(pmu_data->dev_write_current_command_data->need_ack)
            {
                break;
            }
            else
            {
                pcat_pmu_manager_command_data_free(
                    pmu_data->dev_write_current_command_data);
                pmu_data->dev_write_current_command_data = NULL;

                continue;
            }
        }

        remaining_size = buffer->len -
            pmu_data->dev_write_current_command_data->written_size;

        wsize = write(pmu_data->dev_fd,
            buffer->data +
            pmu_data->dev_write_current_command_data->written_size,
            remaining_size > 4096 ? 4096 : remaining_size);

        if(wsize > 0)
        {
            pmu_data->dev_write_current_command_data->written_size += wsize;
            pmu_data->dev_write_current_command_data->timestamp = now;
            pmu_data->dev_write_current_command_data->firstrun = FALSE;
        }
        else
        {
            break;
        }
    }
    while(1);

    if(pmu_data->dev_write_current_command_data!=NULL &&
       pmu_data->dev_write_current_command_data->written_size >=
       pmu_data->dev_write_current_command_data->buffer->len)
    {
        if(pmu_data->dev_write_current_command_data->need_ack &&
           pmu_data->dev_write_current_command_data->retry_count > 0)
        {
            pmu_data->dev_write_current_command_data->retry_count--;
            pmu_data->dev_write_current_command_data->written_size = 0;
        }
        else
        {
            pcat_pmu_manager_command_data_free(
                pmu_data->dev_write_current_command_data);
            pmu_data->dev_write_current_command_data = NULL;
        }
    }

    if(wsize < 0)
    {
        if(errno==EAGAIN)
        {
            ret = TRUE;
        }
        else
        {
            g_warning("PM device write error: %s", strerror(errno));

            pcat_pmu_pm_dev_close(pmu_data);
        }
    }

    if(!ret)
    {
        pmu_data->dev_write_source = 0;
    }

    return ret;
}

static void pcat_pmu_pm_dev_write_data_request(
    PCatPMUUpdaterData *pmu_data, guint16 command,
    const guint8 *extra_data, guint16 extra_data_len, gboolean need_ack)
{
    GByteArray *ba;
    guint16 sv;
    guint16 dp_size;
    PCatPMUManagerCommandData *new_data, *old_data;

    ba = g_byte_array_new();

    g_byte_array_append(ba, (const guint8 *)"\xA5\x01\x81", 3);
    g_byte_array_append(ba, (const guint8 *)&sv, 2);

    if(extra_data!=NULL && extra_data_len > 0 && extra_data_len <= 65532)
    {
        sv = extra_data_len + 3;
        sv = GUINT16_TO_LE(sv);
        g_byte_array_append(ba, (const guint8 *)&sv, 2);

        sv = command;
        sv = GUINT16_TO_LE(sv);
        g_byte_array_append(ba, (const guint8 *)&sv, 2);

        g_byte_array_append(ba, extra_data, extra_data_len);

        dp_size = extra_data_len + 3;
    }
    else
    {
        sv = 3;
        sv = GUINT16_TO_LE(sv);

        g_byte_array_append(ba, (const guint8 *)&sv, 2);

        sv = command;
        sv = GUINT16_TO_LE(sv);
        g_byte_array_append(ba, (const guint8 *)&sv, 2);

        dp_size = 3;
    }

    g_byte_array_append(ba,
        need_ack ? (const guint8 *)"\x01" : (const guint8 *)"\x00", 1);

    sv = pcat_pmu_compute_crc16(ba->data + 1, dp_size + 6);
    sv = GUINT16_TO_LE(sv);
    g_byte_array_append(ba, (const guint8 *)&sv, 2);

    g_byte_array_append(ba, (const guint8 *)"\x5A", 1);

    while(g_queue_get_length(pmu_data->dev_write_command_queue) >
       PCAT_PMU_MANAGER_COMMAND_QUEUE_MAX)
    {
        old_data = g_queue_pop_head(pmu_data->dev_write_command_queue);
        pcat_pmu_manager_command_data_free(old_data);
    }

    new_data = g_new0(PCatPMUManagerCommandData, 1);
    new_data->buffer = ba;
    new_data->timestamp = g_get_monotonic_time();
    new_data->need_ack = need_ack;
    new_data->retry_count = need_ack ? 3 : 1;
    new_data->command = command;
    new_data->firstrun = TRUE;

    g_queue_push_tail(pmu_data->dev_write_command_queue, new_data);

    if(pmu_data->dev_write_current_command_data==NULL)
    {
        pmu_data->dev_write_current_command_data = g_queue_pop_head(
            pmu_data->dev_write_command_queue);
    }

    if(pmu_data->dev_write_source==0)
    {
        pmu_data->dev_write_source = g_io_add_watch(
            pmu_data->dev_channel, G_IO_OUT,
            pcat_pmu_pm_dev_write_watch_func, pmu_data);
    }
}

static void pcat_pmu_pm_dev_read_data_parse(PCatPMUUpdaterData *pmu_data)
{
    guint i;
    gsize used_size = 0, remaining_size;
    GByteArray *buffer = pmu_data->dev_read_buffer;
    guint16 expect_len, extra_data_len;
    const guint8 *p, *extra_data;
    guint16 checksum, rchecksum;
    guint16 command;
    guint8 src, dst;
    gboolean need_ack;

    if(buffer->len < 13)
    {
        return;
    }

    for(i=0;i<buffer->len;i++)
    {
        if(buffer->data[i]==0xA5)
        {
            p = buffer->data + i;
            remaining_size = buffer->len - i;
            used_size = i;

            if(remaining_size < 13)
            {
                break;
            }

            expect_len = p[5] + ((guint16)p[6] << 8);
            if(expect_len < 3 || expect_len > 65532)
            {
                used_size = i;
                continue;
            }
            if(expect_len + 10 > remaining_size)
            {
                if(used_size > 0)
                {
                    g_byte_array_remove_range(
                        pmu_data->dev_read_buffer, 0, used_size);
                }

                return;
            }

            if(p[9+expect_len]!=0x5A)
            {
                used_size = i;
                continue;
            }

            checksum = p[7+expect_len] + ((guint16)p[8+expect_len] << 8);
            rchecksum = pcat_pmu_compute_crc16(p+1, 6+expect_len);

            if(checksum!=rchecksum)
            {
                g_warning("PM device got incorrect checksum %X, "
                    "should be %X!", checksum ,rchecksum);

                i += 9 + expect_len;
                used_size = i + 1;
                continue;
            }

            src = p[1];
            dst = p[2];

            command = p[7] + ((guint16)p[8] << 8);
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

            g_debug("Got command %X from %X to %X.", command, src, dst);

            if(pmu_data->dev_write_current_command_data!=NULL)
            {
                if(pmu_data->dev_write_current_command_data->command + 1==
                    command)
                {
                    pcat_pmu_manager_command_data_free(
                        pmu_data->dev_write_current_command_data);
                    pmu_data->dev_write_current_command_data = NULL;
                }
            }

            if(dst==0x1 || dst==0x80 || dst==0xFF)
            {
                switch(command)
                {
                    case PCAT_PMU_MANAGER_COMMAND_PMU_FW_VERSION_GET_ACK:
                    {
                        if(extra_data_len < 14)
                        {
                            break;
                        }

                        if(pmu_data->pmu_fw_version!=NULL)
                        {
                            g_free(pmu_data->pmu_fw_version);
                        }
                        pmu_data->pmu_fw_version =
                            g_strndup((const gchar *)extra_data, 14);

                        break;
                    }
                    case PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_STATUS:
                    {
                        if(extra_data_len < 1)
                        {
                            break;
                        }

                        switch(extra_data[0])
                        {
                            case 0xA7:
                            {
                                pmu_data->step =
                                    PCAT_PMU_UPDATER_STEP_PMU_FW_INFO_SEND;
                                break;
                            }
                            case 0xA4:
                            {
                                pmu_data->step = PCAT_PMU_UPDATER_STEP_ERROR;
                                pmu_data->fw_update_flag = 9;
                                fprintf(stderr, "Incorrect firmware header!\n");
                                break;
                            }
                            default:
                            {
                                fprintf(stderr,
                                    "Firmware update error, code: %X\n",
                                    extra_data[0]);
                                break;
                            }
                        }

                        break;
                    }
                    case PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_READY:
                    {
                        pmu_data->step =
                            PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_READY;
                        break;
                    }
                    case PCAT_PMU_MANAGER_COMMAND_PMU_FW_INFO_SEND_ACK:
                    {
                        if(extra_data_len < 1)
                        {
                            break;
                        }

                        printf("Firmware info check: %u\n", extra_data[0]);
                        break;
                    }
                    case PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_SEND_ACK:
                    {
                        if(extra_data_len < 1)
                        {
                            break;
                        }
                        if(extra_data[0]==0)
                        {
                            if(pmu_data->fw_data_sent + 256 <
                                pmu_data->fw_data->len)
                            {
                                pmu_data->fw_data_sent += 256;
                                pcat_pmu_updater_pmu_fw_data_send_internal(
                                    pmu_data);
                            }
                            else
                            {
                                pmu_data->step =
                                    PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_FINISH;
                                printf("\rUpdating firmware 100%%...\n");
                            }
                        }
                        else
                        {
                            pcat_pmu_updater_pmu_fw_data_send_internal(
                                pmu_data);
                        }
                        break;
                    }
                    case PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_FINISH_ACK:
                    {
                        if(extra_data_len < 1)
                        {
                            break;
                        }
                        if(extra_data[0]==0)
                        {
                            printf("PMU firmware updated successfully.\n");
                            pmu_data->fw_update_flag = 0;
                        }
                        else
                        {
                            fprintf(stderr, "Failed to update PMU firmware!\n");
                            pmu_data->fw_update_flag = 0x10;
                        }
                        pmu_data->step =
                            PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_END;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }

            i += 9 + expect_len;
            used_size = i + 1;
        }
        else
        {
            used_size = i;
        }
    }

    if(used_size > 0)
    {
        g_byte_array_remove_range(pmu_data->dev_read_buffer, 0, used_size);
    }
}

static gboolean pcat_pmu_pm_dev_read_watch_func(GIOChannel *source,
    GIOCondition condition, gpointer user_data)
{
    PCatPMUUpdaterData *pmu_data = (PCatPMUUpdaterData *)user_data;
    gssize rsize;
    guint8 buffer[4096];

    while((rsize=read(pmu_data->dev_fd, buffer, 4096))>0)
    {
        g_byte_array_append(pmu_data->dev_read_buffer, buffer, rsize);
        if(pmu_data->dev_read_buffer->len > 131072)
        {
            g_byte_array_remove_range(pmu_data->dev_read_buffer, 0,
                pmu_data->dev_read_buffer->len - 65536);
        }

        pcat_pmu_pm_dev_read_data_parse(pmu_data);
    }

    if(rsize < 0)
    {
        if(errno!=EAGAIN)
        {
            g_warning("Read PM device with error %s!", strerror(errno));

            pcat_pmu_pm_dev_close(pmu_data);
        }
    }

    return TRUE;
}

static gboolean pcat_pmu_pm_dev_open(PCatPMUUpdaterData *pmu_data)
{
    int fd;
    GIOChannel *channel;

    fd = open(PCAT_PMU_MANAGER_PM_DEV, O_RDWR | O_NDELAY);
    if(fd < 0)
    {
        fprintf(stderr, "Failed to open PM device: %s\n", strerror(errno));

        return FALSE;
    }

    channel = g_io_channel_unix_new(fd);
    if(channel==NULL)
    {
        fprintf(stderr, "Cannot open channel for PM device!\n");
        close(fd);

        return FALSE;
    }
    g_io_channel_set_flags(channel, G_IO_FLAG_NONBLOCK, NULL);

    if(pmu_data->dev_write_source > 0)
    {
        g_source_remove(pmu_data->dev_write_source);
        pmu_data->dev_write_source = 0;
    }

    if(pmu_data->dev_read_source > 0)
    {
        g_source_remove(pmu_data->dev_read_source);
        pmu_data->dev_read_source = 0;
    }
    if(pmu_data->dev_channel!=NULL)
    {
        g_io_channel_unref(pmu_data->dev_channel);
    }
    if(pmu_data->dev_fd > 0)
    {
        close(pmu_data->dev_fd);
        pmu_data->dev_fd = -1;
    }

    pmu_data->dev_fd = fd;
    pmu_data->dev_channel = channel;

    pmu_data->dev_read_source = g_io_add_watch(channel,
        G_IO_IN, pcat_pmu_pm_dev_read_watch_func, pmu_data);

    printf("Open PM device successfully.\n");

    return TRUE;
}

static void pcat_pmu_pm_dev_close(PCatPMUUpdaterData *pmu_data)
{
    if(pmu_data->dev_write_source > 0)
    {
        g_source_remove(pmu_data->dev_write_source);
        pmu_data->dev_write_source = 0;
    }

    if(pmu_data->dev_read_source > 0)
    {
        g_source_remove(pmu_data->dev_read_source);
        pmu_data->dev_read_source = 0;
    }

    if(pmu_data->dev_channel!=NULL)
    {
        g_io_channel_unref(pmu_data->dev_channel);
        pmu_data->dev_channel = NULL;
    }

    if(pmu_data->dev_fd > 0)
    {
        close(pmu_data->dev_fd);
        pmu_data->dev_fd = -1;
    }
}

static void pcat_pmu_updater_pmu_fw_version_get_internal(
    PCatPMUUpdaterData *pu_data)
{
    pcat_pmu_pm_dev_write_data_request(pu_data,
        PCAT_PMU_MANAGER_COMMAND_PMU_FW_VERSION_GET, NULL, 0, TRUE);
}

static void pcat_pmu_updater_pmu_fw_update_request_internal(
    PCatPMUUpdaterData *pu_data)
{
    pcat_pmu_pm_dev_write_data_request(pu_data,
        PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_REQUEST, NULL, 0, TRUE);
}

static void pcat_pmu_manager_pmu_fw_update_fw_info_send_internal(
    PCatPMUUpdaterData *pu_data, guint fw_raw_size)
{
    guint8 payload[32] = {0};

    memcpy(payload, "2025010100cat", 13);
    snprintf((gchar *)payload + 13, 13, "%012u", fw_raw_size);
    memcpy(payload + 25, "00000", 5);

    pcat_pmu_pm_dev_write_data_request(pu_data,
        PCAT_PMU_MANAGER_COMMAND_PMU_FW_INFO_SEND, payload, 30, TRUE);
}

static void pcat_pmu_updater_pmu_fw_data_send_internal(
    PCatPMUUpdaterData *pu_data)
{
    guint8 payload[256];
    guint percent;
    gsize payload_len;

    if(pu_data->fw_data_sent + 256 < pu_data->fw_data->len)
    {
        memcpy(payload, pu_data->fw_data->data + pu_data->fw_data_sent, 256);
        payload_len = 256;
    }
    else
    {
        memset(payload, 0xFF, 256);
        payload_len = pu_data->fw_data->len - pu_data->fw_data_sent;
        memcpy(payload, pu_data->fw_data->data + pu_data->fw_data_sent,
            payload_len);
    }

    percent = (guint)pu_data->fw_data_sent * 100 /
        pu_data->fw_data->len;
    printf("\rUpdating firmware %u%%...", percent);
    fflush(stdout);

    pcat_pmu_pm_dev_write_data_request(pu_data,
        PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_SEND, payload,
        payload_len, TRUE);
}

static void pcat_pmu_updater_pmu_fw_data_send_finish_internal(
    PCatPMUUpdaterData *pu_data)
{
    pcat_pmu_pm_dev_write_data_request(pu_data,
        PCAT_PMU_MANAGER_COMMAND_PMU_FW_UPDATE_DATA_FINISH, NULL, 0, TRUE);
}

static gboolean pcat_pmu_manager_check_timeout_func(gpointer user_data)
{
    PCatPMUUpdaterData *pu_data = (PCatPMUUpdaterData *)user_data;
    gint64 now;

    now = g_get_monotonic_time();

    if(pu_data->dev_write_source==0 &&
       pu_data->dev_write_current_command_data==NULL &&
       !g_queue_is_empty(pu_data->dev_write_command_queue))
    {
        pu_data->dev_write_source = g_io_add_watch(
            pu_data->dev_channel, G_IO_OUT,
            pcat_pmu_pm_dev_write_watch_func, pu_data);
    }

    if(pu_data->dev_write_source==0 &&
       pu_data->dev_write_current_command_data!=NULL &&
        (pu_data->dev_write_current_command_data->firstrun ||
        now > pu_data->dev_write_current_command_data->timestamp +
        PCAT_PMU_MANAGER_COMMAND_TIMEOUT))
    {
        pu_data->dev_write_source = g_io_add_watch(
            pu_data->dev_channel, G_IO_OUT,
            pcat_pmu_pm_dev_write_watch_func, pu_data);
    }

    switch(pu_data->step)
    {
        case PCAT_PMU_UPDATER_STEP_START:
        {
            pcat_pmu_updater_pmu_fw_version_get_internal(pu_data);
            pu_data->step = PCAT_PMU_UPDATER_STEP_PMU_FW_VERSION_QUERY;
            break;
        }
        case PCAT_PMU_UPDATER_STEP_WAIT:
        {
            break;
        }
        case PCAT_PMU_UPDATER_STEP_ERROR:
        {
            g_main_loop_quit(pu_data->main_loop);
            break;
        }
        case PCAT_PMU_UPDATER_STEP_PMU_FW_VERSION_QUERY:
        {
            if(pu_data->pmu_fw_version==NULL)
            {
                break;
            }

            printf("Firmware version on PMU: %s\n", pu_data->pmu_fw_version);

            if(g_pcat_pmu_updater_cmd_get_pwu_fw_version)
            {
                g_main_loop_quit(pu_data->main_loop);
                break;
            }

            if(!g_pcat_pmu_updater_cmd_force)
            {
                if(g_strcmp0(pu_data->pmu_fw_version + 5,
                    pu_data->fw_version + 5) >= 0)
                {
                    printf("Firmware on PMU is already updated!\n");
                    g_main_loop_quit(pu_data->main_loop);
                    break;
                }
            }

            pcat_pmu_updater_pmu_fw_update_request_internal(pu_data);
            pu_data->step = PCAT_PMU_UPDATER_STEP_WAIT;

            break;
        }
        case PCAT_PMU_UPDATER_STEP_PMU_FW_INFO_SEND:
        {
            pcat_pmu_manager_pmu_fw_update_fw_info_send_internal(pu_data,
                pu_data->fw_data->len);
            pu_data->step = PCAT_PMU_UPDATER_STEP_WAIT;

            break;
        }
        case PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_READY:
        {
            printf("PMU firmware update ready, sending firmware...\n");

            pu_data->fw_data_sent = 0;
            pcat_pmu_updater_pmu_fw_data_send_internal(pu_data);
            pu_data->step = PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_WORKING;

            break;
        }
        case PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_WORKING:
        {
            break;
        }
        case PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_FINISH:
        {
            pcat_pmu_updater_pmu_fw_data_send_finish_internal(pu_data);

            pu_data->step = PCAT_PMU_UPDATER_STEP_WAIT;
            break;
        }
        case PCAT_PMU_UPDATER_STEP_PMU_FW_UPDATE_END:
        {
            g_main_loop_quit(pu_data->main_loop);
            break;
        }
        default:
        {
            break;
        }
    }

    return TRUE;
}


static gboolean pcat_pmu_updater_fw_load(PCatPMUUpdaterData *pu_data,
    const gchar *path)
{
    guint8 *fw_data = NULL;
    gsize fw_data_size = 0;
    GError *error = NULL;
    const guint8 *fw_version;
    guint32 fw_raw_size;
    guint32 fw_raw_crc;
    gint64 fw_timestamp;
    uLong fw_real_crc;

    if(!g_file_get_contents(path, (gchar **)&fw_data, &fw_data_size, &error))
    {
        fprintf(stderr, "Failed to open firmware file %s: %s\n",
            path, error!=NULL ? error->message : "unknown error");
        return FALSE;
    }

    if(fw_data_size < 512)
    {
        g_free(fw_data);
        fprintf(stderr, "Firmware %s file size too small\n!", path);
        return FALSE;
    }

    if(memcmp(fw_data, "ARBDPHC2", 8)!=0)
    {
        g_free(fw_data);
        fprintf(stderr, "Invalid file header of firmware file %s!\n", path);
        return FALSE;
    }

    fw_version = fw_data + 8;

    memcpy(&fw_raw_size, fw_data + 22, 4);
    fw_raw_size = GUINT32_FROM_LE(fw_raw_size);

    memcpy(&fw_raw_crc, fw_data + 26, 4);
    fw_raw_crc = GUINT32_FROM_LE(fw_raw_crc);

    memcpy(&fw_timestamp, fw_data + 30, 8);
    fw_timestamp = GINT64_FROM_LE(fw_timestamp);


    if(memcmp(fw_version, "RA2E1", 5)!=0)
    {
        g_free(fw_data);
        fprintf(stderr, "Invalid version of firmware file %s!\n", path);
        return FALSE;
    }

    if(fw_raw_size+512!=fw_data_size)
    {
        g_free(fw_data);
        fprintf(stderr, "Invalid size of firmware file %s!\n", path);
        return FALSE;
    }

    fw_real_crc = crc32(0L, Z_NULL, 0);
    fw_real_crc = crc32(fw_real_crc, fw_data + 512, fw_raw_size);

    if(fw_raw_crc!=fw_real_crc)
    {
        g_free(fw_data);
        fprintf(stderr, "Firmware file %s checksum error!\n", path);
        return FALSE;
    }

    pu_data->fw_data = g_byte_array_new();
    g_byte_array_append(pu_data->fw_data, fw_data + 512, fw_raw_size);
    memcpy(pu_data->fw_version, fw_version, 14);
    g_free(fw_data);

    printf("Loaded firmware file %s, version %s, size %lu bytes.\n", path,
        pu_data->fw_version, (gulong)fw_raw_size);

    return TRUE;
}

int main(int argc, char *argv[])
{
    GError *error = NULL;
    GOptionContext *context;
    guint i;
    gboolean device_matched = FALSE;
    gint ret;

    context = g_option_context_new("- PCat PMU Updater");
    g_option_context_set_ignore_unknown_options(context, TRUE);
    g_option_context_add_main_entries(context, g_pcat_pmu_updater_cmd_entries,
        "PCPU");
    if(!g_option_context_parse(context, &argc, &argv, &error))
    {
        fprintf(stderr, "Option parsing failed: %s\n", error->message);
        g_clear_error(&error);
    }

    G_STMT_START
    {
        if(!g_file_get_contents("/proc/device-tree/compatible",
            &g_pcat_pmu_updater_data.dtb_compatible,
            &g_pcat_pmu_updater_data.dtb_compatible_len, &error))
        {
            fprintf(stderr, "Failed to get device compatible information: %s\n",
                error!=NULL ? error->message : "Unknown");
            ret = 1;
            break;
        }

        for(i=0;g_pcat_pmu_dtb_compatible_name[i]!=NULL;i++)
        {
            if(g_strstr_len(g_pcat_pmu_updater_data.dtb_compatible, -1,
                g_pcat_pmu_dtb_compatible_name[i])!=NULL)
            {
                device_matched = TRUE;
            }
        }

        if(!device_matched)
        {
            fprintf(stderr, "This device is not compatible for "
                "PMU firmware update!\n");
            ret = 1;
            break;
        }

        g_pcat_pmu_updater_data.main_loop = g_main_loop_new(NULL, FALSE);

        g_pcat_pmu_updater_data.dev_read_buffer = g_byte_array_new();
        g_pcat_pmu_updater_data.dev_write_command_queue = g_queue_new();
        g_pcat_pmu_updater_data.dev_write_current_command_data = NULL;

        if(!g_pcat_pmu_updater_cmd_get_pwu_fw_version)
        {
            if(argc < 2)
            {
                fprintf(stderr, "Please specify firmware file to update.\n");
                return 2;
            }

            if(!pcat_pmu_updater_fw_load(&g_pcat_pmu_updater_data, argv[1]))
            {
                return 3;
            }
        }

        if(!pcat_pmu_pm_dev_open(&g_pcat_pmu_updater_data))
        {
            return 4;
        }

        g_pcat_pmu_updater_data.check_timeout_id = g_timeout_add(100,
            pcat_pmu_manager_check_timeout_func, &g_pcat_pmu_updater_data);

        g_main_loop_run(g_pcat_pmu_updater_data.main_loop);

        if(g_pcat_pmu_updater_data.check_timeout_id > 0)
        {
            g_source_remove(g_pcat_pmu_updater_data.check_timeout_id);
            g_pcat_pmu_updater_data.check_timeout_id = 0;
        }

        ret = g_pcat_pmu_updater_data.fw_update_flag;
    }
    G_STMT_END;

    pcat_pmu_pm_dev_close(&g_pcat_pmu_updater_data);

    if(g_pcat_pmu_updater_data.dev_write_current_command_data!=NULL)
    {
        pcat_pmu_manager_command_data_free(
            g_pcat_pmu_updater_data.dev_write_current_command_data);
        g_pcat_pmu_updater_data.dev_write_current_command_data = NULL;
    }
    if(g_pcat_pmu_updater_data.dev_write_command_queue!=NULL)
    {
        g_queue_free_full(g_pcat_pmu_updater_data.dev_write_command_queue,
            (GDestroyNotify)pcat_pmu_manager_command_data_free);
        g_pcat_pmu_updater_data.dev_write_command_queue = NULL;
    }

    if(g_pcat_pmu_updater_data.dev_read_buffer!=NULL)
    {
        g_byte_array_unref(g_pcat_pmu_updater_data.dev_read_buffer);
        g_pcat_pmu_updater_data.dev_read_buffer = NULL;
    }

    if(g_pcat_pmu_updater_data.pmu_fw_version!=NULL)
    {
        g_free(g_pcat_pmu_updater_data.pmu_fw_version);
        g_pcat_pmu_updater_data.pmu_fw_version = NULL;
    }

    if(g_pcat_pmu_updater_data.main_loop!=NULL)
    {
        g_main_loop_unref(g_pcat_pmu_updater_data.main_loop);
        g_pcat_pmu_updater_data.main_loop = NULL;
    }

    if(g_pcat_pmu_updater_data.fw_data!=NULL)
    {
        g_byte_array_unref(g_pcat_pmu_updater_data.fw_data);
        g_pcat_pmu_updater_data.fw_data = NULL;
    }

    if(g_pcat_pmu_updater_data.dtb_compatible!=NULL)
    {
        g_free(g_pcat_pmu_updater_data.dtb_compatible);
        g_pcat_pmu_updater_data.dtb_compatible = NULL;
    }

    return ret;
}


