/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEAtrue_SENSOR
** File: - oplus_cm_fb.c
** Description: Source file for oplus client manager feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_pb_util.h"
#include "sns_mem_util.h"
#include "sns_memmgr.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#include "sns_island.h"
#include "sns_osa_lock.h"
#include "sns_client.pb.h"
#include "sns_resampler.pb.h"
#include "sns_fw_sensor.h"
#include "sns_fw_attribute_service.h"
#include "sns_isafe_list.h"
#include "sns_sensor_uid.h"
#include "sns_fw_sensor_instance.h"
#include "sns_fw_data_stream.h"
#include "sns_fw_request.h"

#include "oplus_list.h"
#include "oplus_fb_utils.h"

#define FB_MALLOC(x)  sns_malloc(SNS_HEAP_MAIN, x)
#define FB_ISLAND_MALLOC(x)  sns_malloc(SNS_HEAP_ISLAND, x)

static float normal_mode_wakeup_rate;
static float island_mode_wakeup_rate;
static sns_osa_lock *cm_lock = NULL;

struct oplus_instance {
    struct list_head list;
    sns_sensor_instance* this;
    sns_sensor_uid msg_suid;
    int proc_type;
    int delivery_type;
    char src_name[24];
};

struct oplus_sensor {
    sns_fw_sensor *sensor; //read only
    sns_sensor_uid suid;
    struct oplus_instance *cm_instance;
    char *const name;
    bool is_physical_sensor;
    bool is_available;
    bool is_enable;
    int proc_type;
    int delivery_type;
    float wakeup_rate;
};

struct oplus_src_suid {
    sns_sensor_uid src_suid;
    bool is_valid;
};

struct oplus_src_suid g_src_suid = {
    .is_valid = true,
};

static bool is_oplus_sensor_list_inited = false;

static struct oplus_sensor physical_sensor_list[] = {
    {
        .sensor = NULL,
        .name = "proximity",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
    {
        .name = "wise_light",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
    {
        .name = "ambient_light",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
    {
        .name = "accel",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
    {
        .name = "gyro",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
    {
        .name = "mag",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
    {
        .name = "resampler",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = true,
    },
};

static struct oplus_sensor virtual_sensor_list[]  = {
    {
        .name = "cm",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
    {
        .name = "lux_aod",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
    {
        .name = "pick_up_motion",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
    {
        .name = "pedometer",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
    {
        .name = "pedometer_minute",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
    {
        .name = "free_fall",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
    {
        .name = "amd_oppo",
        .is_available = false,
        .is_enable = false,
        .is_physical_sensor = false,
    },
};

void oplus_collect_wakeup_rate(float normal_rate, float island_rate)
{
    normal_mode_wakeup_rate = normal_rate;
    island_mode_wakeup_rate = island_rate;
}

float oplus_get_normal_wake_up_rate(void)
{
    return normal_mode_wakeup_rate;
}

float oplus_get_island_wake_up_rate(void)
{
    return island_mode_wakeup_rate;
}

void oplus_init_fw_sensor(void)
{
    sns_isafe_list *libraries = NULL;
    sns_isafe_list_iter iter_library, iter_sensor;
    char data_type[64];
    int index = 0;
    const sns_sensor_uid *suid;
    sns_osa_lock_attr attr;

    if (!virtual_sensor_list[0].sensor) {
        SNS_PRINTF(ERROR, sns_fw_printf, "null sensor");
        return;
    }

    libraries = virtual_sensor_list[0].sensor->library->list_entry.list;//library head
    if (!libraries) {
        SNS_PRINTF(ERROR, sns_fw_printf, "null libraries");
        return;
    }

    if (!cm_lock) {
        sns_osa_lock_attr_init(&attr);
        sns_osa_lock_attr_set_memory_partition(&attr, SNS_OSA_MEM_TYPE_ISLAND);
        sns_osa_lock_create(&attr, &cm_lock);
    }

    SNS_ISLAND_EXIT();
    for (sns_isafe_list_iter_init(&iter_library, libraries, true);
        NULL != sns_isafe_list_iter_curr(&iter_library);
        sns_isafe_list_iter_advance(&iter_library)) {

        sns_sensor_library *library = (sns_sensor_library*)
            sns_isafe_list_iter_get_curr_data(&iter_library);

        for (sns_isafe_list_iter_init(&iter_sensor, &library->sensors, true);
            NULL != sns_isafe_list_iter_curr(&iter_sensor);
            sns_isafe_list_iter_advance(&iter_sensor)) {

            sns_fw_sensor *fw_sensor = (sns_fw_sensor*)
                sns_isafe_list_iter_get_curr_data(&iter_sensor);
            /*physical sensor*/
            for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {
                sns_attr_info_get_data_type(fw_sensor->attr_info, data_type, sizeof(data_type));
                if (!strcmp(physical_sensor_list[index].name, data_type)) {
                    if (!physical_sensor_list[index].is_available) {
                        physical_sensor_list[index].sensor = fw_sensor;
                        suid = ((sns_sensor*)fw_sensor)->sensor_api->get_sensor_uid((sns_sensor*)fw_sensor);
                        if (suid) {
                            memcpy(&physical_sensor_list[index].suid , suid, sizeof(sns_sensor_uid));
                        }
                        physical_sensor_list[index].is_available = fw_sensor->attr_info->available;
                        if (fw_sensor->sensor_instances.cnt > 0)
                            physical_sensor_list[index].is_enable = true;
                        else
                            physical_sensor_list[index].is_enable = false;
                    }
                    break;
                }
            }
            /*virtual sensor*/
            for (index = 0; index < (sizeof(virtual_sensor_list) / sizeof(struct oplus_sensor)); index++) {
                sns_attr_info_get_data_type(fw_sensor->attr_info, data_type, sizeof(data_type));
                if (!strcmp(virtual_sensor_list[index].name, data_type)) {
                    virtual_sensor_list[index].sensor = fw_sensor;
                    suid = ((sns_sensor*)fw_sensor)->sensor_api->get_sensor_uid((sns_sensor*)fw_sensor);
                    if (suid) {
                        memcpy(&virtual_sensor_list[index].suid , suid, sizeof(sns_sensor_uid));
                    }
                    virtual_sensor_list[index].is_available = fw_sensor->attr_info->available;
                    if (fw_sensor->sensor_instances.cnt > 0)
                        virtual_sensor_list[index].is_enable = true;
                    else
                        virtual_sensor_list[index].is_enable = false;
                    break;
                }
            }
        }
    }
    is_oplus_sensor_list_inited = true;
}

void oplus_init_cm_fw_sensor(sns_sensor_instance *this)
{

    sns_fw_sensor *const fw_sensor = (sns_fw_sensor *)(((sns_fw_sensor_instance *)this)->sensor);
    char data_type[64];

    if (!fw_sensor) {
        SNS_PRINTF(ERROR, sns_fw_printf, "cm null sensor");
        return;
    }
    sns_attr_info_get_data_type(fw_sensor->attr_info, data_type, sizeof(data_type));

    if (!strcmp(virtual_sensor_list[0].name, data_type)) {
        virtual_sensor_list[0].sensor = fw_sensor;
        virtual_sensor_list[0].is_available = fw_sensor->attr_info->available;
        if (fw_sensor->sensor_instances.cnt > 0)
            virtual_sensor_list[0].is_enable = true;
        else
            virtual_sensor_list[0].is_enable = false;
    }
}

static struct oplus_sensor *oplus_find_sensor(const sns_sensor_uid *suid)
{
    int index = 0;
    bool find = false;
    struct oplus_sensor *sensor = NULL;

    /*physical sensor*/
    for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (sns_sensor_uid_compare(suid, &physical_sensor_list[index].suid)) {
            if (!physical_sensor_list[index].is_available && (physical_sensor_list[index].is_available !=
                physical_sensor_list[index].sensor->attr_info->available)) {
                physical_sensor_list[index].is_available = physical_sensor_list[index].sensor->attr_info->available;
            }
            if (physical_sensor_list[index].is_available) {
                find = true;
                sensor = &physical_sensor_list[index];
                break;
            }
        }
    }

    if (!find) {
        for (index = 0; index < (sizeof(virtual_sensor_list) / sizeof(struct oplus_sensor)); index++) {
            if (sns_sensor_uid_compare(suid, &virtual_sensor_list[index].suid)) {
                if (!virtual_sensor_list[index].is_available && (virtual_sensor_list[index].is_available !=
                    virtual_sensor_list[index].sensor->attr_info->available)) {
                    virtual_sensor_list[index].is_available = virtual_sensor_list[index].sensor->attr_info->available;
                }
                if (virtual_sensor_list[index].is_available) {
                    find = true;
                    sensor = &virtual_sensor_list[index];
                    break;
                }
            }
        }
    }

    return sensor;
}

bool oplus_is_resampler_request(const sns_sensor_uid *suid)
{
    struct oplus_sensor *sensor = NULL;
    bool find = false;

    if (!is_oplus_sensor_list_inited) {
        return find;
    }

    sensor = oplus_find_sensor(suid);
    if (sensor) {
        if (sensor->is_physical_sensor) {
            if (!strcmp(sensor->name, "resampler")) {
                find = true;
            }
        }
    }
    return find;
}

typedef struct {
    sns_std_request std_request;
    sns_resampler_config config;
} resampler_request;

void oplus_parse_resampler_request(sns_request* request)
{
    pb_simple_cb_arg arg;
    pb_istream_t stream;

    if (!request) {
        SNS_PRINTF(ERROR, sns_fw_printf, "request fail");
        return;
    }

    resampler_request oplus_resampler_req = {
        .std_request = sns_std_request_init_default,
        .config = sns_resampler_config_init_default
    };

    arg.decoded_struct = &oplus_resampler_req.config;
    arg.fields = sns_resampler_config_fields;
    oplus_resampler_req.std_request.payload = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_simple_cb,
        .arg = &arg
    };
    stream = pb_istream_from_buffer(request->request, request->request_len);

    if (pb_decode(&stream, sns_std_request_fields, &oplus_resampler_req.std_request)) {
        sns_memscpy(&g_src_suid.src_suid, sizeof(sns_sensor_uid),
            (sns_sensor_uid const *)&oplus_resampler_req.config.sensor_uid,
            sizeof(sns_sensor_uid));
        g_src_suid.is_valid = true;
    } else {
        SNS_PRINTF(ERROR, sns_fw_printf, "oplus_parse_resampler_request pb_decode fail");
    }
}

static void oplus_update_source_name(struct oplus_instance * this, const sns_sensor_uid *src_suid)
{
    int index = 0;
    bool find = false;

    if (!is_oplus_sensor_list_inited) {
        return;
    }

    if (!src_suid) {
        SNS_PRINTF(ERROR, sns_fw_printf, "suid null");
        return;
    }

    for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (sns_sensor_uid_compare(src_suid, &physical_sensor_list[index].suid)) {
            find = true;
            break;
        }
    }
    if (find) {
        strncpy(this->src_name, physical_sensor_list[index].name, 24);
    }
}

static int oplus_update_cm_info(struct oplus_sensor *sensor, sns_sensor_instance* this,
    const sns_sensor_uid *suid, int proc_type, int delivery_type, bool is_remove)
{
    struct oplus_instance *pos = NULL;
    struct oplus_instance *new = NULL;
    bool find = false;

    if (!sensor || !this)
        return -1;

    if (is_remove) {
        if (sensor->cm_instance) {
            list_for_each_entry(pos, struct oplus_instance, &(sensor->cm_instance->list), list) {
                if (pos->this == this) {
                    find = true;
                    break;
                }
            }
            if (find) {
                list_del(&(pos->list));
                sns_free(pos);
            }
        }
    } else {
        if (!sensor->cm_instance) {//null HEAD
            sensor->cm_instance = (struct oplus_instance *)FB_ISLAND_MALLOC(sizeof(struct oplus_instance));
            if (!sensor->cm_instance) {
                SNS_PRINTF(ERROR, sns_fw_printf, "malloc fail");
                return -1;
            }
            sensor->proc_type = 0;
            sensor->delivery_type = 0;
            sensor->cm_instance->this = NULL;
            INIT_LIST_HEAD(&sensor->cm_instance->list);
            new = (struct oplus_instance *)FB_ISLAND_MALLOC(sizeof(struct oplus_instance));
            if (!new) {
                SNS_PRINTF(ERROR, sns_fw_printf, "malloc fail");
                return -1;
            }
            new->proc_type = proc_type;
            new->delivery_type = delivery_type;
            new->this = this;
            memcpy(&new->msg_suid, suid, sizeof(sns_sensor_uid));
            if (oplus_is_resampler_request(suid)) {
                if (g_src_suid.is_valid) {
                    g_src_suid.is_valid = false;
                    oplus_update_source_name(new, &g_src_suid.src_suid);
                }
            }
            list_add(&(new->list), &(sensor->cm_instance->list));
        } else {
            list_for_each_entry(pos, struct oplus_instance, &(sensor->cm_instance->list), list) {
                if (pos->this == this) {
                    find = true;
                    break;
                }
            }
            if (find) {
                pos->proc_type = proc_type;
                pos->delivery_type = delivery_type;
            } else {
                new = (struct oplus_instance *)FB_ISLAND_MALLOC(sizeof(struct oplus_instance));
                if (!new) {
                    SNS_PRINTF(ERROR, sns_fw_printf, "malloc fail");
                    return -1;
                }
                new->proc_type = proc_type;
                new->delivery_type = delivery_type;
                new->this = this;
                memcpy(&new->msg_suid, suid, sizeof(sns_sensor_uid));
                if (oplus_is_resampler_request(suid)) {
                    if (g_src_suid.is_valid) {
                        g_src_suid.is_valid = false;
                        oplus_update_source_name(new, &g_src_suid.src_suid);
                    }
                }
                list_add(&(new->list), &(sensor->cm_instance->list));
            }
        }
    }
    return 0;
}

/*
*  delivery_type: 0: wakeup,1: no_wakeup
*  proc_type: SSC = 0, APSS = 1, ADSP = 2, MDSP = 3, CDSP = 4
*/
void oplus_collect_cm_info(sns_sensor_instance* this, const sns_sensor_uid *suid,
    int proc_type, int delivery_type)
{
    struct oplus_sensor *sensor = NULL;

    if (!is_oplus_sensor_list_inited) {
        return;
    }

    sensor = oplus_find_sensor(suid);
    if (sensor) {
        sns_osa_lock_acquire(cm_lock);
        if (sensor->is_physical_sensor) {
            oplus_update_cm_info(sensor, this, suid, proc_type, delivery_type, false);
        }
        sns_osa_lock_release(cm_lock);
    }
}

void oplus_clear_cm_info(sns_sensor_instance* this, const sns_sensor_uid *suid)
{
    struct oplus_sensor *sensor = NULL;

    if (!is_oplus_sensor_list_inited) {
        return;
    }

    sensor = oplus_find_sensor(suid);
    if (sensor) {
        sns_osa_lock_acquire(cm_lock);
        if (sensor->is_physical_sensor) {
            oplus_update_cm_info(sensor, this, suid, 0, 0, true);
        }
        sns_osa_lock_release(cm_lock);
    }
}

void oplus_set_sensor_wakeup_rate(const sns_sensor_uid *suid, float wakeup_rate)
{
    struct oplus_sensor *sensor = NULL;

    if (!is_oplus_sensor_list_inited) {
        return;
    }

    sensor = oplus_find_sensor(suid);
    if (sensor) {
        sensor->wakeup_rate = wakeup_rate;
        if (strcmp(sensor->name, "proximity")) {//except proximity
            if (wakeup_rate)
                sensor->is_enable = true;
            else
                sensor->is_enable = false;
        }
    }
}

void oplus_set_proximity_enable(bool enable)
{
    physical_sensor_list[0].is_enable = enable;
}

void oplus_show_fw_sensor(void)
{
    int index = 0;

    SNS_PRINTF(ERROR, sns_fw_printf, "oplus_show_fw_sensor:");

    for (index = 0; index < sizeof(physical_sensor_list) / sizeof(struct oplus_sensor); index++) {
        SNS_SPRINTF(ERROR, sns_fw_printf, "name:%s,is_available:%d,is_enable:%d,proc_type:%d,delivery_type:%d,rate:%f",
            physical_sensor_list[index].name,
            physical_sensor_list[index].is_available,
            physical_sensor_list[index].is_enable,
            physical_sensor_list[index].proc_type,
            physical_sensor_list[index].delivery_type,
            physical_sensor_list[index].wakeup_rate);
    }
    for (index = 0; index < (sizeof(virtual_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        SNS_SPRINTF(ERROR, sns_fw_printf, "name:%s,is_available:%d,is_enable:%d,proc_type:%d,delivery_type:%d,rate:%f",
            virtual_sensor_list[index].name,
            virtual_sensor_list[index].is_available,
            virtual_sensor_list[index].is_enable,
            virtual_sensor_list[index].proc_type,
            virtual_sensor_list[index].delivery_type,
            virtual_sensor_list[index].wakeup_rate);
    }
}

static void oplus_update_virtual_list(void)
{
    int index = 0;

    for (index = 0; index < (sizeof(virtual_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (virtual_sensor_list[index].is_available && virtual_sensor_list[index].sensor) {
            if (virtual_sensor_list[index].sensor->sensor_instances.cnt > 0) {
                virtual_sensor_list[index].is_enable = true;
            } else {
                virtual_sensor_list[index].is_enable = false;
            }
        }
    }
}

static void oplus_save_enable_sensor_info(void)
{
    int index = 0;
    struct fb_event event;
    int enable_physical_sensor = 0;
    int enable_virtual_sensor = 0;

    memset(&event, 0, sizeof(struct fb_event));

    oplus_update_virtual_list();

    for (index = 0; index < (sizeof(virtual_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (virtual_sensor_list[index].is_enable) {
            enable_virtual_sensor |= (1 << index);
        }
    }
    event.buff[0] = enable_virtual_sensor;

    for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (physical_sensor_list[index].is_available && physical_sensor_list[index].sensor) {
            if (physical_sensor_list[index].is_enable) {
                enable_physical_sensor |= (1 << index);
            }
        }
    }
    event.buff[1] = enable_physical_sensor;

    event.event_id = POWER_SENSOR_INFO_ID;
    oplus_add_fd_event(&event);

    SNS_SPRINTF(ERROR, sns_fw_printf, "oplus_save_enable_sensor_info event_id:%d,data[0x%x,0x%x,0x%x]",
        event.event_id, event.buff[0], event.buff[1], event.buff[2]);
}

void oplus_save_available_sensor_info(void)
{
    int index = 0;
    struct fb_event event;
    int available_physical_sensor = 0;
    int available_virtual_sensor = 0;

    memset(&event, 0, sizeof(struct fb_event));

    for (index = 0; index < (sizeof(virtual_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (virtual_sensor_list[index].is_available) {
            available_virtual_sensor |= (1 << index);
        }
    }
    event.buff[0] = available_virtual_sensor;

    for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (physical_sensor_list[index].is_available) {
            available_physical_sensor |= (1 << index);
        }
    }
    event.buff[1] = available_physical_sensor;

    event.event_id = ALAILABLE_SENSOR_LIST_ID;
    oplus_add_fd_event(&event);

    SNS_SPRINTF(ERROR, sns_fw_printf, "oplus_save_available_sensor_info event_id:%d,data[0x%x,0x%x,0x%x]",
        event.event_id, event.buff[0], event.buff[1], event.buff[2]);
}

static void oplus_save_wakeup_rate_info(void)
{
    struct fb_event event;

    memset(&event, 0, sizeof(struct fb_event));

    event.event_id = POWER_WAKE_UP_RATE_ID;
    event.buff[0] = (int)normal_mode_wakeup_rate;
    event.buff[1] = (int)island_mode_wakeup_rate;

    oplus_add_fd_event(&event);
}


struct physical_sensor_fb {
    char *const name;
    enum sensor_fb_event_id id;
};

static struct physical_sensor_fb fb_list[] = {
    {
        .name = "proximity",
        .id = POWER_PROXIMITY_INFO_ID,
    },
    {
        .name = "wise_light",
        .id = POWER_WISE_LIGHT_INFO_ID,
    },
    {
        .name = "ambient_light",
        .id = POWER_LIGHT_INFO_ID,
    },
    {
        .name = "accel",
        .id = POWER_ACCEL_INFO_ID,
    },
    {
        .name = "gyro",
        .id = POWER_GYRO_INFO_ID,
    },
    {
        .name = "mag",
        .id = POWER_MAG_INFO_ID,
    },
};

static void oplus_save_cm_info(void)
{
    int index = 0;
    int ii = 0;
    int shift_count = 0;
    bool is_full = false;
    bool find = false;
    struct oplus_instance *pos = NULL;
    struct oplus_sensor *resampler = NULL;
    struct fb_event event;

    memset(&event, 0, sizeof(struct fb_event));

    sns_osa_lock_acquire(cm_lock);
    //for resampler
    for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {
        if (!strcmp("resampler", physical_sensor_list[index].name)) {
            resampler = &physical_sensor_list[index];
            break;
        }
    }
    for (index = 0; index < (sizeof(physical_sensor_list) / sizeof(struct oplus_sensor)); index++) {

        for (ii = 0; ii < (sizeof(fb_list) / sizeof(struct physical_sensor_fb)); ii++) {
            if (!strcmp(fb_list[ii].name, physical_sensor_list[index].name)) {
                if (physical_sensor_list[index].is_enable) {
                    find = true;
                }
                if (physical_sensor_list[index].cm_instance) {//for each sensor opened by cm directly
                    list_for_each_entry(pos, struct oplus_instance, &(physical_sensor_list[index].cm_instance->list), list) {
                        event.event_id = fb_list[ii].id;
                        event.buff[0] |= pos->proc_type << shift_count;
                        event.buff[1] |= pos->delivery_type << shift_count;
                        event.buff[2] = physical_sensor_list[index].wakeup_rate;
                        shift_count += 3;
                        if (shift_count > 32) {
                            is_full = true;
                            break;
                        }
                    }
                }
            }
            if (find && !is_full && resampler && resampler->cm_instance) {//for each sensor opened by resampler
                list_for_each_entry(pos, struct oplus_instance, &(resampler->cm_instance->list), list) {
                    if (!strcmp(pos->src_name, fb_list[ii].name)) {
                        event.event_id = fb_list[ii].id;
                        event.buff[0] |= pos->proc_type << shift_count;
                        event.buff[1] |= pos->delivery_type << shift_count;
                        event.buff[2] = physical_sensor_list[index].wakeup_rate;
                        shift_count += 3;
                        if (shift_count > 32) {
                            break;
                        }
                    }
                }
            }
            if (find) {
                SNS_SPRINTF(ERROR, sns_fw_printf, "oplus_save_cm_info name:%s, event_id:%d,data[0x%x,0x%x,0x%x]",
                    fb_list[ii].name, event.event_id, event.buff[0], event.buff[1], event.buff[2]);
                find = false;
                is_full = false;
                oplus_add_fd_event(&event);
                shift_count = 0;
                memset(&event, 0, sizeof(struct fb_event));
                break;
            }
        }
    }
    sns_osa_lock_release(cm_lock);
}

void oplus_save_power_info(void)
{
    oplus_save_enable_sensor_info();
    oplus_save_wakeup_rate_info();
    oplus_save_cm_info();
}
