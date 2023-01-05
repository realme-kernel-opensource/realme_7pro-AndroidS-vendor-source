/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_sensor_fb.cpp
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "sns_mem_util.h"
#include "sns_memmgr.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#include "sns_island.h"
#include "smem.h"
#include "sns_osa_lock.h"

#include "oplus_list.h"
#include "oplus_sensor_fb.h"

#define FB_MALLOC(x)  sns_malloc(SNS_HEAP_MAIN, x)
#define FB_ISLAND_MALLOC(x)  sns_malloc(SNS_HEAP_ISLAND, x)

#define SMEM_SENSOR_FEEDBACK 128

static bool fd_init = false;
static int island_fifo_len = 0;
static int smem_fifo_len = 0;
static struct fb_event_smem *smem_fifo = NULL;
static sns_osa_lock *fd_lock = NULL;
static bool has_main_heap_malloc = false;

static struct fb_event_node f_head = {
    .event = {
        .event_id = FD_HEAD_EVENT_ID,
        .count = 0xffff
    },
};

void oplus_feedback_init(void)
{
    uint32 nSize = 0;
    sns_osa_lock_attr attr;

    if (!fd_init) {
        INIT_LIST_HEAD(&f_head.list);

        SNS_ISLAND_EXIT();
        if (!fd_lock) {
            sns_osa_lock_attr_init(&attr);
            sns_osa_lock_attr_set_memory_partition(&attr, SNS_OSA_MEM_TYPE_ISLAND);
            sns_osa_lock_create(&attr, &fd_lock);
        }
        smem_fifo = (struct fb_event_smem *)smem_get_addr(SMEM_SENSOR_FEEDBACK, &nSize);
        if (smem_fifo == NULL || nSize == 0) {
            SNS_PRINTF(ERROR, sns_fw_printf, "invalid smem %d", nSize);
            return;
        }
        memset(smem_fifo, 0, sizeof(struct fb_event_smem));
        SNS_PRINTF(ERROR, sns_fw_printf, "smem_fifo  %p", smem_fifo);

        fd_init = true;
    }
}

int oplus_add_fd_event(struct fb_event *event)
{
    bool find = false;
    struct fb_event_node *pos = NULL;
    struct fb_event_node *new_s = NULL;

    if (!event) {
        SNS_PRINTF(ERROR, sns_fw_printf, "invalid event");
        return -EINVAL;
    }

    if (!fd_init) {
        SNS_PRINTF(ERROR, sns_fw_printf, "not fd_init");
        return -EINVAL;
    }

    sns_osa_lock_acquire(fd_lock);
    if (has_main_heap_malloc) {
        SNS_ISLAND_EXIT();
    }

    list_for_each_entry(pos, struct fb_event_node, &(f_head.list), list) {
        if (event->event_id == pos->event.event_id) {
            find = true;
            SNS_PRINTF(ERROR, sns_fw_printf, "find event_id %d", event->event_id);
            break;
        }
    }

    if (!find) {
        if (island_fifo_len >= EVNET_NUM_MAX) {
            SNS_PRINTF(ERROR, sns_fw_printf, "event count limited %d", island_fifo_len);
            sns_osa_lock_release(fd_lock);
            return -EINVAL;
        }

        new_s = (struct fb_event_node*)FB_ISLAND_MALLOC(sizeof(struct fb_event_node));
        if (!new_s) {
            SNS_PRINTF(ERROR, sns_fw_printf, "island malloc fail,try to malloc in main");
            SNS_ISLAND_EXIT();
            new_s = (struct fb_event_node*)FB_MALLOC(sizeof(struct fb_event_node));
            if (!new_s) {
                SNS_PRINTF(ERROR, sns_fw_printf, "malloc fail");
                sns_osa_lock_release(fd_lock);
                return -EINVAL;
            }
            has_main_heap_malloc = true;
        }

        memset(new_s, 0, sizeof(struct fb_event_node));
        memcpy(&new_s->event, event, sizeof(struct fb_event));
        if (event->count <= 0) {
            new_s->event.count = 1;
        }
        list_add(&(new_s->list), &(f_head.list));
        island_fifo_len++;

        SNS_PRINTF(ERROR, sns_fw_printf, "add event_id %d count %d, island_fifo_len = %d",
            event->event_id, event->count, island_fifo_len);
        event->count = 0;
    } else { //update event
        if (event->count > 0) {
            pos->event.count += event->count;
        } else {
            pos->event.count++;
        }
        event->count = 0;
        memcpy(pos->event.buff, event->buff, sizeof(int) * EVNET_DATA_LEN);
        SNS_PRINTF(ERROR, sns_fw_printf, "update event %d count %d", event->event_id, pos->event.count);
    }
    sns_osa_lock_release(fd_lock);
    return 0;
}

int oplus_remove_fd_event(struct fb_event *event)
{
    bool find = false;
    struct fb_event_node *pos = NULL;

    if (!event || !fd_init) {
        SNS_PRINTF(ERROR, sns_fw_printf, "invalid event");
        return -EINVAL;
    }

    sns_osa_lock_acquire(fd_lock);
    if (has_main_heap_malloc) {
        SNS_ISLAND_EXIT();
    }

    list_for_each_entry(pos, struct fb_event_node, &(f_head.list), list) {
        if (event->event_id == pos->event.event_id) {
            find = true;
            SNS_PRINTF(ERROR, sns_fw_printf, "find event_id %d\n", event->event_id);
            break;
        }
    }

    if (find) {
        SNS_PRINTF(ERROR, sns_fw_printf, "delet event %d \n", event->event_id);
        list_del(&(pos->list));
        sns_free(pos);
        island_fifo_len--;
    }
    sns_osa_lock_release(fd_lock);

    return 0;
}

static void oplus_reset_smem_fifo(void)
{
    if (smem_fifo) {
        SNS_ISLAND_EXIT();
        memset(smem_fifo, 0, sizeof(struct fb_event_smem));
    }
}

int oplus_smem_fifo_show(void)
{
    struct fb_event *s_fifo = (struct fb_event *)smem_fifo;
    int index = 0;

    sns_osa_lock_acquire(fd_lock);

    SNS_ISLAND_EXIT();
    for (index = 0; index < smem_fifo_len; index ++) {
        SNS_PRINTF(ERROR, sns_fw_printf, "smem_fifo_len %d ,fifo[%d]:event_id = %d, count = %d",
            smem_fifo_len, index, s_fifo->event_id, s_fifo->count);
        s_fifo++;
    }
    sns_osa_lock_release(fd_lock);

    return 0;
}

int oplus_get_smem_fifo_len(void)
{
    return smem_fifo_len;
}

int oplus_copy_to_smem_fifo(void)
{
    struct fb_event_node *pos = NULL;
    struct fb_event_node *n = NULL;
    struct fb_event *s_fifo = (struct fb_event *)smem_fifo;
    int count = 0;

    SNS_PRINTF(ERROR, sns_fw_printf, "oplus_copy_to_smem_fifo");

    sns_osa_lock_acquire(fd_lock);

    SNS_ISLAND_EXIT();

    oplus_reset_smem_fifo();
    smem_fifo_len = island_fifo_len;

    list_for_each_entry_safe(pos, n, struct fb_event_node, &(f_head.list), list) {
        memcpy(s_fifo, &pos->event, sizeof(struct fb_event));
        s_fifo++;
        count++;
        list_del(&(pos->list));
        sns_free(pos);
    }
    SNS_PRINTF(ERROR, sns_fw_printf, "fifo_len = %d, memcpy count = %d",
        smem_fifo_len, count);

    island_fifo_len = 0;
    has_main_heap_malloc = false;

    sns_osa_lock_release(fd_lock);

    return 0;
}
