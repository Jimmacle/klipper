// Support for gathering acceleration data from ADXL345 chip
//
// Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer

struct adxl345 {
    struct timer timer;
    uint32_t rest_ticks, stop_time;
    struct spidev_s *spi;
    uint16_t sequence;
    uint8_t flags, data_count;
    uint8_t data[48];
};

enum {
    AX_HAVE_START = 1<<0, AX_RUNNING = 1<<1, AX_PENDING = 1<<2,
};

static struct task_wake adxl345_wake;

static uint_fast8_t
adxl345_event(struct timer *timer)
{
    struct adxl345 *ax = container_of(timer, struct adxl345, timer);
    ax->flags |= AX_PENDING;
    sched_wake_task(&adxl345_wake);
    return SF_DONE;
}

void
command_config_adxl345(uint32_t *args)
{
    struct adxl345 *ax = oid_alloc(args[0], command_config_adxl345
                                   , sizeof(*ax));
    ax->timer.func = adxl345_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_adxl345, "config_adxl345 oid=%c spi_oid=%c");

// Report local measurement buffer
static void
adxl_report(struct adxl345 *ax, uint8_t oid)
{
    sendf("adxl345_data oid=%c sequence=%hu data=%*s"
          , oid, ax->sequence, ax->data_count, ax->data);
    ax->data_count = 0;
    ax->sequence++;
}

// Chip registers
#define AR_DATAX0   0x32
#define AR_FIFO_CTL 0x38
#define AM_READ  0x80
#define AM_MULTI 0x40

// Startup measurements
static void
adxl_start(struct adxl345 *ax, uint8_t oid)
{
    sched_del_timer(&ax->timer);
    ax->flags = AX_RUNNING;
    uint8_t msg[2] = { AR_FIFO_CTL, 0x80 };
    uint32_t start_time = timer_read_time();
    spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    irq_disable();
    uint32_t end_time = timer_read_time();
    ax->timer.waketime = end_time + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
    sendf("adxl345_start oid=%c start_time=%u end_time=%u"
          , oid, start_time, end_time);
}

// End measurements
static void
adxl_stop(struct adxl345 *ax, uint8_t oid)
{
    sched_del_timer(&ax->timer);
    ax->flags = 0;
    uint8_t msg[2] = { AR_FIFO_CTL, 0x00 };
    spidev_transfer(ax->spi, 0, sizeof(msg), msg);
    if (ax->data_count)
        adxl_report(ax, oid);
    sendf("adxl345_end oid=%c sequence=%hu", oid, ax->sequence);
}

// Query accelerometer data
static void
adxl_query(struct adxl345 *ax, uint8_t oid)
{
    uint8_t msg[9] = { AR_DATAX0 | AM_READ | AM_MULTI, 0, 0, 0, 0, 0, 0, 0, 0 };
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    memcpy(&ax->data[ax->data_count], &msg[1], 6);
    ax->data_count += 6;
    if (ax->data_count + 6 >= ARRAY_SIZE(ax->data))
        adxl_report(ax, oid);
    if ((msg[8] & 0x3f) > 1) {
        // More data in fifo - wake this task again
        sched_wake_task(&adxl345_wake);
    } else {
        // Sleep until next check time
        sched_del_timer(&ax->timer);
        ax->flags &= ~AX_PENDING;
        irq_disable();
        ax->timer.waketime = timer_read_time() + ax->rest_ticks;
        sched_add_timer(&ax->timer);
        irq_enable();
    }
}

void
command_query_adxl345(uint32_t *args)
{
    struct adxl345 *ax = oid_lookup(args[0], command_config_adxl345);

    if (!args[2]) {
        // End measurements
        adxl_stop(ax, args[0]);
        return;
    }
    // Start new measurements query
    sched_del_timer(&ax->timer);
    ax->timer.waketime = args[1];
    ax->rest_ticks = args[2];
    ax->flags = AX_HAVE_START;
    ax->sequence = 0;
    ax->data_count = 0;
    sched_add_timer(&ax->timer);
}
DECL_COMMAND(command_query_adxl345,
             "query_adxl345 oid=%c clock=%u rest_ticks=%u");

void
adxl345_task(void)
{
    if (!sched_check_wake(&adxl345_wake))
        return;
    uint8_t oid;
    struct adxl345 *ax;
    foreach_oid(oid, ax, command_config_adxl345) {
        uint_fast8_t flags = ax->flags;
        if (!(flags & AX_PENDING))
            continue;
        if (flags & AX_HAVE_START)
            adxl_start(ax, oid);
        else
            adxl_query(ax, oid);
    }
}
DECL_TASK(adxl345_task);
