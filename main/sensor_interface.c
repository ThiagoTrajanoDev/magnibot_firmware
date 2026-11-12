#include "sensor_interface.h"
#include <string.h>

static bool valid_driver(const sensor_driver_t *driver) {
    if (!driver || driver->source == SENSOR_NONE)
        return !driver || (!driver->request && !driver->poll && !driver->cancel);
    return (driver->source == SENSOR_TEST || driver->source == SENSOR_HARDWARE) &&
        driver->request && driver->poll && driver->cancel;
}

bool sensor_interface_init(sensor_interface_t *interface, sensor_data_t *data,
                           const sensor_driver_t *distance, const sensor_driver_t *current,
                           uint32_t period_ms, uint32_t timeout_ms, uint32_t max_age_ms) {
    /* An initialised interface (possibly stopped/latched) is never silently reset. */
    if (!interface || interface->initialized) return false;
    memset(interface, 0, sizeof(*interface));
    interface->stopped = true;
    if (!data || !period_ms || !timeout_ms || !max_age_ms ||
        period_ms > 60000 || timeout_ms > 60000 || max_age_ms > 60000 ||
        !valid_driver(distance) || !valid_driver(current)) return false;
    sensor_data_init(data);
    interface->distance.sample = &data->distance_mm;
    interface->current.sample = &data->current_ma;
    if (distance) interface->distance.driver = *distance;
    if (current) interface->current.driver = *current;
    interface->distance.last_result = interface->current.last_result = IO_DISABLED;
    interface->period_ms = period_ms;
    interface->timeout_ms = timeout_ms;
    interface->max_age_ms = max_age_ms;
    interface->initialized = true;
    interface->stopped = false;
    return true;
}

static void invalidate(sensor_channel_t *channel, hardware_result_t reason) {
    channel->last_result = reason;
    if (reason != IO_LATCHED && channel->failures != UINT32_MAX) ++channel->failures;
    if (channel->sample) channel->sample->valid = false;
}

static void stop_channel(sensor_channel_t *channel, hardware_result_t reason) {
    if (channel->pending && channel->driver.cancel)
        channel->driver.cancel(channel->driver.context);
    channel->pending = false;
    invalidate(channel, reason);
}

void sensor_interface_stop(sensor_interface_t *interface) {
    if (!interface || !interface->initialized || interface->stopped) return;
    interface->stopped = true;
    stop_channel(&interface->distance, IO_LATCHED);
    stop_channel(&interface->current, IO_LATCHED);
}

static void poll_channel(sensor_interface_t *interface, sensor_channel_t *channel, int64_t now_ms) {
    if (channel->driver.source == SENSOR_NONE) return;
    if (!channel->pending) {
        if (channel->requested_once && now_ms - channel->requested_ms < interface->period_ms) return;
        channel->requested_once = true;
        channel->requested_ms = now_ms;
        hardware_result_t result = channel->driver.request(channel->driver.context, now_ms);
        if (result != IO_OK) {
            /* A rejected request may still have touched a peripheral. */
            channel->driver.cancel(channel->driver.context);
            invalidate(channel, result == IO_PENDING ? IO_INVALID : result);
            return;
        }
        channel->pending = true;
        channel->last_result = IO_PENDING;
        return;
    }
    if (now_ms - channel->requested_ms >= interface->timeout_ms) {
        stop_channel(channel, IO_TIMEOUT);
        return;
    }
    float value = 0;
    int64_t timestamp = -1;
    hardware_result_t result = channel->driver.poll(channel->driver.context, &value, &timestamp);
    if (result == IO_PENDING) return;
    channel->pending = false;
    if (result != IO_OK) {
        channel->driver.cancel(channel->driver.context);
        invalidate(channel, result);
        return;
    }
    /* Reject bad time before advancing the accepted timestamp watermark. */
    if (timestamp < 0 || timestamp > now_ms || now_ms - timestamp > interface->max_age_ms) {
        invalidate(channel, IO_INVALID);
        return;
    }
    bool valid = sensor_sample_update(channel->sample, value, timestamp, channel->driver.source, true);
    if (!valid) {
        channel->sample->valid = false;
        channel->last_result = IO_INVALID;
        if (channel->failures != UINT32_MAX) ++channel->failures;
        return;
    }
    channel->last_result = IO_OK;
}

void sensor_interface_poll(sensor_interface_t *interface, int64_t now_ms) {
    if (!interface || !interface->initialized || interface->stopped) return;
    if (now_ms < interface->last_ms || now_ms < 0) {
        interface->stopped = true;
        stop_channel(&interface->distance, IO_CLOCK);
        stop_channel(&interface->current, IO_CLOCK);
        return;
    }
    interface->last_ms = now_ms;
    poll_channel(interface, &interface->distance, now_ms);
    poll_channel(interface, &interface->current, now_ms);
}
