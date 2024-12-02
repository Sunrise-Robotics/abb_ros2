
#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER hardware_interface

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "./abb_hardware_interface/hardware_interface_tp.h"

#if !defined(HARDWARE_INTERFACE_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define HARDWARE_INTERFACE_TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    hardware_interface,
    read_start,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    hardware_interface,
    read_end,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    hardware_interface,
    write_start,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    hardware_interface,
    write_end,
    TP_ARGS(),
    TP_FIELDS()
)


#endif /* HARDWARE_INTERFACE_TP_H */

#include <lttng/tracepoint-event.h>
