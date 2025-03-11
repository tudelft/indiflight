#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/custom_transport.h"

#include "std_msgs/msg/string.h"
#include "nav_msgs/msg/odometry.h"
#include "geometry_msgs/msg/pose_stamped.h"
#include "geometry_msgs/msg/twist_stamped.h"

#include "platform.h"

#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "flight/ahrs.h"
#include "flight/ekf.h"
#include "cli/cli.h"

#include "stdbool.h"


#if defined(USE_UROS)

#define RCLCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ goto fail; }}

// clock_gettime, which is a dependency, see default_transport.cpp
#define micro_rollover_useconds 4294967295
int clock_gettime(clockid_t unused, struct timespec *tp) __attribute__ ((weak));
int clock_gettime(clockid_t unused, struct timespec *tp) {
    (void)unused;
    static uint32_t rollover = 0;
    static uint32_t last_measure = 0;

    uint32_t m = micros();
    rollover += (m < last_measure) ? 1 : 0;

    uint64_t real_us = (uint64_t) (m + rollover * micro_rollover_useconds);
    tp->tv_sec = real_us / 1000000;
    tp->tv_nsec = (real_us % 1000000) * 1000;
    last_measure = m;

    return 0;
}

// --- UROS TRANSPORT SETUP
#ifndef MOCKUP

static serialPort_t *port = NULL;
static const serialPortConfig_t *portConfig;

static bool custom_transport_open(struct uxrCustomTransport *transport) {
    UNUSED(transport);

    portConfig = findSerialPortConfig(FUNCTION_UROS);
    if (!portConfig) {
        return false;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_115200;
    }

    port = openSerialPort(portConfig->identifier,
        FUNCTION_UROS,
        NULL,
        NULL,
        baudRates[baudRateIndex],
        MODE_RXTX,
        SERIAL_NOT_INVERTED);

    if (!port) {
        return false;
    }

    return true;
}
static bool custom_transport_close(struct uxrCustomTransport *transport) {
    UNUSED(transport);
    if (!port) {
        return false;
    }

    closeSerialPort(port);
    port = NULL;

    return true;
}
static size_t custom_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    UNUSED(transport);
    UNUSED(err);
    serialWriteBuf(port, buf, len);
    return len; // ?
}
static size_t custom_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    UNUSED(transport);
    UNUSED(timeout);
    UNUSED(err);
    UNUSED(len);
    size_t i = 0;
    while (serialRxBytesWaiting(port) && i < len) {
        buf[i++] = serialRead(port);
    }
    return i; // ?
}
#endif // MOCKUP

// --- UROS config
// objects
static bool urosIsInitialized = false;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;

// topics / messages
static rcl_publisher_t pub_odom;
static rcl_subscription_t sub_pose;
static rcl_subscription_t sub_twist;
static geometry_msgs__msg__PoseStamped sub_msg_pose;
static geometry_msgs__msg__TwistStamped sub_msg_twist;
static geometry_msgs__msg__PoseStamped pub_msg_pose;

// Callback function for subscriber
static void sub_cb_pose(const void *msgin) {
    const geometry_msgs__msg__PoseStamped *msg = (const geometry_msgs__msg__PoseStamped *)msgin;
    UNUSED(msg);
    // cliPrintLinef("Received Pose: %s\n", msg->header.stamp);
}
static void sub_cb_twist(const void *msgin) {
    const geometry_msgs__msg__TwistStamped *msg = (const geometry_msgs__msg__TwistStamped *)msgin;
    UNUSED(msg);
    // cliPrintLinef("Received Twist: %s\n", msg->header.stamp);
}

// Custom transport implementation

void urosInit(void) {
    urosIsInitialized = false;

#ifndef MOCKUP
    // 7️⃣ Attach custom serial transport
    RCLCHECK( rmw_uros_set_custom_transport(
        true,
        (void *)NULL,
        custom_transport_open,
        custom_transport_close,
        custom_transport_write,
        custom_transport_read
    ));
#endif

    // 1️⃣ Initialize allocator with static memory (no malloc)
    allocator = rcl_get_default_allocator();

    // 2️⃣ Initialize micro-ROS support (static memory)
    RCLCHECK( rclc_support_init(&support, 0, NULL, &allocator) );

    // 3️⃣ Create the ROS node
    RCLCHECK( rclc_node_init_default(&node, "indiflight", "", &support) );

    // 4️⃣ Create a publisher
    RCLCHECK( rclc_publisher_init_default(
        &pub_odom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "/indiflight/pose"
    ));

    // 5️⃣ Create a subscriber
    RCLCHECK( rclc_subscription_init_best_effort(
        &sub_pose,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "/mocap/pose"
    ));
    RCLCHECK( rclc_subscription_init_best_effort(
        &sub_twist,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "/mocap/twist"
    ));

    // 6️⃣ Set up the executor (for handling callbacks)
    RCLCHECK( rclc_executor_init(&executor, &support.context, 2, &allocator) );
    RCLCHECK( rclc_executor_add_subscription(&executor, &sub_pose, &sub_msg_pose, &sub_cb_pose, ON_NEW_DATA) );
    RCLCHECK( rclc_executor_add_subscription(&executor, &sub_twist, &sub_msg_twist, &sub_cb_twist, ON_NEW_DATA) );

    urosIsInitialized = true;

    return;

fail:
    {
        rcl_error_state_t err;
        err = *rcutils_get_error_state();
        UNUSED(err);
        return;
    }
}

void urosUpdate(timeUs_t currentTimeUs) {
    if (!urosIsInitialized) {
        goto fail;
    }

    // 8️⃣ publish some messages
    // Set message
    pub_msg_pose.header.stamp.sec = currentTimeUs / 1000000;
    pub_msg_pose.header.stamp.nanosec = 1000 * (currentTimeUs % 1000000);
    pub_msg_pose.header.frame_id.data = (char*) "map";
    pub_msg_pose.header.frame_id.size = strlen(pub_msg_pose.header.frame_id.data);

    fp_quaternion_t q;
    getHoverAttitudeQuaternion(&q);
    pub_msg_pose.pose.position.x = posEstNed.V.X;
    pub_msg_pose.pose.position.y = posEstNed.V.Y;
    pub_msg_pose.pose.position.z = posEstNed.V.Z;
    pub_msg_pose.pose.orientation.w = q.w;
    pub_msg_pose.pose.orientation.x = q.x;
    pub_msg_pose.pose.orientation.y = q.y;
    pub_msg_pose.pose.orientation.z = q.z;

    // Publish message
    RCLCHECK( rcl_publish(&pub_odom, &pub_msg_pose, NULL) );

    // Spin executor (handles subscriptions)
    RCLCHECK( rclc_executor_spin_some(&executor, RCL_US_TO_NS(10)) );

fail:
    {
        rcl_error_state_t err;
        err = *rcutils_get_error_state();
        UNUSED(err);
        return;
    }
}

//void urosClose(void) {
//    // Cleanup (never reaches here)
//    RCLCHECK( rclc_executor_fini(&executor) );
//    RCLCHECK( rcl_publisher_fini(&pub_odom, &node) );
//    RCLCHECK( rcl_subscription_fini(&sub_cb_pose, &node) );
//    RCLCHECK( rcl_subscription_fini(&sub_cb_twist, &node) );
//    RCLCHECK( rcl_node_fini(&node) );
//    RCLCHECK( rclc_support_fini(&support) );
//
//fail:
//    return;
//}

#endif