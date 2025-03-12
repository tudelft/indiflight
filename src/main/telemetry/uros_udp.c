#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "common/time.h"
#include "uros.h"

#if defined(USE_TELEMETRY_UROS) && defined(MOCKUP)

#define STRING_BUFFER_LEN 250

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t ping_publisher;
//static rcl_publisher_t pong_publisher;
//static rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

//std_msgs__msg__Header incoming_ping;
std_msgs__msg__Header outcoming_ping;
std_msgs__msg__Header incoming_pong;


// char incoming_ping_buffer[STRING_BUFFER_LEN];
// incoming_ping.frame_id.data = incoming_ping_buffer;
// incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;



int device_id;
int seq_no;
int pong_count;

void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;

	if (timer != NULL) {

		seq_no = rand();
		sprintf(outcoming_ping.frame_id.data, "%d_%d", seq_no, device_id);
		outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);
		
		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_ping.stamp.sec = ts.tv_sec;
		outcoming_ping.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the ping message
		pong_count = 0;
		RCSOFTCHECK(rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL));
		printf("Ping send seq %s\n", outcoming_ping.frame_id.data);
	}
}

void pong_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

    if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) == 0) {
        pong_count++;
        printf("Pong for seq %s (%d)\n", msg->frame_id.data, pong_count);
    }
}

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rcl_timer_t timer;
void urosInit(void)
{

	RCSOFTCHECK(rcl_publisher_fini(&ping_publisher, &node));
	RCSOFTCHECK(rcl_subscription_fini(&pong_subscriber, &node));
	RCSOFTCHECK(rcl_node_fini(&node));
    rclc_support_fini(&support);
    RCSOFTCHECK(rcl_shutdown(&support.context));

	allocator = rcl_get_default_allocator();

    if (!rcl_context_is_valid(&support.context)) {
        printf("CONTEXT IS INVALID\n");
    }

	// create init_options
	RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    //printf("RCL error: %s\n", rcl_get_error_string().str);
    //rcl_reset_error();

	// create node
	RCSOFTCHECK(rclc_node_init_default(&node, "indiflight", "", &support));

	RCSOFTCHECK(rclc_publisher_init_best_effort(
        &ping_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        "/microROS/ping"
    ));

	// Create a best effort  pong subscriber
	RCSOFTCHECK(rclc_subscription_init_best_effort(
        &pong_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        "/microROS/pong"
    ));

    // create a timer
	RCSOFTCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), ping_timer_callback));

	// Create executor
	executor = rclc_executor_get_zero_initialized_executor();
	RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));

	RCSOFTCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong, &pong_subscription_callback, ON_NEW_DATA));
}

void urosUpdate(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
	// Create and allocate the pingpong messages
    char outcoming_ping_buffer[STRING_BUFFER_LEN];
    char incoming_pong_buffer[STRING_BUFFER_LEN];
    outcoming_ping.frame_id.data = outcoming_ping_buffer;
    outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;
    incoming_pong.frame_id.data = incoming_pong_buffer;
    incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

    if (!rcl_context_is_valid(&support.context)) {
        printf("CONTEXT IS INVALID\n");
    }

	RCSOFTCHECK(rclc_executor_spin_some(&executor, 1000));
}

// static void urosClose(void) {
	// RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
	// RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
	// RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
	// RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
	// RCCHECK(rcl_node_fini(&node));
// }

#endif