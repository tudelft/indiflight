/* 
  This code belongs to the T4 Actuators Board solution. 
  Used for ESC and Servo telemetry and error codes.
 */

#include <stdint.h>

#pragma once

#define START_BYTE_ACTUATORS_T4 0x9A

struct __attribute__((__packed__)) ActuatorsT4Out {

  // ESC RPM
  int16_t esc_1_rpm;
  int16_t esc_2_rpm;
  int16_t esc_3_rpm;
  int16_t esc_4_rpm;

  // ESC error code
  int16_t esc_1_error_code; 
  int16_t esc_2_error_code; 
  int16_t esc_3_error_code; 
  int16_t esc_4_error_code;

  // ESC current in mA
  int16_t esc_1_current;
  int16_t esc_2_current;
  int16_t esc_3_current;
  int16_t esc_4_current;

  // ESC voltage in mV     
  int16_t esc_1_voltage;
  int16_t esc_2_voltage;
  int16_t esc_3_voltage;
  int16_t esc_4_voltage;

  // Servos telemetry & update rate

  // Servo angle in Degrees x 100
  int16_t servo_1_angle;
  int16_t servo_2_angle; 
  int16_t servo_3_angle; 
  int16_t servo_4_angle; 
  int16_t servo_5_angle; 
  int16_t servo_6_angle; 
  int16_t servo_7_angle; 
  int16_t servo_8_angle; 
  int16_t servo_9_angle; 
  int16_t servo_10_angle;

  // With PWM servos, the calculated, not measured angle
  int16_t servo_11_angle;
  int16_t servo_12_angle;

  // Servo load in ?    
  int16_t servo_1_load;
  int16_t servo_2_load;
  int16_t servo_3_load;
  int16_t servo_4_load;
  int16_t servo_5_load;
  int16_t servo_6_load;
  int16_t servo_7_load;
  int16_t servo_8_load;
  int16_t servo_9_load;
  int16_t servo_10_load;
  // Note that PWM actuators 11 and 12 have no real load feedback therefore commented out here
  //int16_t servo_11_load;
  //int16_t servo_12_load;

  uint16_t bitmask_servo_health; //Bitmask of servo health status

  // Rolling message in, where in is out
  float rolling_msg_out;
  uint8_t rolling_msg_out_id;

  // CHECKSUM
  uint8_t checksum_out;
};

struct __attribute__((__packed__)) ActuatorsT4In {

  // ESC and Servo commands and arming settings

  // Arming of ESC and Servo
  uint8_t esc_arm; // Arm motor boolean bitmask
  uint16_t servo_arm; // Arm servo boolean bitmask

  // ESC command per default defines it is 0 - 1999
  int16_t esc_1_dshot_cmd;
  int16_t esc_2_dshot_cmd;
  int16_t esc_3_dshot_cmd;
  int16_t esc_4_dshot_cmd;

  // Servo command per default define it is in Degrees x 100 
  int16_t servo_1_cmd; 
  int16_t servo_2_cmd; 
  int16_t servo_3_cmd; 
  int16_t servo_4_cmd; 
  int16_t servo_5_cmd; 
  int16_t servo_6_cmd; 
  int16_t servo_7_cmd; 
  int16_t servo_8_cmd; 
  int16_t servo_9_cmd; 
  int16_t servo_10_cmd;   
  int16_t servo_11_cmd;   
  int16_t servo_12_cmd;

  // Rolling message out, where out is in
  float rolling_msg_in;
  uint8_t rolling_msg_in_id;

  // CHECKSUM
  uint8_t checksum_in;
};

struct __attribute__((__packed__)) ActuatorsT4Local {
  
  // ESC telemetry and error codes

  // ESC RPM
  int16_t esc_1_rpm;
  int16_t esc_2_rpm;
  int16_t esc_3_rpm;
  int16_t esc_4_rpm;
  // ESC error code 
  int16_t esc_1_error_code; 
  int16_t esc_2_error_code; 
  int16_t esc_3_error_code; 
  int16_t esc_4_error_code;
  // ESC current in mA
  int16_t esc_1_current;
  int16_t esc_2_current;
  int16_t esc_3_current;
  int16_t esc_4_current;
  // ESC voltage in mV
  int16_t esc_1_voltage;
  int16_t esc_2_voltage;
  int16_t esc_3_voltage;
  int16_t esc_4_voltage;
  // ESC consumed mAh 
  int16_t esc_1_consumed_mAh;
  int16_t esc_2_consumed_mAh;
  int16_t esc_3_consumed_mAh;
  int16_t esc_4_consumed_mAh;
  // Servo telemetry and update rate 
  // Servo angle Degrees * 100
  int16_t servo_1_angle; 
  int16_t servo_2_angle; 
  int16_t servo_3_angle; 
  int16_t servo_4_angle; 
  int16_t servo_5_angle; 
  int16_t servo_6_angle; 
  int16_t servo_7_angle; 
  int16_t servo_8_angle; 
  int16_t servo_9_angle; 
  int16_t servo_10_angle; 
  int16_t servo_11_angle; 
  int16_t servo_12_angle;
  // Servo speed in ?
  int16_t servo_1_speed; 
  int16_t servo_2_speed; 
  int16_t servo_3_speed; 
  int16_t servo_4_speed; 
  int16_t servo_5_speed;
  int16_t servo_6_speed; 
  int16_t servo_7_speed;
  int16_t servo_8_speed; 
  int16_t servo_9_speed;
  int16_t servo_10_speed;
  // Servo load in ?
  int16_t servo_1_load;
  int16_t servo_2_load;
  int16_t servo_3_load;
  int16_t servo_4_load;
  int16_t servo_5_load;
  int16_t servo_6_load;
  int16_t servo_7_load;
  int16_t servo_8_load;
  int16_t servo_9_load;
  int16_t servo_10_load;
  // Servo voltage in mV
  int16_t servo_1_volt; 
  int16_t servo_2_volt;
  int16_t servo_3_volt;
  int16_t servo_4_volt;
  int16_t servo_5_volt;
  int16_t servo_6_volt;
  int16_t servo_7_volt;
  int16_t servo_8_volt;
  int16_t servo_9_volt;
  int16_t servo_10_volt;
  // Servo temperature in Celcius
  int16_t servo_1_temp;
  int16_t servo_2_temp;
  int16_t servo_3_temp;
  int16_t servo_4_temp;
  int16_t servo_5_temp;
  int16_t servo_6_temp;
  int16_t servo_7_temp;
  int16_t servo_8_temp;
  int16_t servo_9_temp;
  int16_t servo_10_temp;
  // Servo feedback update time in Microseconds
  int16_t servo_1_feedback_update_time_us;
  int16_t servo_2_feedback_update_time_us;
  int16_t servo_3_feedback_update_time_us;
  int16_t servo_4_feedback_update_time_us;
  int16_t servo_5_feedback_update_time_us;
  int16_t servo_6_feedback_update_time_us;
  int16_t servo_7_feedback_update_time_us;
  int16_t servo_8_feedback_update_time_us;
  int16_t servo_9_feedback_update_time_us;
  int16_t servo_10_feedback_update_time_us;
};

// Instruction set
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_WRITE 0x83

// Memory Address
//-------EPROM(Read only)--------
#define SBS_VERSION_L 3
#define SBS_VERSION_H 4

//-------EPROM(Read And write)--------
#define SBS_ID 5
#define SBS_BAUD_RATE 6
#define SBS_MIN_ANGLE_LIMIT_L 9
#define SBS_MIN_ANGLE_LIMIT_H 10
#define SBS_MAX_ANGLE_LIMIT_L 11
#define SBS_MAX_ANGLE_LIMIT_H 12
#define SBS_CW_DEAD 26
#define SBS_CCW_DEAD 27

//-------SRAM(Read & Write)--------
#define SBS_TORQUE_ENABLE 40
#define SBS_GOAL_POSITION_L 42
#define SBS_GOAL_POSITION_H 43
#define SBS_GOAL_TIME_L 44
#define SBS_GOAL_TIME_H 45
#define SBS_GOAL_SPEED_L 46
#define SBS_GOAL_SPEED_H 47
#define SBS_LOCK 48

//-------SRAM(Read Only)--------
#define SBS_PRESENT_POSITION_L 56
#define SBS_PRESENT_POSITION_H 57
#define SBS_PRESENT_SPEED_L 58
#define SBS_PRESENT_SPEED_H 59
#define SBS_PRESENT_LOAD_L 60
#define SBS_PRESENT_LOAD_H 61
#define SBS_PRESENT_VOLTAGE 62
#define SBS_PRESENT_TEMPERATURE 63
#define SBS_MOVING 66
#define SBS_PRESENT_CURRENT_L 69
#define SBS_PRESENT_CURRENT_H 70

// END Definitions_SxS.h
