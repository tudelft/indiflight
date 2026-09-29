/*
 * Feetech STS3032 Serial Bus Servo Driver for Flight Controller (Indiflight)
 * Full-duplex UART testing (1,000,000 baud).
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_ACTUATORS_T4)

#ifndef USE_SERVOS
#error "USE_ACTUATORS_T4 requires USE_SERVOS"
#endif

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/serial.h"
#include "io/serial.h"

#include "flight/mixer.h"
#include "flight/servos.h"

#include "t4.h"
#include "io/t4_protocol.h"
//#include "Definitions_SxS.h"

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



typedef uint8_t byte;

// =============================================================================
// HARDWARE PARAMETERS & BUS TIMING 
// =============================================================================
#define BAUDRATE_SERVO                  1000000 // 1.0 MBaud physical bus standard
#define SERVO_MAX_COMD_DEFAULT          4095.0f // STS3032: 12-bit max step limit
#define DEFAULT_STEPS_FOR_FULL_ROTATION 4095.0f // STS3032: Steps per 360 degrees
#define FULLROTATION                    360.0f  // Degrees per full circle
#define DEFAULT_MULTIPLIER_DEGREES      100.0f  // Fixed-point scaling factor (1 deg = 100 cdeg)

#define Servo_1_ID                      1
#define Servo_2_ID                      2

// Timing guards
#define COMM_DISPATCH_INTERVAL_US       1000    // 1 ms time-slice (250 Hz cycle per servo)
#define RX_FRAME_TIMEOUT_US             2000    // 2 ms timeout to clear incomplete/corrupted frames
#define RX_BUFFER_SIZE                  50

static serialPort_t *t4Port = NULL; // *t4Port is a pointer to the serial port used for communication with the STS3032 servos. It is initialized to NULL and will be set to the appropriate serial port during configuration.
static const serialPortConfig_t *portConfig;





// =============================================================================
// HELPER FUNCTIONS 
// =============================================================================

/* 1 16-bit split into 2 8 digits (Teensy implementation) */
static void SplitByte(uint8_t* DataL, uint8_t* DataH, uint16_t Data) {
    *DataH = (Data >> 8);
    *DataL = (Data & 0xff);
}

// 2 8-digit combinations for 1 16-digit number (Teensy implementation) 
static uint16_t CompactBytes(uint8_t DataL, uint8_t DataH) {
    uint16_t Data;
    Data = DataL;
    Data <<= 8;
    Data |= DataH;
    return Data; 
}

/**
 * 
 * Builds: [0xFF] [0xFF] [ID] [Length] [Instruction] [Params...] [Checksum]
 */
static void SendInstruction(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount) {
    if (!t4Port) {
        return;
    }

    /* -------- SEND INSTRUCTION ------------ */
    int buffer_tx_idx = 0;
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;

    for (int i = 0; i < s32_ParamCount; i++) {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i = 2; i < buffer_tx_idx; i++) {
        u8_Checksum += buffer_tx[i];
    }

    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    /* Send out to FC UART TX buffer */
    for (int i = 0; i < buffer_tx_idx; i++) {
        serialWrite(t4Port, buffer_tx[i]);
    }
}

// Duing actual integration, we need to check the code line 1279 to 1287  in teensy code

// =============================================================================
// HARDWARE PORT 
// =============================================================================

void freeActuatorsT4Port(void)
{
    closeSerialPort(t4Port);
    t4Port = NULL;
}

void initActuatorsT4(void)
{
    portConfig = findSerialPortConfig(FUNCTION_ACTUATORS_T4); 
}

void configureActuatorsT4Port(void)
{
    if (!portConfig) {
        return;
    }

    t4Port = openSerialPort(
        portConfig->identifier,
        FUNCTION_ACTUATORS_T4,
        NULL,
        NULL,
        BAUDRATE_SERVO,
        MODE_RXTX,
        SERIAL_NOT_INVERTED
    );

    if (!t4Port) {
        return;
    }
}

// =============================================================================
// TX 
// =============================================================================

void sendActuatorsT4(void)
{
    if (!t4Port) {
        return;
    }
    static struct ActuatorsT4In  in; 
    static uint32_t lastDispatchTimeUs = 0;
    uint32_t currentTimeUs = micros();

    // Throttle transmission to 1 ms intervals to avoid saturating UART TX buffer
    if (currentTimeUs - lastDispatchTimeUs < COMM_DISPATCH_INTERVAL_US) // If the time since the last dispatch is less than the defined interval, we return early to avoid sending data too frequently
    {
        return;
    }
    lastDispatchTimeUs = currentTimeUs;

    // 1. Pack the ActuatorsT4In struct from normalized flight mixer outputs
    in.esc_arm = 0x00; // disarm all escs
    in.servo_arm = 0xFFFF; // Arm all servos
    in.servo_1_cmd = (int16_t)(servo_normalized[0] * DEFAULT_MULTIPLIER_DEGREES * DEFAULT_MULTIPLIER_DEGREES);
    in.servo_2_cmd = (int16_t)(servo_normalized[1] * DEFAULT_MULTIPLIER_DEGREES * DEFAULT_MULTIPLIER_DEGREES);

    static uint8_t step = 0;
    byte u8_Data_1[7] = { 0 };
    byte u8_Data_2[2] = { 0 };

    switch (step) {
        case 0:
        {
            /* --- SERVO 1 WRITE --- */
            
            int Target_position_servo_1 = (int)(constrain(
                (DEFAULT_STEPS_FOR_FULL_ROTATION / FULLROTATION) * (in.servo_1_cmd / DEFAULT_MULTIPLIER_DEGREES) + (SERVO_MAX_COMD_DEFAULT / 2.0f),
                0,
                SERVO_MAX_COMD_DEFAULT
            ));

            u8_Data_1[0] = SBS_GOAL_POSITION_L;                               // Reg 42 (0x2A)
            SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_1); // Reg 42 (Low Byte), Reg 43 (High Byte)
            SplitByte(&u8_Data_1[3], &u8_Data_1[4], 0);                       // Reg 44 & 45 (Transit Time = 0)
            SplitByte(&u8_Data_1[5], &u8_Data_1[6], 0);                       // Reg 46 & 47 (Speed Limit = 0)
            SendInstruction(Servo_1_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));

            step = 1;
            break;
        }

        case 1:
        {
            /* --- SERVO 1 READ TELEMETRY --- */
            u8_Data_2[0] = SBS_PRESENT_POSITION_L; // Reg 56 (0x38)
            u8_Data_2[1] = 8;                      // Request 8 registers
            SendInstruction(Servo_1_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));

            step = 2;
            break;
        }

        case 2:
        {
            /* --- SERVO 2 WRITE --- */
           
            int Target_position_servo_2 = (int)(constrain(
                (DEFAULT_STEPS_FOR_FULL_ROTATION / FULLROTATION) * (in.servo_2_cmd / DEFAULT_MULTIPLIER_DEGREES) + (SERVO_MAX_COMD_DEFAULT / 2.0f),
                0,
                SERVO_MAX_COMD_DEFAULT
            ));

            u8_Data_1[0] = SBS_GOAL_POSITION_L;                               // Reg 42 (0x2A)
            SplitByte(&u8_Data_1[1], &u8_Data_1[2], Target_position_servo_2); // Reg 42 (Low Byte), Reg 43 (High Byte)
            SplitByte(&u8_Data_1[3], &u8_Data_1[4], 0);                       // Reg 44 & 45 (Transit Time = 0)
            SplitByte(&u8_Data_1[5], &u8_Data_1[6], 0);                       // Reg 46 & 47 (Speed Limit = 0)
            SendInstruction(Servo_2_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1));

            step = 3;
            break;
        }

        case 3:
        {
            /* --- SERVO 2 READ TELEMETRY --- */
            u8_Data_2[0] = SBS_PRESENT_POSITION_L; // Reg 56 (0x38)
            u8_Data_2[1] = 8;                      // Request 8 registers
            SendInstruction(Servo_2_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));

            step = 0;
            break;
        }

        default:
            step = 0;
            break;
    }
}

struct ActuatorsT4Out t4_out ={0}; 

void handleActuatorsT4(void)
{
    
    if (!t4Port) {
        return;
    }
    

    static byte buffer_servo[RX_BUFFER_SIZE];
    static int buffer_servo_idx = 0;
    static uint32_t lastByteTimeUs = 0;

    uint32_t currentTimeUs = micros(); // micros() returns the number of microseconds since the program started running (absolute time). This is used to track the time since the last byte was received.

    // Reset buffer index if incoming frame stalls
    if (buffer_servo_idx > 0 && (currentTimeUs - lastByteTimeUs > RX_FRAME_TIMEOUT_US)) {
        buffer_servo_idx = 0;
    }

    while (serialRxBytesWaiting(t4Port)) // While there are bytes waiting in the serial RX buffer
     {
        byte byte_in = serialRead(t4Port); // Read only one byte from the serial port and when the next while loop iteration occurs, the next byte will be read. This is done to avoid reading all bytes at once and potentially missing some bytes if the buffer is not large enough.
        lastByteTimeUs = micros(); // Updates the last byte received time

        // 1. Detect 0xFF 0xFF synchronization preamble
        if (buffer_servo_idx == 0) {
            if (byte_in == 0xFF) {
                buffer_servo[buffer_servo_idx++] = byte_in;// Writes first byte into buffer_servo[0] 
            }
            continue; // If the first byte is not 0xFF, ignore it and start while loop again to wait for the next byte
        } 
        else if (buffer_servo_idx == 1) 
        {
            if (byte_in == 0xFF) {
                buffer_servo[buffer_servo_idx++] = byte_in; //  Writes second byte into buffer_servo[1]
            } else {
                buffer_servo_idx = 0;
            }
            continue; //
        }
        else if (buffer_servo_idx == 2 && byte_in == 0xFF)
         {
            continue; // Ignore any additional 0xFF bytes after the first two, as they are not part of the protocol
        }

        // 2. Buffer incoming bytes
        if (buffer_servo_idx < RX_BUFFER_SIZE) // Prevent buffer overflow
         {
            buffer_servo[buffer_servo_idx++] = byte_in;
        } else {
            buffer_servo_idx = 0;
            continue; // Reset buffer if overflow occurs
        }

        // 3. Dynamic Length Validation 
            // buffer_Servo[2]  is the ID field in the protocol, which indicates which servo is responding.
            // buffer_servo[3] is the Length field in the protocol, which indicates how many bytes of parameters follow. The total expected bytes for a complete frame is 4 (headers + ID + Length) + Length field value. If the buffer index has reached or exceeded this total, we can validate the frame.

        if (buffer_servo_idx >= 4) 
        {
            int total_expected_bytes = buffer_servo[3] + 4; // Headers(2) + ID(1) + Length(1) + LengthField

            if (buffer_servo_idx >= total_expected_bytes) // If we have received enough bytes for a complete frame, we can validate the checksum and process the data
            {
                // Compute checksum: bitwise NOT of sum from ID through parameter payload
                uint8_t bitsum_servo = 0;
                for (int i = 2; i < total_expected_bytes - 1; i++) // Sum from ID through parameter payload and ignore checksum byte
                {
                    bitsum_servo += buffer_servo[i]; 
                }

                uint8_t expected_checksum = (uint8_t)(~bitsum_servo);
                if (expected_checksum == (uint8_t)buffer_servo[total_expected_bytes - 1]) // If the computed checksum matches the received checksum, we can process the data
                {
                    byte servo_id = buffer_servo[2];

                    // 14-byte telemetry response packet (buffer_servo[3] == 10)
                    if  (buffer_servo[3] >= 4) //
                    {
                        // Reconstruct raw angle into centidegrees
                        int16_t servo_angle = (int16_t)(
                            ((float)CompactBytes(buffer_servo[6], buffer_servo[5]) - (SERVO_MAX_COMD_DEFAULT / 2.0f)) *
                            (FULLROTATION * DEFAULT_MULTIPLIER_DEGREES) /
                            DEFAULT_STEPS_FOR_FULL_ROTATION
                        );

                        // Store in the ActuatorsT4Out struct, then assign to Indiflight array
                        if (servo_id == Servo_1_ID) {
                            t4_out.servo_1_angle = servo_angle;
                            servo_feedback[0] = t4_out.servo_1_angle;
                        } else if (servo_id == Servo_2_ID) {
                            t4_out.servo_2_angle = servo_angle;
                            servo_feedback[1] = t4_out.servo_2_angle;
                        }

  #ifdef USE_CLI_DEBUG_PRINT
                        static unsigned printCounter = 1;
                        if (printCounter++ % 100 == 0) {
                            printCounter = 1;
                            cliDebugPrintLinef("Servo position %d cdeg %d cdeg", servo_feedback[0], servo_feedback[1]);
                        }
#endif                      
                    }
                }

                // Reset buffer for next frame
                buffer_servo_idx = 0;
            }
        }
    }
}
#endif