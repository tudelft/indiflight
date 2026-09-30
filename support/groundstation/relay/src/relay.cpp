// Copyright 2024 Erin Lucassen (Delft University of Technology)
//                Robin Ferede (Delft University of Technology)
//                Stavrow Bahnam (Delft University of Technology)
//                Till Blaha (Delft University of Technology)
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>

#include <fstream>
#include <sys/time.h>
#include <chrono>
#include <ctime>
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;


#include <iostream>
// to use the std:min function
#include <algorithm>
#include "pi-protocol.h"
#include "pi-messages.h"

#include <stdlib.h>
#include <arpa/inet.h>
#include <math.h>

#define OPTITRACK_PORT 5005
#define SETPOINT_PORT 5006
#define KEYBOARD_PORT 5007
#define NN_FEATURE_PORT 5010
#define MAX_BUFFER_SIZE 1024
#define PI_MSG_PAYLOAD_OFFSET (PI_MSG_ID_BYTES + PI_MSG_PAYLOAD_LEN_BYTES)

// NN_INPUT_CHUNK carries a 64-float vector split into 2 chunks of 32 (see
// lib/main/pi-protocol/msgs/NN_INPUT_CHUNK.yaml: a single message can't fit
// all 64 floats under the 255 byte payload cap). Port 5010 carries depth_hitl's
// "DPTH" datagram (DiffSim/hitl/src/netout.hpp): a 76-byte header — magic,
// version, seq, a uint16 payload type at offset 12, a uint32 payload length at
// offset 20, pose metadata — followed by the payload, all native little-endian
// (both this host and the ground station are LE, so no byte-swap is applied).
// Only the CNN-feature payload (type 2, 64 float32) is forwarded; relay splits
// it into the 2 wire messages here.
#define NN_FEATURE_DIM 64
#define NN_FEATURE_CHUNK_SIZE 32
#define DEPTH_HEADER_BYTES 76
#define DEPTH_MAGIC "DPTH"
#define DEPTH_PAYLOAD_FEATURES_F32 2
static_assert(NN_FEATURE_DIM % NN_FEATURE_CHUNK_SIZE == 0,
    "NN_FEATURE_DIM must be an exact multiple of NN_FEATURE_CHUNK_SIZE");
static_assert(PI_MSG_NN_INPUT_CHUNK_PAYLOAD_LEN == 1 + NN_FEATURE_CHUNK_SIZE * sizeof(float),
    "NN_FEATURE_CHUNK_SIZE doesn't match the field count in msgs/NN_INPUT_CHUNK.yaml");

// hypersimple on-demand status updates similar to dd
// https://en.wikipedia.org/wiki/C_signal_handling
// idea: when sending SIGUSR1 -- print messages
//       when sending SIGUSR2 -- print statistics
// TODO: fork this program into two threads so that the main program isn't 
//       interrupted for stats/msgs. Neglect membar's, because we dont care if 
//       sometimes messages are garbled during the print and this hopefully 
//       wont occur too often
#include <signal.h>
#include <stdlib.h>

static void catch_function(int signo) {
    switch(signo) {
        case SIGUSR1:
            piPrintMsgs(&printf);
            break;
        case SIGUSR2:
            piPrintStats(&printf);
            break;
    }
}

int openSerialPort(const char* port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("openSerialPort: Unable to open port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

int openUdpPort(const uint16_t port) {
    int sockfd;
    struct sockaddr_in server_addr;

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Make socket non-blocking
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    memset(&server_addr, 0, sizeof(server_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    // Bind socket
    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

int serialPortFd; // needs to be global to be used inside serialWriter

void serialWriter(uint8_t byte)
{
    //printf("%d", serialPortFd);
    write(serialPortFd, &byte, 1);
}

float ntohf(float in) {
    uint32_t dummy;
    dummy = ntohl(*(uint32_t*)&(in));
    return *(float*)(&dummy);
}

// for external pose
typedef struct pose_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} pose_t;

typedef struct pose_der_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float wx;
    float wy;
    float wz;
} pose_der_t;

pose_t pose;
pose_der_t pose_der;

static pi_parse_states_t piParseStates = {0};

void print_usage(char* progname) {
    printf("Usage:\n\n");
    printf("%s serial_port baudrate\n\n", progname);
}

int main(int argc, char** argv) {
    // enable logging if the first argument is -l
    if (argc != 3) {
        print_usage(argv[0]);
        return 1;
    }

    const char* serialPort = argv[1];
// #ifdef ORIN
//     const char* serialPort = "/dev/ttyTHS1";
// #else
//     //const char* serialPort = "/dev/ttyAMA0";
//     const char* serialPort = "/dev/ttyDB";
// #endif

    int baudrate;
    switch (atoi(argv[2])) {
        case 38400: baudrate = B38400; break;
        case 57600: baudrate = B57600; break;
        case 115200: baudrate = B115200; break;
        case 230400: baudrate = B230400; break;
        case 460800: baudrate = B460800; break;
        case 500000: baudrate = B500000; break;
        case 921600: baudrate = B921600; break;
        case 1000000: baudrate = B1000000; break;
        case 1500000: baudrate = B1500000; break;
        default:
            printf("Baudrate %s not supported\n", argv[2]);
            return 2;
    }

    serialPortFd = openSerialPort(serialPort, baudrate);
    if (serialPortFd == -1) {
        return EXIT_FAILURE;
    }

    // register signal handlers (https://en.wikipedia.org/wiki/C_signal_handling)
    if (signal(SIGUSR1, catch_function) == SIG_ERR) {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        return EXIT_FAILURE;
    }
    if (signal(SIGUSR2, catch_function) == SIG_ERR) {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        return EXIT_FAILURE;
    }

    uint8_t piBuffer[PI_MAX_PACKET_LEN];

    // open udps
    int optitrackFd = openUdpPort(OPTITRACK_PORT);
    printf("Server listening on port %d for Optitrack...\n", OPTITRACK_PORT);
    static constexpr size_t OPTITRACK_BUFFER_SIZE = sizeof(unsigned int) + sizeof(pose_t) + sizeof(pose_der_t);
    uint8_t optitrackBuffer[OPTITRACK_BUFFER_SIZE];

    int setpointFd = openUdpPort(SETPOINT_PORT);
    printf("Server listening on port %d for Setpoints...\n", SETPOINT_PORT);
    uint8_t setpointBuffer[PI_MSG_POS_SETPOINT_PAYLOAD_LEN];

    int keyboardFd = openUdpPort(KEYBOARD_PORT);
    printf("Server listening on port %d for Keystrokes...\n", KEYBOARD_PORT);
    uint8_t keyboardBuffer[PI_MSG_KEYBOARD_PAYLOAD_LEN];

    int nnFeatureFd = openUdpPort(NN_FEATURE_PORT);
    printf("Server listening on port %d for NN input features...\n", NN_FEATURE_PORT);
    static constexpr size_t NN_FEATURE_PAYLOAD_SIZE = NN_FEATURE_DIM * sizeof(float);
    static constexpr size_t NN_FEATURE_BUFFER_SIZE = DEPTH_HEADER_BYTES + NN_FEATURE_PAYLOAD_SIZE;
    uint8_t nnFeatureBuffer[NN_FEATURE_BUFFER_SIZE];

    while (true) {
        // Q1: doesnt this add a lot of delay? Because the buffer is only filled once
        // PI_MAX_PACKET_LENGTH is read?

        // Q2: Is the UART TX buffer even big enough?
        // answer, seems like the PL011 buffer is 32byte deep. termios wraps it
        // in a PAGE_SIZE deep buffer (4096bytes), so this is fine at least
        ssize_t numBytes = read(serialPortFd, piBuffer, PI_MAX_PACKET_LEN);
        bool newMessage = false;
        if (numBytes) {
            for (int i=0; i < numBytes; i++) {
                if (piParse(&piParseStates, piBuffer[i]) == PI_MSG_EKF_INPUTS_ID) {
                    newMessage = true;
                }
            }
        }

        if ((piMsgEkfInputsRxState == PI_MSG_RX_STATE_NONE) || (!newMessage)) {
            // cannot go on, no time information to timestamp gps msgs, or setpoints
            usleep(100); // reduce CPU load a bit
            continue;
        } else {
            newMessage = false;
        }

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int optitrackBytes = recvfrom(optitrackFd, (uint8_t *)(optitrackBuffer), OPTITRACK_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (optitrackBytes > 0) {
            // we have optitrack copy from the buffer into the structs
            memcpy((uint8_t *)(&pose), optitrackBuffer+sizeof(unsigned int), sizeof(pose_t));
            memcpy((uint8_t *)(&pose_der), optitrackBuffer+sizeof(unsigned int)+sizeof(pose_t), sizeof(pose_der_t));

            // proceed to send Fake GPS and External Pose
            piMsgFakeGpsTx.time_us = piMsgEkfInputsRx->time_us;
            static constexpr double CYBERZOO_LAT = 51.99071002805145;
            static constexpr double CYBERZOO_LON = 4.376727452462819;
            static constexpr double RE = 6378137.;
            piMsgFakeGpsTx.lat = (int) 1e7 * 
                (CYBERZOO_LAT + 180. / M_PI * (pose.x / RE));
            piMsgFakeGpsTx.lon = (int) 1e7 *
                (CYBERZOO_LON + 180 / M_PI * (pose.y / RE) / cos(CYBERZOO_LON * M_PI / 180.));
            piMsgFakeGpsTx.altCm = (int) (pose.z * 100.f);
            piMsgFakeGpsTx.hdop =  (short) 150;
            piMsgFakeGpsTx.groundSpeed = (short) (hypotf(pose_der.x, pose_der.y) * 100.f);
            piMsgFakeGpsTx.groundCourse = (short) (1800.f * atan2(pose_der.x, pose_der.y) / M_PI);
            piMsgFakeGpsTx.numSat = 8;
            piSendMsg(&piMsgFakeGpsTx, &serialWriter);

            piMsgExternalPoseTx.time_us = piMsgEkfInputsRx->time_us;
            piMsgExternalPoseTx.ned_x   = pose.x;
            piMsgExternalPoseTx.ned_y   = pose.y;
            piMsgExternalPoseTx.ned_z   = pose.z;
            piMsgExternalPoseTx.ned_xd  = pose_der.x;
            piMsgExternalPoseTx.ned_yd  = pose_der.y;
            piMsgExternalPoseTx.ned_zd  = pose_der.z;
            piMsgExternalPoseTx.body_qi = pose.qw;
            piMsgExternalPoseTx.body_qx = pose.qx;
            piMsgExternalPoseTx.body_qy = pose.qy;
            piMsgExternalPoseTx.body_qz = pose.qz;
            piSendMsg(&piMsgExternalPoseTx, &serialWriter);
            printf("relayed EXTERNAL_POSE \n");
        }

        // ---- setpoints ----
        pi_POS_SETPOINT_t msgPosSetpoint;
        int setpointBytes = recvfrom(setpointFd, (uint8_t *)(setpointBuffer), PI_MSG_POS_SETPOINT_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (setpointBytes > 0) {
            memcpy((uint8_t *)(&msgPosSetpoint)+PI_MSG_PAYLOAD_OFFSET, setpointBuffer, PI_MSG_POS_SETPOINT_PAYLOAD_LEN);
            piMsgPosSetpointTx.time_us = piMsgEkfInputsRx->time_us;

            piMsgPosSetpointTx.ned_x  = ntohf(msgPosSetpoint.ned_x);
            piMsgPosSetpointTx.ned_y  = ntohf(msgPosSetpoint.ned_y);
            piMsgPosSetpointTx.ned_z  = ntohf(msgPosSetpoint.ned_z);
            piMsgPosSetpointTx.ned_xd = ntohf(msgPosSetpoint.ned_xd);
            piMsgPosSetpointTx.ned_yd = ntohf(msgPosSetpoint.ned_yd);
            piMsgPosSetpointTx.ned_zd = ntohf(msgPosSetpoint.ned_zd);
            piMsgPosSetpointTx.yaw = ntohf(msgPosSetpoint.yaw);
            piSendMsg(&piMsgPosSetpointTx, &serialWriter);
            printf("relayed SETPOINT \n");
        }

        // ---- keyboard ----
        pi_KEYBOARD_t msgKeyboard;
        int keyboardBytes = recvfrom(keyboardFd, (uint8_t *)(keyboardBuffer), PI_MSG_KEYBOARD_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (keyboardBytes > 0) {
            memcpy((uint8_t *)(&msgKeyboard)+PI_MSG_PAYLOAD_OFFSET, keyboardBuffer, PI_MSG_KEYBOARD_PAYLOAD_LEN);
            piMsgKeyboardTx.time_us = piMsgEkfInputsRx->time_us;
            piMsgKeyboardTx.key = msgKeyboard.key;
            piSendMsg(&piMsgKeyboardTx, &serialWriter);
            printf("relayed KEYBOARD \n");

        }

        // ---- nn input features ----
        int nnFeatureBytes = recvfrom(nnFeatureFd, (uint8_t *)(nnFeatureBuffer), NN_FEATURE_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (nnFeatureBytes == (int) NN_FEATURE_BUFFER_SIZE &&
            memcmp(nnFeatureBuffer, DEPTH_MAGIC, 4) == 0) {
            uint16_t payloadType;
            uint32_t payloadBytes;
            memcpy(&payloadType, nnFeatureBuffer + 12, sizeof(payloadType));
            memcpy(&payloadBytes, nnFeatureBuffer + 20, sizeof(payloadBytes));

            if (payloadType != DEPTH_PAYLOAD_FEATURES_F32 || payloadBytes != NN_FEATURE_PAYLOAD_SIZE) {
                printf("NN_INPUT_CHUNK: DPTH header says type %u / %u bytes, expected type %d / %zu bytes, dropping\n",
                       payloadType, payloadBytes, DEPTH_PAYLOAD_FEATURES_F32, NN_FEATURE_PAYLOAD_SIZE);
            } else {
                float vals[NN_FEATURE_DIM];
                // Native little-endian on both ends (see the comment above) —
                // no byte-swap, unlike the network-byte-order streams below.
                memcpy(vals, nnFeatureBuffer + DEPTH_HEADER_BYTES, sizeof(vals));

                for (size_t chunk = 0; chunk < NN_FEATURE_DIM / NN_FEATURE_CHUNK_SIZE; chunk++) {
                    const float* c = vals + chunk * NN_FEATURE_CHUNK_SIZE;
                    piMsgNnInputChunkTx.chunk_index = (uint8_t) chunk;
                    piMsgNnInputChunkTx.f0  = c[0];  piMsgNnInputChunkTx.f1  = c[1];
                    piMsgNnInputChunkTx.f2  = c[2];  piMsgNnInputChunkTx.f3  = c[3];
                    piMsgNnInputChunkTx.f4  = c[4];  piMsgNnInputChunkTx.f5  = c[5];
                    piMsgNnInputChunkTx.f6  = c[6];  piMsgNnInputChunkTx.f7  = c[7];
                    piMsgNnInputChunkTx.f8  = c[8];  piMsgNnInputChunkTx.f9  = c[9];
                    piMsgNnInputChunkTx.f10 = c[10]; piMsgNnInputChunkTx.f11 = c[11];
                    piMsgNnInputChunkTx.f12 = c[12]; piMsgNnInputChunkTx.f13 = c[13];
                    piMsgNnInputChunkTx.f14 = c[14]; piMsgNnInputChunkTx.f15 = c[15];
                    piMsgNnInputChunkTx.f16 = c[16]; piMsgNnInputChunkTx.f17 = c[17];
                    piMsgNnInputChunkTx.f18 = c[18]; piMsgNnInputChunkTx.f19 = c[19];
                    piMsgNnInputChunkTx.f20 = c[20]; piMsgNnInputChunkTx.f21 = c[21];
                    piMsgNnInputChunkTx.f22 = c[22]; piMsgNnInputChunkTx.f23 = c[23];
                    piMsgNnInputChunkTx.f24 = c[24]; piMsgNnInputChunkTx.f25 = c[25];
                    piMsgNnInputChunkTx.f26 = c[26]; piMsgNnInputChunkTx.f27 = c[27];
                    piMsgNnInputChunkTx.f28 = c[28]; piMsgNnInputChunkTx.f29 = c[29];
                    piMsgNnInputChunkTx.f30 = c[30]; piMsgNnInputChunkTx.f31 = c[31];
                    piSendMsg(&piMsgNnInputChunkTx, &serialWriter);
                }
                printf("relayed NN_INPUT_CHUNK (2 chunks) \n");
            }
        } else if (nnFeatureBytes > 0) {
            printf("NN_INPUT_CHUNK: got %d bytes, expected %zu, dropping\n", nnFeatureBytes, NN_FEATURE_BUFFER_SIZE);
        }
    }

    close(serialPortFd);
    close(optitrackFd);
    close(setpointFd);
    close(keyboardFd);
    close(nnFeatureFd);

    return 0;
}
