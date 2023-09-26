New log:
- quatAtt 8: between -1 and 1. so (quat * ((127 << 6) - 1)) DONE
- quatSp 8: same as quatAtt 1
- omegaSp 6: same as gyro is logged now (deg/s?)
- dv 8: signed, dv*100?
- alphaCurrent 6: deg/s/s
- u 8: same as quatAtt, because in future maybe signed
- u_state 8: same as u
- dshot_erpm: whatever it is now
- pos 6: in mm, so *1000 and likely 2 byte max
- posSp 6: same as pos
- vel 6: in cm/s, so *100 and likely 2 byte max

8+8+6+8+6+8+8+6+6+6 = 70

Unlog:
- axisP - 6?
- axisI - 6
- axisD - 6
- axisF - 6
- setpoint - 8?
saving 6+6+6+6+8 = 32 out of 50.

So we're increasing estimated log size by 70-32 = 38 bytes, so (38+50)/50 = 176% filesize. Should be fine TM


## Process
### Conditions
#### blackbox_fielddefs.h:
1. add to FlightLogFieldCondition after DEBUG_LOG (I only put INDI for all of the new ones)
2. add to FlightLogFieldSelect_e after GPS (I only put INDI for all of the new ones)
##### blackbox.c:
1. add to testBlackboxConditionUncached

### Encodings/Values
For all of the below, check for USE_INDI first
1. add to blackboxMainFields after SERVO
2. add to blackboxMainState_t, simply at the end
3. add to loadMainState
4. add to writeIntraframe after MOTOR
5. add to writeInterframe after MOTOR

settings:
- add to blackboxWriteSysinfo and don't bother with the defines
