;/* only gcc */
.syntax unified ; /* allow thumb-2 32bit instruction encoding */

;/* this is stolen from DPS/Source/TransformFunctions/arm_bitreversal2.S */
.global scalar_mult_f32_asm_4
.type scalar_mult_f32_asm_4, %function

scalar_mult_f32_asm_4:
    ;/* pointer to a is in r0
    ;* pointer to b is in r1
    ;* number if values is in r2
    ;* 
    ;* check r2 >= 16. If so, continue
    ;* if r2 < 16, do the rest in unblocked code*/

    ;/* multiply r2 by the size of a float work */
    mov r3, #4
    mla r3, r3, r2, r0
    subs r2, r3, #16
    cmp r2, r0
    bmi finish_4

    ;/* todo: r2 can be eliminated by just using the pointer in r0 as "index" */

outer_4:
    vldmia.32 r0!, {s0 - s3} /* ! means auto increment r0 */
    vldmdb.32 r3!, {s16 - s19}
    vmul.f32  s0, s0, s16
    vmul.f32  s1, s1, s17
    vmul.f32  s2, s2, s18
    vmul.f32  s3, s3, s19
    vstmia.32 r1!, {s0 - s3}

    cmp r2, r0
    bpl outer_4

finish_4:
    adds r2, r2, #16
    b finish


.global scalar_mult_f32_asm_8
.type scalar_mult_f32_asm_8, %function

scalar_mult_f32_asm_8:
    ;/* pointer to a is in r0
    ;* pointer to b is in r1
    ;* number if values is in r2
    ;* 
    ;* check r2 >= 16. If so, continue
    ;* if r2 < 16, do the rest in unblocked code*/

    ;/* multiply r2 by the size of a float work */
    mov r3, #4
    mla r3, r3, r2, r0
    subs r2, r3, #32
    cmp r2, r0
    bmi finish_8

    ;/* todo: r2 can be eliminated by just using the pointer in r0 as "index" */

outer_8:
    vldmia.32 r0!, {s0 - s7} /* ! means auto increment r0 */
    vldmdb.32 r3!, {s16 - s23}
    vmul.f32  s0, s0, s16
    vmul.f32  s1, s1, s17
    vmul.f32  s2, s2, s18
    vmul.f32  s3, s3, s19
    vmul.f32  s4, s4, s20
    vmul.f32  s5, s5, s21
    vmul.f32  s6, s6, s22
    vmul.f32  s7, s7, s23
    vstmia.32 r1!, {s0 - s7}

    cmp r2, r0
    bpl outer_8

finish_8:
    adds r2, r2, #32
    b finish


.global scalar_mult_f32_asm_16
.type scalar_mult_f32_asm_16, %function

scalar_mult_f32_asm_16:
    ;/* pointer to a is in r0
    ;* pointer to b is in r1
    ;* number if values is in r2
    ;* 
    ;* check r2 >= 16. If so, continue
    ;* if r2 < 16, do the rest in unblocked code*/

    ;/* multiply r2 by the size of a float work */
    mov r3, #4
    mla r3, r3, r2, r0
    subs r2, r3, #64
    cmp r2, r0
    bmi finish_16

    ;/* todo: r2 can be eliminated by just using the pointer in r0 as "index" */

outer_16:
    vldmia.32 r0!, {s0 - s15} /* ! means auto increment r0 */
    vldmdb.32 r3!, {s16 - s31}
    vmul.f32  s0, s0, s16
    vmul.f32  s1, s1, s17
    vmul.f32  s2, s2, s18
    vmul.f32  s3, s3, s19
    vmul.f32  s4, s4, s20
    vmul.f32  s5, s5, s21
    vmul.f32  s6, s6, s22
    vmul.f32  s7, s7, s23
    vmul.f32  s8, s8, s24
    vmul.f32  s9, s9, s25
    vmul.f32  s10, s10, s26
    vmul.f32  s11, s11, s27
    vmul.f32  s12, s12, s28
    vmul.f32  s13, s13, s29
    vmul.f32  s14, s14, s30
    vmul.f32  s15, s15, s31
    vstmia.32 r1!, {s0 - s15}

    cmp r2, r0
    bpl outer_16

finish_16:
    adds r2, r2, #64
    b finish ; /* could be omitted */



finish:
    cmp r2, r0
    beq exit
    vldmia.32 r0!, {s0}
    vldmdb.32 r3!, {s16}
    vmul.f32 s0, s0, s16
    vstmia.32 r1!, {s0}
    b finish

exit:
    bx lr
