.section .text
.global check_limit

# int check_limit(double value1, double value2, double limit)
check_limit:
    # xmm0 = value1
    # xmm1 = value2
    # xmm2 = limit

    # Compute difference: value1 - value2
    subsd %xmm1, %xmm0        # xmm0 = xmm0 - xmm1

    # Absolute value: fabs(value1 - value2)
    andpd abs_mask(%rip), %xmm0   # clear sign bit

    # Compare |diff| with limit
    comisd %xmm2, %xmm0       # compare xmm0 with xmm2
    ja exceeded               # jump if |diff| > limit

    jp exceeded

    # Not exceeded → return 0
    xorl %eax, %eax
    ret

exceeded:
    movl $1, %eax
    ret

.section .rodata
.align 16
abs_mask:
    .quad 0x7FFFFFFFFFFFFFFF
