.section .text
.global check_limit

# int check_limit(double value1, double value2, double limit)
check_limit:

    # Step 1: compute difference = value1 - value2
    movapd %xmm0, %xmm3        # xmm3 = value1
    subsd  %xmm1, %xmm3        # xmm3 = xmm3 - xmm1

    # Step 2: check if difference < 0
    xorpd %xmm4, %xmm4         # xmm4 = 0.0
    comisd %xmm4, %xmm3        # compare xmm3 with 0
    jb make_positive            # if xmm3 < 0, jump

continue:

    # Step 3: compare |difference| with limit
    comisd %xmm2, %xmm3        # compare xmm3 with limit
    ja limit_exceeded           # if xmm3 > limit

    # Step 4: return 0 (not exceeded)
    movl $0, %eax
    ret

make_positive:
    # Step 2b: diff = 0 - diff  (manual absolute value)
    subsd %xmm3, %xmm4         # xmm4 = 0 - xmm3
    movapd %xmm4, %xmm3
    jmp continue

limit_exceeded:
    # Step 5: return 1 (limit exceeded)
    movl $1, %eax
    ret