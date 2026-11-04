.section .text.start,"ax",@progbits
.globl _start
_start:
 .option push
 .option norelax
 la sp, _stack_top
 .option pop
 call main
1: j 1b
