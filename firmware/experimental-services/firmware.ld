ENTRY(_start)
MEMORY { ROM (rx) : ORIGIN = 0x80000000, LENGTH = 16K
 RAM (rw) : ORIGIN = 0x80004000, LENGTH = 1K }
SECTIONS {
 .text : { KEEP(*(.text.start)) *(.text*) } > ROM
 .rodata : { *(.rodata*) *(.srodata*) } > ROM
 .data : { *(.data*) *(.sdata*) } > RAM
 .bss (NOLOAD) : { . = ALIGN(4); __bss_start = .; *(.bss*) *(.sbss*) *(COMMON); . = ALIGN(4); __bss_end = .; } > RAM
 _stack_top = 0x80004400;
 ASSERT(SIZEOF(.data) == 0, "Initialized RAM excluded")
 ASSERT(__bss_end < 0x80004200, "Reserve512B stack")
}
