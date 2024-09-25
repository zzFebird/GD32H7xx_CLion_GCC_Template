/* memory map */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 3840K
  SRAM (xrw)      : ORIGIN = 0x24000000, LENGTH = 832K
  SDRAM (xrw)     : ORIGIN = 0XC0000000, LENGTH = 32M
  ITCM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
  DTCM (xrw)      : ORIGIN = 0X20000000, LENGTH = 128K
}

ENTRY(Reset_Handler)

SECTIONS
{
  __stack_size = DEFINED(__stack_size) ? __stack_size : 2K;
  
/* ISR vectors */
  .vectors :
  {
    . = ALIGN(4);
    KEEP(*(.vectors))
    . = ALIGN(4);
    __Vectors_End = .;
    __Vectors_Size = __Vectors_End - __gVectors;
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7) 
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    /* the symbol ��_etext�� will be defined at the end of code section */
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

   .ARM.extab :
  { 
     *(.ARM.extab* .gnu.linkonce.armextab.*) 
  } >FLASH
  
    .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    } >FLASH

  .ARM.attributes : { *(.ARM.attributes) } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array*))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* provide some necessary symbols for startup file to initialize data */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    /* the symbol ��_sdata�� will be defined at the data section end start */
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    /* the symbol ��_edata�� will be defined at the data section end */
    _edata = .;
  } >SRAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    /* the symbol ��_sbss�� will be defined at the bss section start */
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    /* the symbol ��_ebss�� will be defined at the bss section end */
    _ebss = .;
    __bss_end__ = _ebss;
  } >SRAM

 . = ALIGN(8);
  PROVIDE ( end = _ebss );
  PROVIDE ( _end = _ebss );

  .stack ORIGIN(SRAM) + LENGTH(SRAM) - __stack_size :
  {
    PROVIDE( _heap_end = . ); 
    . = __stack_size;  
    PROVIDE( _sp = . ); 
  } >SRAM AT>SRAM

	.sdram_data (NOLOAD) : 
	{ 
	. = ALIGN(4); 
	_sdram_data_begin = .; 
	*(.sdram_data) 
	*(.sdram_data*) 
	. = ALIGN(4); 
	_sdram_data_end = .; 
	} >SDRAM

	.itcm_data (NOLOAD) : 
	{ 
	. = ALIGN(4); 
	_itcm_data_begin = .; 
	*(.itcm_data) 
	*(.itcm_data*) 
	. = ALIGN(4); 
	_itcm_data_end = .; 
	} >ITCM

	.dtcm_data (NOLOAD) : 
	{ 
	. = ALIGN(4); 
	_dtcm_data_begin = .; 
	*(.dtcm_data) 
	*(.dtcm_data*) 
	. = ALIGN(4); 
	_dtcm_data_end = .; 
	} >DTCM
}

 /* input sections */
GROUP(libgcc.a libc.a libm.a libnosys.a)
