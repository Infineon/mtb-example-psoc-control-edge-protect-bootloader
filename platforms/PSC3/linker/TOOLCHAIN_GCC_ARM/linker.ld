/***************************************************************************//**
* \file boy2_s_flash.ld
* \version 1.0.0
*
* Linker file for the GNU C compiler.
*
* The main purpose of the linker script is to describe how the sections in the
* input files should be mapped into the output file, and to control the memory
* layout of the output file.
*
* \note The entry point location is fixed and starts at 0x10000000. The valid
* application image should be placed there.
*
* \note The linker files included with the PDL template projects must be generic
* and handle all common use cases. Your project may not use every section
* defined in the linker files. In that case you may see warnings during the
* build process. In your project, you can simply comment out or remove the
* relevant code in the linker file.
*
********************************************************************************
* \copyright
* Copyright 2016-2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)
ENTRY(Reset_Handler)

/* The size of the stack section at the end of CM33 SRAM */
STACK_SIZE = 0x2000;

RAMVECTORS_ALIGNMENT = 128;

/* Force symbol to be entered in the output file as an undefined symbol. Doing
* this may, for example, trigger linking of additional modules from standard
* libraries. You may list several symbols for each EXTERN, and you may use
* EXTERN multiple times. This command has the same effect as the -u command-line
* option.
*/
EXTERN(Reset_Handler)

/* _base_SRAM                          = 0x34000000; sbus ram secure offset */
_base_SRAM                          = RAM_ORIGIN; /* RAM_ORIGIN passed from makefile, 0x34000000 */
/* _size_SRAM                          = 0x00010000; */
_size_SRAM                          = RAM_SIZE; /* RAM_SIZE  passed from makefile */
/* _base_CODE_FLASH_VMA                = 0x32000000; /* cbus flash secure offset */
_base_CODE_FLASH_VMA                = FLASH_ORIGIN + FLASH_OFS; /* FLASH_ORIGIN  passed from makefile */
/* _base_CODE_FLASH_LMA                = 0x32000000; /* sbus flash secure offset */
_base_CODE_FLASH_LMA                = FLASH_ORIGIN + FLASH_OFS;
/* _size_CODE_FLASH                    = 0x00040000; */
_size_CODE_FLASH                    = FLASH_SIZE; /* FLASH_SIZE  passed from makefile */

_size_SRAM_CLEANUP                  = 0x400;

/* The MEMORY section below describes the location and size of blocks of memory in the target.
* Use this section to specify the memory regions available for allocation.
*/
MEMORY
{
    ram               (rwx)   : ORIGIN = _base_SRAM + _size_SRAM_CLEANUP,   LENGTH = _size_SRAM - _size_SRAM_CLEANUP
    flash             (rx)    : ORIGIN = _base_CODE_FLASH_VMA, LENGTH = _size_CODE_FLASH 
}

/* Library configurations */
GROUP(libgcc.a libc.a libm.a libnosys.a)

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   __Vectors_End
 *   __Vectors_Size
 */


SECTIONS
{
    /* Cortex-M33 application flash area */
    .text ORIGIN(flash): AT (_base_CODE_FLASH_LMA)
    {
        /* Cortex-M33 flash vector table */
        __Vectors = . ;
        KEEP(*(.vectors))
        . = ALIGN(4);
        __Vectors_End = .;
        __Vectors_Size = __Vectors_End - __Vectors;
        __end__ = .;

        . = ALIGN(4);
        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        /* Read-only code (constants). */
        *(.rodata .rodata.* .constdata .constdata.* .conststring .conststring.*)

        KEEP(*(.eh_frame*))
    } > flash


    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    __exidx_start = .;

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > flash
    __exidx_end = .;


    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;

        /* Copy interrupt vectors from flash to RAM */
        LONG (__Vectors)                                    /* From */
        LONG (__ram_vectors_start__)                        /* To   */
        LONG ((__Vectors_End - __Vectors) / 4)                    /* Size */

        /* Copy data section to RAM */
        LONG (__etext)                                      /* From */
        LONG (__data_start__)                               /* To   */
        LONG ((__data_end__ - __data_start__) / 4)                /* Size */

        __copy_table_end__ = .;
    } > flash


    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG ((__bss_end__ - __bss_start__) / 4)
        __zero_table_end__ = .;
    } > flash

    __etext =  . ;


    .ramVectors (NOLOAD) : ALIGN(RAMVECTORS_ALIGNMENT)
    {
        __ram_vectors_start__ = .;
        KEEP(*(.ram_vectors))
        __ram_vectors_end__   = .;
    } > ram


    .data __ram_vectors_end__ :AT(_base_CODE_FLASH_LMA + __etext - ORIGIN(flash))
    {
        . = ALIGN(4);
        __data_start__ = .;

        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);

        KEEP(*(.cy_ramfunc*))
        . = ALIGN(4);

        __data_end__ = .;

    } > ram

    .cleanup_helper_section ORIGIN(ram) - _size_SRAM_CLEANUP :
    {
        . = ALIGN(4);
        KEEP(*(.cleanup_helper_area*))
        . = ALIGN(4);
    }

    /* Place variables in the section that should not be initialized during the
    *  device startup.
    */
    .noinit (NOLOAD) : ALIGN(8)
    {
      KEEP(*(.noinit))
    } > ram


    /* The uninitialized global or static variables are placed in this section.
    *
    * The NOLOAD attribute tells linker that .bss section does not consume
    * any space in the image. The NOLOAD attribute changes the .bss type to
    * NOBITS, and that  makes linker to A) not allocate section in memory, and
    * B) put information to clear the section with all zeros during application
    * loading.
    *
    * Without the NOLOAD attribute, the .bss section might get PROGBITS type.
    * This  makes linker to A) allocate zeroed section in memory, and B) copy
    * this section to RAM during application loading.
    */
    .bss (NOLOAD):
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > ram


    .heap (NOLOAD):
    {
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        . = ORIGIN(ram) + LENGTH(ram) - STACK_SIZE;
        __HeapLimit = .;
    } > ram


    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (NOLOAD):
    {
        KEEP(*(.stack*))
    } > ram


    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    . = ALIGN(32);
    __StackTop = ORIGIN(ram) + LENGTH(ram);
    __StackLimit =  __StackTop - STACK_SIZE;
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

}
/* EOF */
