/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * STM32H743xI generic setup.
 * 
 * AXI SRAM     - BSS, Data, Heap.
 * SRAM1+SRAM2  - None.
 * SRAM3        - NOCACHE, ETH.
 * SRAM4        - None.
 * DTCM-RAM     - Main Stack, Process Stack.
 * ITCM-RAM     - None.
 * BCKP SRAM    - None.
 */
MEMORY
{
    flash0 (rx) : org = 0x08020000, len = 2M        /* Flash bank1+bank2 */
    flash1 (rx) : org = 0x08020000, len = 1M        /* Flash bank 1 */
    flash2 (rx) : org = 0x08100000, len = 1M        /* Flash bank 2 */
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0nc  (wx) : org = 0x24000000, len = 128k       /* AXI SRAM NON CACHED */
    ram0    (wx) : org = 0x24020000, len = 384k      /* AXI SRAM */
    ram1   (wx) : org = 0x30000000, len = 256k      /* AHB SRAM1+SRAM2 */
    ram2   (wx) : org = 0x30000000, len = 288k      /* AHB SRAM1+SRAM2+SRAM3 */
    ram3   (wx) : org = 0x30040000, len = 32k       /* AHB SRAM3 NON CACHED*/
    ram4   (wx) : org = 0x38000000, len = 64k       /* AHB SRAM4 NON CACHED*/
    ram5   (wx) : org = 0x20000000, len = 128k      /* DTCM-RAM */
    ram6   (wx) : org = 0x00000000, len = 64k       /* ITCM-RAM */
    ram7   (wx) : org = 0x38800000, len = 4k        /* BCKP SRAM */
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram5);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram5);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

/* Stack rules inclusion.*/
INCLUDE rules_stacks.ld

/*===========================================================================*/
/* Custom sections for STM32H7xx.                                            */
/* SRAM3 is assumed to be marked non-cacheable using MPU.                    */
/*===========================================================================*/

/* RAM region to be used for ram0nc (SDMMC1) nocache segment.*/
/* named nocache because SDIO driver is hardwired to declare buffer nocache */
REGION_ALIAS("NOCACHE_RAM", ram0nc);

/* RAM region to be used for ram3 nocache segment.*/
REGION_ALIAS("NOCACHE3_RAM", ram3);

/* RAM region to be used for ram4 (BDMA) nocache segment.*/
REGION_ALIAS("NOCACHE4_RAM", ram4);

/* RAM region to be used for eth segment.*/
REGION_ALIAS("ETH_RAM", ram3);

SECTIONS
{
    /* Special section for non cache-able areas.*/
    .nocache (NOLOAD) : ALIGN(4)
    {
        __nocache_base__ = .;
        *(.nocache)
        *(.nocache.*)
        *(.bss.__nocache_*)
        . = ALIGN(4);
        __nocache_end__ = .;
    	} > NOCACHE_RAM

/* Special section for non cache-able areas.*/
    .nocache3 (NOLOAD) : ALIGN(4)
    {
        __nocache3_base__ = .;
        *(.nocache3)
        *(.nocache3.*)
        *(.bss.__nocache3_*)
        . = ALIGN(4);
        __nocache3_end__ = .;
    } > NOCACHE3_RAM

/* Special section for non cache4-able areas.*/
    .nocache4 (NOLOAD) : ALIGN(4)
    {
        __nocache4_base__ = .;
        *(.nocache4)
        *(.nocache4.*)
        *(.bss.__nocache4_*)
        . = ALIGN(4);
        __nocache4_end__ = .;
    } > NOCACHE4_RAM

    /* Special section for Ethernet DMA non cache-able areas.*/
    .eth (NOLOAD) : ALIGN(4)
    {
        __eth_base__ = .;
        *(.eth)
        *(.eth.*)
        *(.bss.__eth_*)
        . = ALIGN(4);
        __eth_end__ = .;
    } > ETH_RAM
}

/* Code rules inclusion.*/
INCLUDE rules_code.ld

/* Data rules inclusion.*/
INCLUDE rules_data.ld

/* Custom memory rules.*/

__ram0nc_base__         = ORIGIN(ram0nc);
__ram0nc_size__         = LENGTH(ram0nc);
__ram0nc_end__          = __ram0nc_base__ + __ram0nc_size__;

SECTIONS
{
     .ram0nc_init : ALIGN(4)
    {
        __ram0nc_init_text__ = LOADADDR(.ram0nc_init);
        __ram0nc_init__ = .;
        KEEP(*(.ram0nc_init))
        KEEP(*(.ram0nc_init.*))
        . = ALIGN(4);
    } > ram0nc AT > RAM_INIT_FLASH_LMA

    .ram0nc (NOLOAD) : ALIGN(4)
    {
        __ram0nc_clear__ = .;
        *(.ram0nc_clear)
        *(.ram0nc_clear.*)
        . = ALIGN(4);
        __ram0nc_noinit__ = .;
        *(.ram0nc)
        *(.ram0nc.*)
        . = ALIGN(4);
        __ram0nc_free__ = .;
    } > ram0nc
}

/* Memory rules inclusion.*/
INCLUDE rules_memory.ld

