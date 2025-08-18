/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

MEMORY
{
	rom (rx):       ORIGIN = 0x08000000, LENGTH = 2048K
	dtcm (rwx):     ORIGIN = 0x20000000, LENGTH = 128K
	itcm (rwx):     ORIGIN = 0x00000000, LENGTH = 64K
	ram (rwx):      ORIGIN = 0x24000000, LENGTH = 512K
	sram1 (rwx):    ORIGIN = 0x30000000, LENGTH = 128K
	sram2 (rwx):    ORIGIN = 0x30020000, LENGTH = 128K
	sram3 (rwx):    ORIGIN = 0x30040000, LENGTH = 32K
	sram4 (rwx):    ORIGIN = 0x38000000, LENGTH = 64K
	backup (rwx):   ORIGIN = 0x38800000, LENGTH = 4K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

PROVIDE(__app_start__ = ORIGIN(rom));
PROVIDE(__app_size__ = _data_loadaddr + _edata - _data - __app_start__);
PROVIDE(__app_code_size__ = _etext - __app_start__);
PROVIDE(__app_ram_size__ = _ebss - ORIGIN(ram));
