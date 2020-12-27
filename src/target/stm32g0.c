/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2021 Fabrice Prost-Boucle <fabalthazar@falbalab.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This file implements STM32G0 target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 *
 * References:
 * RM0454 - Rev 5
 *   Reference manual - STM32G0x0 advanced ARM(R)-based 32-bit MCUs
 *                      (STM32G030/STM32G050/STM32G070/STM32G0B0)
 * RM0444 - Rev 5
 *   Reference manual - STM32G0x1 advanced ARM(R)-based 32-bit MCUs
 *                      (STM32G031/STM32G041/STM32G051/STM32G061/
 *                       STM32G071/STM32G081/STM32G0B1/STM32G0C1)
 * PM0223 - Rev 5
 *   Programming manual - Cortex(R)-M0+ programming manual for STM32L0, STM32G0,
 *                        STM32WL and STM32WB Series
 */



static int skeleton_flash_erase(struct target_flash *f,
                                target_addr addr, size_t len);
static int skeleton_flash_write(struct target_flash *f,
                                target_addr dest, const void *src, size_t len);

static void skeleton_add_flash(target *t)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	f->start = SKELETON_FLASH_BASE;
	f->length = SKELETON_FLASH_SIZE;
	f->blocksize = SKELETON_BLOCKSIZE;
	f->erase = skeleton_flash_erase;
	f->write = skeleton_flash_write;
	target_add_flash(t, f);
}

bool skeleton_probe(target *t)
{
	if (target_mem_read32(t, SKELETON_DEVID_ADDR) == SKELETON_DEVID) {
		skeleton_add_flash(t);
		return true;
	} else {
		return false;
	}
}
