/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Written by Cristian Maglie

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SAMD_OTA_INTERNALFLASH_H_
#define SAMD_OTA_INTERNALFLASH_H_

#pragma once

#include <Arduino.h>


class OTAInternalFlash {
public:
  OTAInternalFlash();

  void write(uint32_t offset, const void *data, uint32_t size);
  void read(uint32_t offset, void *data, uint32_t size);
  bool sameContent(uint32_t offset, const void *data, uint32_t size);
  void readAtAbsoluteAddr(uint32_t addr, void *data, uint32_t size);

  void writeFirmwareAndReboot(int32_t firmwareSize);

  uint32_t get_flash_size() const { return _flash_size;}
  void *get_flash_address() const { return (void*)_flash_address;}
  void *get_sketch_start_address() const { return (void*)_sketch_start;}

#if defined(__SAMD51__)
  static const uint32_t BLOCK_SIZE=8192;
#else
  static const uint32_t BLOCK_SIZE=512;
#endif

private:
  void *_sketch_start;
  void *_flash_address;
  uint32_t _flash_size;
};

#endif	// SAMD_INTERNALFLASH_H_
