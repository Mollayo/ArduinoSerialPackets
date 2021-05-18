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


#include <Arduino.h>
#include "SAMD_OTAInternalFlash.h"


#define FLASH_PAGE_SIZE (8 << NVMCTRL->PARAM.bit.PSZ)
#define FLASH_NUM_PAGES NVMCTRL->PARAM.bit.NVMP
#define FLASH_SIZE (FLASH_PAGE_SIZE * FLASH_NUM_PAGES)
#define FLASH_BLOCK_SIZE (FLASH_PAGE_SIZE * 16)

extern "C" {
char * __text_start__(); // 0x2000, 0x0 without bootloader and 0x4000 for M0 original bootloader
extern uint32_t __etext; // CODE END. Symbol exported from linker script


/**************************************************************************/
/*!
    @brief  Internal function, waits until flash memory controller is idle.
*/
/**************************************************************************/
// These functions must be in RAM (.data) and NOT inlined as they erase and copy the sketch data in flash
__attribute__ ((long_call, noinline, section (".data#")))
static void wait_ready() {
#if defined(__SAMD51__)
  while (!NVMCTRL->STATUS.bit.READY);
#else
  while (!NVMCTRL->INTFLAG.bit.READY);
#endif
}


#if defined(__SAMD51__)
// Source: https://github.com/adafruit/Adafruit_Arcada/blob/master/Adafruit_Arcada_InternalFlash.cpp
// These functions must be in RAM (.data) and NOT inlined as they erase and copy the sketch data in flash
__attribute__ ((long_call, noinline, section (".data#")))
static void writeDataToFlash(uint8_t *flashAddress, uint8_t *ramAddress,uint32_t len)
{
  uint16_t saveCache = NVMCTRL->CTRLA.reg; // Cache in Rev a silicon
  NVMCTRL->CTRLA.bit.CACHEDIS0 = true;     // isn't reliable when
  NVMCTRL->CTRLA.bit.CACHEDIS1 = true;     // writing to NVM.

  // Set manual write mode - only needed once, not in loop
  NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN;

  // Clear page buffer, only needed once, quadword write also clears it
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;

  for (uint8_t tries = 0;;) { // Repeat write sequence until success or limit

    uint8_t *src = (uint8_t *)ramAddress;     // Maintain passed-in pointers,
    uint32_t *dst = (uint32_t *)flashAddress; // modify these instead.
    int32_t bytesThisPass, bytesToGo = len;

    wait_ready(); // Wait for any NVM write op in progress

    while (bytesToGo > 0) {
      // Because dst (via flashAddress) is always quadword-aligned at this
      // point, and flash blocks are known to be a quadword-multiple size,
      // this comparison is reasonable for checking for start of block...
      if (!((uint32_t)dst % FLASH_BLOCK_SIZE)) { // At block boundary
        // If ANY changed data within the entire block, it must be erased
        bytesThisPass = min(FLASH_BLOCK_SIZE, bytesToGo);

        uint8_t diff=0;
        for (uint32_t i=0;i<bytesThisPass;i++)
        {
          if (src[i]!=dst[i])
          {
            diff=1;
            break;
          }
        }
        // memcpy does not work in this context
        if (diff) { // >0 if different
          wait_ready();
          NVMCTRL->ADDR.reg = (uint32_t)dst; // Destination address in flash
          NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
        } else {              // Skip entire block
          bytesToGo -= bytesThisPass;
          src += FLASH_BLOCK_SIZE; // Advance to next block
          dst += FLASH_BLOCK_SIZE / 4;
          continue;
        }
      }

      // Examine next quadword, write only if needed (reduce flash wear)
      bytesThisPass = min(16, bytesToGo);

      uint8_t diff=0;
      for (uint32_t i=0;i<bytesThisPass;i++)
      {
        if (src[i]!=dst[i])
        {
          diff=1;
          break;
        }
      }
      // memcpy does not work in this context
      if (diff) { // >0 if different
        // src might not be 32-bit aligned and must be read byte-at-a-time.
        // dst write ops MUST be 32-bit! Won't work with memcpy().
        dst[0] = src[0] | (src[1] << 8) | (src[2] << 16) | (src[3] << 24);
        dst[1] = src[4] | (src[5] << 8) | (src[6] << 16) | (src[7] << 24);
        dst[2] = src[8] | (src[9] << 8) | (src[10] << 16) | (src[11] << 24);
        dst[3] = src[12] | (src[13] << 8) | (src[14] << 16) | (src[15] << 24);
        // Trigger the quadword write
        wait_ready();
        NVMCTRL->ADDR.reg = (uint32_t)dst;
        NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW;
      }
      bytesToGo -= bytesThisPass;
      src += 16; // Advance to next quadword
      dst += 4;
    }
    wait_ready(); // Wait for last write to finish
    break;
  }

  NVMCTRL->CTRLA.reg = saveCache; // Restore NVM cache settings
}

#else

// Source: https://github.com/cmaglie/FlashStorage/blob/master/src/FlashStorage.cpp

static const uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
uint32_t PAGE_SIZE=pageSizes[NVMCTRL->PARAM.bit.PSZ];
uint32_t ROW_SIZE=PAGE_SIZE * 4;

__attribute__ ((long_call, noinline, section (".data#")))
static uint32_t read_unaligned_uint32(const void *data)
{
  union {
    uint32_t u32;
    uint8_t u8[4];
  } res;
  const uint8_t *d = (const uint8_t *)data;
  res.u8[0] = d[0];
  res.u8[1] = d[1];
  res.u8[2] = d[2];
  res.u8[3] = d[3];
  return res.u32;
}


__attribute__ ((long_call, noinline, section (".data#")))
static void write(const void *flash_ptr, const void *data, uint32_t size)
{
  // Calculate data boundaries
  size = (size + 3) / 4;
  volatile uint32_t *dst_addr = (uint32_t *)flash_ptr;
  const uint8_t *src_addr = (uint8_t *)data;

  // Disable automatic page write
  NVMCTRL->CTRLB.bit.MANW = 1;

  // Do writes in pages
  while (size) {
    // Execute "PBC" Page Buffer Clear
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    wait_ready();

    // Fill page buffer
    uint32_t i;
    for (i=0; i<(PAGE_SIZE/4) && size; i++) {
      *dst_addr = read_unaligned_uint32(src_addr);
      src_addr += 4;
      dst_addr++;
      size--;
    }

    // Execute "WP" Write Page
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
    wait_ready();
  }
}

__attribute__ ((long_call, noinline, section (".data#")))
static void eraseOneAddr(const void *flash_ptr)
{
  NVMCTRL->ADDR.reg = ((uint32_t)flash_ptr) / 2;
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
  wait_ready();
}

__attribute__ ((long_call, noinline, section (".data#")))
static void erase(const void *flash_ptr, uint32_t size)
{
  const uint8_t *ptr = (const uint8_t *)flash_ptr;
  while (size > ROW_SIZE) {
    eraseOneAddr(ptr);
    ptr += ROW_SIZE;
    size -= ROW_SIZE;
  }
  eraseOneAddr(ptr);
}


// These functions must be in RAM (.data) and NOT inlined as they erase and copy the sketch data in flash
__attribute__ ((long_call, noinline, section (".data#")))
static void writeDataToFlash(uint8_t *dest, uint8_t *src,uint32_t len)
{
  // We do it incrementaly so that src and dest can partially overlap
  // This makes possible to upload a firmware larger than half of the flash size
  // by uploading a minimal firmware and then the large one
  uint32_t addr=0;
  while(addr<len)
  {
    // Erase first
    erase(dest+addr, ROW_SIZE);
    // Write the data
    write(dest+addr, src+addr, ROW_SIZE);
    // Should test here if the data have been well writen into the flash
    //...
    // Go to next block
    addr=addr+ROW_SIZE;
  }
}

#endif

__attribute__ ((long_call, noinline, section (".data#")))
static void doWriteFirmwareAndReboot(void *dst, void *src, int32_t size)
{
  // Disable the interrupts. Otherwise, the copy does not work
  __disable_irq();
  // Copy everything at once
  writeDataToFlash((uint8_t*)dst, (uint8_t*)src, size);
  // Reboot the MCU
  NVIC_SystemReset();
}
}

// Replace the firmware with the one that has been uploaded into the internal flash
void OTAInternalFlash::writeFirmwareAndReboot(int32_t firmwareSize)
{
  doWriteFirmwareAndReboot((void*)_sketch_start,(void*)_flash_address, firmwareSize);
}

OTAInternalFlash::OTAInternalFlash()
{
  _sketch_start=(void*)__text_start__;            // Begin the of the sketch
  _flash_address = (uint8_t *)&__etext; // OK to overwrite the '0' there
  uint16_t partialBlock = (uint32_t)_flash_address % FLASH_BLOCK_SIZE;
  if (partialBlock) {
    _flash_address += FLASH_BLOCK_SIZE - partialBlock;
  }
  // Move ahead one block. This shouldn't be necessary, but for
  // some reason certain programs are clobbering themselves.
  _flash_address += FLASH_BLOCK_SIZE;
  _flash_size=FLASH_SIZE-(int)_flash_address;
}

void OTAInternalFlash::write(uint32_t offset, const void *data, uint32_t size)
{
  writeDataToFlash((uint8_t*)(_flash_address+offset), (uint8_t*)data, size);
}


void OTAInternalFlash::read(uint32_t offset, void *data, uint32_t size)
{
  memcpy(data, _flash_address+offset, size);
}

bool OTAInternalFlash::sameContent(uint32_t offset, const void *data, uint32_t size)
{
  for (uint32_t i=0; i<size;i++)
    if ((((uint8_t*)(_flash_address+offset))[i]!=((const uint8_t*)data)[i]))
      return false;
  return true;
}

void OTAInternalFlash::readAtAbsoluteAddr(uint32_t addr, void *data, uint32_t size)
{
  memcpy(data, (void*)addr, size);
}
