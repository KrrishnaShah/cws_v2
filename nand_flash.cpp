#include "pins_arduino.h"
#include "WinbondW25N.h"
#include <Arduino.h>
#include <string.h>
#include "pinmap.h"
#include "nand_flash.h"
#include "spi_api.h"
#include "trace.h"

static W25N flash = W25N();

void nand_flash_init(void)
{
  if (flash.begin(D7))
  {
    TRACE("------> Flash init successful.");
  }
  else
  {
    TRACE("Flash init Failed");
  }
}
