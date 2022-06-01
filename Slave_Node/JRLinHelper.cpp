#include "JRLinHelper.h"


/* Lin defines its checksum as an inverted 8 bit sum with carry */
uint8_t JRLINHelper::dataChecksum(const uint8_t* message, char nBytes, uint16_t sum)
{
  while (nBytes-- > 0) sum += *(message++);

  // Add the carry
  while (sum >> 8) // In case adding the carry causes another carry
    sum = (sum & 255) + (sum >> 8);
    
  return (~sum);
}

/* Create the Lin ID parity */
#define BIT(data,shift) ((addr&(1<<shift))>>shift)

uint8_t JRLINHelper::addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr, 0) ^ BIT(addr, 1) ^ BIT(addr, 2) ^ BIT(addr, 4);
  uint8_t p1 = ~(BIT(addr, 1) ^ BIT(addr, 3) ^ BIT(addr, 4) ^ BIT(addr, 5));

  return (p0 | (p1 << 1)) << 6;
}
