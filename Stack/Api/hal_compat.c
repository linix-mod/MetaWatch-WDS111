/**
 * various functions to deal with flaws and portability issues
 *
 * @author Matthias Ringwald
 */

#include "hal_compat.h"
#include <msp430x54x.h>

/* access far text for MSP430X platform
#if defined(__GNUC__) && (__MSP430X__ > 0)

uint8_t FlashReadByte (uint32_t addr){
    
    uint8_t result;
    uint32_t register sr, flash;
    
    __asm__ __volatile__ (
                          "mov    r2  , %1   \n"
                          "bic    %3  , r2   \n"
                          "nop               \n"
                          "movx.a %4  , %2   \n"
                          "movx.b @%2, %0    \n"
                          "mov    %1 , r2    \n"
                          :"=X"(result),"=r"(sr),"=r"(flash)
                          :"i"(GIE),"m"(addr));
    
    return result;
}

// argument order matches memcpy
void FlashReadBlock(uint8_t *buffer, uint32_t addr,  uint16_t len){
    while (len){
        *buffer++ = FlashReadByte(addr++);
        len--;
    }
}

#endif

*/

/* The following function is used to copy                            */
/* Patch data to a local buffer.  This is done because a SMALL data  */
/* model configuration can't access data past 64k.  The function     */
/* receives as its first parameter the offset from the beginning of  */
/* the Patch data where the copy should begin.  The second parameter */
/* is a pointer to a local buffer where the data is to be moved.  The*/
/* last parameter is the number of bytes that are to be moved.       */
/* * NOTE * The C compiler will not process the information within   */
/*          the 'asm' statements so we must reference the parameters */
/*          passed in to the optimizer will remove variables that are*/
/*          not referenced.  If none of the variables are referenced */
/*          in 'C' code then the entire function will be optimized   */
/*          out.                                                     */
/* * NOTE * This function will not allow an offset of 0xFFFF.        */
void MoveInitBytes(unsigned int offset, unsigned char *dest, unsigned int length)
{
  /* Check to make sure that the parameters passed in appear valid.    */
  if((offset+1) && (dest) && (length))
  {
      asm("LOOP_2564:");
      asm("    MOVX.B   0x10000+0(r12),0(r13)");    /* Move 1 bytes         */
      asm("    ADD.W    #1,r12");                   /* INC offset           */
      asm("    ADD.W    #1,r13");                   /* INC dest             */
      asm("    SUB.W    #1,r14");                   /* DEC length           */
      asm("    CMP.W    #0,r14");                   /* Check (length == 0)  */
      asm("    JNE      LOOP_2564");
  }
  else
  {
    /* Clear the Packet Flag.                                         */
    *dest = 0;
  }
}
