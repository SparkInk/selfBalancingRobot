/*
 * Encoder.c
 *
 *  Created on: Nov. 20, 2021
 *      Author: Ethan Eigenraam
 */
#include "quadEncDecode.h"
#include <msp430.h>

void encoderInit(QUAD_ENC_DECODER * qEDecoder){

    CHAB_DIR &= ~(ENCODER_CHANNELS);     // set P2.0 and 1 to input
    //CHAB_PORT P1IN                      //use this to read the input port

    P1IES &= ~(ENCODER_CHANNELS);          // initialize edge detection to L->H
    P1IFG  = 0;                        // clear interrupt flags by READING the register.
    P1IE  |= (ENCODER_CHANNELS);          // enable P1.2 P1.3 interrupt
    qEDecoder->posCount1 = 0x00;
    qEDecoder->posCount2 = 0x00;

}

#pragma vector = PORT1_VECTOR           // this statement must be left as is. PORT1_VECTOR is defined in msp430.h as are the other interrupt vector addresses.
__interrupt void Port1_ISR1 (void) {   // you can choose any name for the ISR. Port1_ISR2
    static unsigned char prev1B, prev2B, curr1A, curr1B, curr2A, curr2B; // initialize all them's Greg had these Global but its not needed we only care about the count

    //prev1A = curr1A; don't need these since we only check the previous B channels
    prev1B = curr1B;
    //prev2A = curr2A;
    prev2B = curr2B;

    curr1A = (CHAB_PORT & CH1A)>>2;     // Shift 2nd to the 1st bit
    curr1B = (CHAB_PORT & CH1B)>>3;     // Shift 3rd to the 1st bit
    curr2A = (CHAB_PORT & CH2A)>>4;     // Shift 4th to the 1st bit
    curr2B = (CHAB_PORT & CH2B)>>5;     // Shift 5th to the 1st bit

    if((prev1B^curr1A) != 0x00){      // Prev1B XOR with Curr1B
        qEDecoder.posCount1++;
    } else {
        qEDecoder.posCount1--;
    }
    if((prev2B^curr2A) != 0x00){      // Prev1B XOR with Curr1B
        qEDecoder.posCount2++;
    } else {
        qEDecoder.posCount2--;
    }

    switch(__even_in_range(P1IV,4))
      {
      case 0: break;  // if you get here there's a problem
      case 2:
          P1IES ^= CH1A;        // We will automatically be in the correct case, we can switch the edge select right there.
          break;
      case 4:
          P1IES ^= CH1B;        // Same for the next 3
          break;
      case 6:
          P1IES ^= CH2A;
          break;
      case 8:
          P1IES ^= CH2B;
          break;
      default: break;
      }
}

