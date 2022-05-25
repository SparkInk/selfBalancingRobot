/*
 * quadEncDecode.h
 *
 *  Created on: Nov. 20, 2021
 *      Author: Ethan Eigenraam
 */

#ifndef QUADENCDECODE_H_
#define QUADENCDECODE_H_


    #define             CH1A BIT2
    #define             CH1B BIT3
    #define             CH2A BIT4
    #define             CH2B BIT5
    #define ENCODER_CHANNELS CH1A|CH1B|CH2A|CH2B
    #define         CHAB_DIR P1DIR
    #define        CHAB_PORT P1IN

    typedef struct QUAD_ENC_DECODER {
        unsigned char channel1State[4]; // current and previous channel A,B states
        // channelState[0] > CHB_PREV, channelState[1] > CHA_PREV
        // channelState[2] > CHB_CURR, channelState[3] > CHA_CURR
        int posCount1; // 16 bit signed position counter updated every quadrature event
        unsigned char channel2State[4];
        int posCount2;
    } QUAD_ENC_DECODER;

    QUAD_ENC_DECODER qEDecoder;



    void encoderInit(QUAD_ENC_DECODER * qEdecoder);

    // don't know if anything else is needed here


#endif /* QUADENCDECODE_H_ */
