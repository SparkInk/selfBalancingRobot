/*
 * hcsr04.h
 *
 *  Created on: Mar 15, 2022
 *      Author: iumrikhin
 */

#ifndef HCSR04_H_
#define HCSR04_H_

    // Macros
    #define INIT_15us   P1OUT |= BIT5; __delay_cycles(150); P1OUT &= ~BIT5;

    // functions' definitions
    void timerA0Init(void);

    void pin14Init(void);

    unsigned int distance(unsigned int time);

    void dispCnsl(signed int computedSpeed);
#endif /* HCSR04_H_ */
