#include <msp430.h>

int temp = 0;
int count = 0;

int main(void) {
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    ADC10CTL1 = SHS_1 + CONSEQ_2 + INCH_0;    // TA1 trigger sample start
    ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;
    __enable_interrupt();                     // Enable interrupts.

    TA0CCR0 = 30;                              // Delay to allow Ref to settle
    TA0CCTL0 |= CCIE;                          // Compare-mode interrupt.
    TA0CTL = TASSEL_2 + MC_1;                  // TACLK = SMCLK, Up mode.
    LPM0;                                     // Wait for delay.
    TA0CCTL0 &= ~CCIE;                         // Disable timer Interrupt
    __disable_interrupt();

    ADC10CTL0 |= ENC;                         // ADC10 Enable
    ADC10AE0 |= BIT0;                         // P1.1 ADC10 option select
    TA0CCR0 = 2048-1;                          // PWM Period
    TA0CCTL1 = OUTMOD_3;                       // TACCR1 set/reset
    TA0CCR1 = 2047;                            // TACCR1 PWM Duty Cycle
    TA0CTL = TASSEL_1 + MC_1;                  // ACLK, up mode

    P1SEL |= BIT1 + BIT2;               //P1.1 = RXD P1.2 = TXD
    P1SEL2 |= BIT1 + BIT2;              //P1.1 = RXD P1.2 = TXD
    UCA0CTL1 |= UCSSEL_2;               //smclk
    UCA0BR0 = 104;                      //1MHz 9600 baud rate
    UCA0BR1 = 0;                        //1MHz 9600 baud rate
    UCA0MCTL = UCBRS0;                  //modulation UBRSx = 1
    UCA0CTL1 &= ~UCSWRST;               //initialize usci state machine
    IE2 |= UCA0RXIE;                    //enable RX interrupt

    P2DIR |= BIT1;               //set P2.1 to output
    P2SEL |= BIT1;               //enable pwm for P2.1
    TA1CTL |= TASSEL_2 + MC_1;          //set smclk, up mode
    TA1CCTL1 |= OUTMOD_7;               //set/reset output
    TA1CCR0 = 255;                      //pwm period
    TA1CCR1 = 0;                        //initialize green pwm*/

    __bis_SR_register(GIE); //low power mode and interrupt enabled

    /*while (1) {
        int i;
        for (i = 0; i < 5000; i++);
        if (temp >= ADC10MEM * 75 / 512 - 50) {
            if (TA1CCR1 > 0)
                TA1CCR1 -= 1;
        } else {
            if (TA1CCR1 < 255)
                TA1CCR1 += 1;
        }
    }*/
}

#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    if (temp >= ADC10MEM * 75 / 512 - 50) {
        if (TA1CCR1 > 0)
            TA1CCR1 -= 1;
    } else {
        if (TA1CCR1 < 255)
            TA1CCR1 += 1;
    }
    if (count >= 15) {
        count = 0;
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = ADC10MEM * 75 / 512 - 50;
    } else {
        count++;
    }
}

#pragma vector = USCIAB0RX_VECTOR       //interrupt routine
__interrupt void RXInterrupt(void) {
    while (!(IFG2 & UCA0TXIFG));        //wait until a byte is ready, is USCI_A0 TX buffer ready?
    temp = UCA0RXBUF;
    UCA0TXBUF = ADC10MEM * 75 / 512 - 50;
    IFG2 = 0x00;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void ta0_isr(void) {
    TA0CTL = 0;
    LPM0_EXIT;                                // Exit LPM0 on return
}
