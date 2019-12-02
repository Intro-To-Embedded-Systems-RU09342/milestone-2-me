#include <msp430.h>

int count = 0;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  ADC10CTL1 = SHS_1 + CONSEQ_2 + INCH_1;    // TA1 trigger sample start
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;
  __enable_interrupt();                     // Enable interrupts.
  TACCR0 = 30;                              // Delay to allow Ref to settle
  TACCTL0 |= CCIE;                          // Compare-mode interrupt.
  TACTL = TASSEL_2 + MC_1;                  // TACLK = SMCLK, Up mode.
  LPM0;                                     // Wait for delay.
  TACCTL0 &= ~CCIE;                         // Disable timer Interrupt
  __disable_interrupt();
  ADC10CTL0 |= ENC;                         // ADC10 Enable
  ADC10AE0 |= 0x02;                         // P1.1 ADC10 option select
  P1DIR |= 0x01;                            // Set P1.0 output
  TACCR0 = 2048-1;                          // PWM Period
  TACCTL1 = OUTMOD_3;                       // TACCR1 set/reset
  TACCR1 = 2047;                            // TACCR1 PWM Duty Cycle
  TACTL = TASSEL_1 + MC_1;                  // ACLK, up mode

  P1SEL |= BIT1 + BIT2;               //P1.1 = RXD P1.2 = TXD
      P1SEL2 |= BIT1 + BIT2;              //P1.1 = RXD P1.2 = TXD
      UCA0CTL1 |= UCSSEL_2;               //smclk
      UCA0BR0 = 104;                      //1MHz 9600 baud rate
      UCA0BR1 = 0;                        //1MHz 9600 baud rate
      UCA0MCTL = UCBRS0;                  //modulation UBRSx = 1
      UCA0CTL1 &= ~UCSWRST;               //initialize usci state machine
      IE2 |= UCA0RXIE;                    //enable RX interrupt

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3 w/ interrupts
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  if (ADC10MEM < 0x155)                     // ADC10MEM = A1 > 0.5V?
    P1OUT &= ~0x01;                         // Clear P1.0 LED off
  else
    P1OUT |= 0x01;                          // Set P1.0 LED on
  if (count >= 15) {
          count = 0;
          while (!(IFG2 & UCA0TXIFG));
          UCA0TXBUF = ADC10MEM * 75 / 512 - 50;
      } else {
          count++;
      }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void ta0_isr(void)
{
  TACTL = 0;
  LPM0_EXIT;                                // Exit LPM0 on return
}
