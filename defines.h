//
//  defines.h
//  TWI_Slave
//
//  Created by Ruedi Heimlicher on 19.09.2016.
//
//

#ifndef defines_h
#define defines_h
//defines.h

#define PWM_FAKTOR   2.0

#define K_DELTA   .10

#define K_PROP    3.0
#define K_PROP_RED   1.0

#define K_INT     0.4

#define K_DIFF    0.1

#define K_WINDUP_HI  20
#define K_WINDUP_LO  -10

#define TIMER2_ENDWERT 0x9E // OV 20ms

#define TIMER2_COMPA 0x9E // OV 20ms

#define TIMER2_PWM_INTERVALL 0xFF // Paketlaenge


#define OSZIPORT				PORTB
#define OSZIDDR            DDRB
#define OSZIA					1
#define OSZILO OSZIPORT &= ~(1<<OSZIA)
#define OSZIHI OSZIPORT |= (1<<OSZIA)
#define OSZITOGG OSZIPORT ^= (1<<OSZIA)
// Define fuer Slave:
#define TOPLED_PIN			1 // Blinkt waehrend heizen, voll wenn Temp erreicht
#define ADCPORT PORTC
#define ADCDDR    DDRC
#define ADC_SOLL_PIN  1 // von Einstellung Temperatur
#define ADC_IST_PIN  0 // von PTC
#define OUTPORT   PORTD
#define OUTDDR    DDRD
#define PWM_OUT_PIN    3
#define PWM_ON  0
#define PWM_ADC  1
#define PID_FIRST_RUN  2

//avr-size  --mcu=attiny85 -C Laminator.elf

#endif /* defines_h */
