#ifndef _TWISLAVE_H
#define _TWISLAVE_H

#include <util/twi.h> //enth�lt z.B. die Bezeichnungen f�r die Statuscodes in TWSR
#include <avr/interrupt.h> //dient zur behandlung der Interrupts
#include <stdint.h> //definiert den Datentyp uint8_t
#ifndef _ADC_H
#define _ADC_H


//#ifndef _LCD_H
//#define _LCD_H

//#include "adc.c"
//#include <avr/wdt.h>
/*
Dieses Programm in einer separaten Datei (z.B. twislave.c) abspeichern und in das eigene Programm
einbinden.

Betrieb eines AVRs mit Hardware-TWI-Schnittstelle als Slave. Zu Beginn muss init_twi_slave mit der gew�nschten
Slave-Adresse als Parameter aufgerufen werden. Der Datenaustausch mit dem Master erfolgt �ber die Buffer 
rxbuffer und txbuffer, auf die von Master und Slave zugegriffen werden kann. 
rxbuffer und txbuffer sind globale Variablen (Array aus uint8_t). 
Die Ansteuerung des rxbuffers, in den der Master schreiben kann, erfolgt �hnlich wie bei einem normalen I2C-EEPROM.
Man sendet zun�chst die Bufferposition, an die man schreiben will, und dann die Daten. Die Bufferposition wird 
automatisch hochgez�hlt, sodass man mehrere Datenbytes hintereinander schreiben kann, ohne jedesmal
die Bufferadresse zu schreiben.
Um den txbuffer vom Master aus zu lesen, �bertr�gt man zun�chst in einem Schreibzugriff die gew�nschte Bufferposition und
liest dann nach einem repeated start die Daten aus. Die Bufferposition wird automatisch hochgez�hlt, sodass man mehrere
Datenbytes hintereinander lesen kann, ohne jedesmal die Bufferposition zu schreiben.

Autor: Uwe Gro�e-Wortmann (uwegw)
Status: Testphase, keine Garantie f�r ordnungsgem��e Funktion! 
letze �nderungen: 
23.03.07 Makros f�r TWCR eingef�gt. Abbruch des Sendens, wenn der TXbuffer komplett gesendet wurde. 
24.03.07 verbotene Buffergr��en abgefangen
25.03.07 n�tige externe Bibliotheken eingebunden


Abgefangene Fehlbedienung durch den Master:
- Lesen �ber die Grenze des txbuffers hinaus
- Schreiben �ber die Grenzen des rxbuffers hinaus
- Angabe einer ung�ltigen Schreib/Lese-Adresse
- Lesezuggriff, ohne vorher Leseadresse geschrieben zu haben


 */ 
 



//%%%%%%%% von Benutzer konfigurierbare Einstellungen %%%%%%%%

#define buffer_size 8			//Gr��e der Buffer in Byte (2..128)
#define maxBuffer_Size 128		//Gr�ssere Zahlen sind Befehle des Masters an den Slave
//

//%%%%%%%% Globale Variablen, die vom Hauptprogramm genutzt werden %%%%%%%%

/*Der Buffer, in dem die empfangenen Daten gespeichert werden. Der Slave funktioniert �hnlich  wie ein normales
 Speicher-IC [I2C-EEPROM], man sendet die Adresse, an die man schreiben will, dann die Daten, die interne Speicher-Adresse
 wird dabei automatisch hochgez�hlt*/
volatile uint8_t rxbuffer[buffer_size];

//	Markierung von eingegangenen Bits
volatile uint8_t rxdata=0;
/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
extern volatile uint8_t txbuffer[buffer_size];

/*Datenbuffer fuer Werte vom ADC*/
volatile uint16_t adcbuffer[4];

//%%%%%%%% Funktionen, die vom Hauptprogramm aufgerufen werden k�nnen %%%%%%%%
 
/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter: adr: gew�nschte Slave-Adresse*/
void init_twi_slave (uint8_t adr);
extern uint16_t readKanalOrig(uint8_t derKanal, uint8_t num);
extern uint16_t readKanal(uint8_t derKanal);
void twidelay_ms(unsigned int ms);

extern void lcd_cls(void);
extern void lcd_putint(uint8_t zahl);
extern void lcd_gotoxy(uint8_t x,uint8_t y);
void lcd_puts(const char *s);
void lcd_putint1(uint8_t zahl);
void lcd_clr_line(uint8_t zahl);
void lcd_puthex(uint8_t zahl);

uint8_t lcd_delay=1;
extern volatile uint8_t TWI_Pause;
extern volatile uint8_t twi;
//%%%%%%%% ab hier sind normalerweise keine weiteren �nderungen erforderlich! %%%%%%%%//
//____________________________________________________________________________________//

#include <util/twi.h> //enth�lt z.B. die Bezeichnungen f�r die Statuscodes in TWSR






void twidelay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}


volatile uint8_t buffer_adr; //"Adressregister" f�r den Buffer


#endif
#endif //#ifdef _TWISLAVE_H

//#endif //ifdef _LCD_H
////Ende von twislave.c////

