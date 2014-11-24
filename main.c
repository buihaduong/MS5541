#include <msp430g2553.h>

/*
 * main.c
 */

//	MSP430		MS5541
//Pin 1.1 UCA0SOMI	-	Pin 8 DOUT
//Pin 1.2 UCA0SIMO 	-	Pin 7 DIN
//Pin 1.4 UCA0CLK	-	Pin 1 SCKL
//Pin 1.0 ACLK		-	Pin 6 MCKL
//Pin VCC			-	Pin 5 VDD
//Pin GND			-	Pin 2 VSS
#define SPI_MODE_0 (UCCKPH)			    /* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)                 	/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)    /* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)			    /* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

void spiSetDataMode(int mode) {
	UCA0CTL1 |= UCSWRST;        // go into reset state
	switch (mode) {
	case 0: /* SPI_MODE0 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_0;
		break;
	case 1: /* SPI_MODE1 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_1;
		break;
	case 2: /* SPI_MODE2 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_2;
		break;
	case 4: /* SPI_MODE3 */
		UCA0CTL0 = (UCA0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_3;
		break;
	default:
		break;
	}
	UCA0CTL1 &= ~UCSWRST;       // release for operation
}

int spiSendByte(int data) {
//	while (!(IFG2 & UCA0TXIFG))
//		;   // USCI_A0 TX buffer ready?
	UCA0TXBUF = data;              // Send 0xAA over SPI to Slave
	while ((UCBUSY & UCA0STAT))
		;   // USCI_A0 RX Received?
	return UCA0RXBUF;       // Store received data
}

void resetsensor(void) {
	spiSetDataMode(0);
	spiSendByte(0x15);
	spiSendByte(0x55);
	spiSendByte(0x40);
}

unsigned long result1 = 0;
unsigned long inbyte1 = 0;
unsigned long result2 = 0;
unsigned long inbyte2 = 0;
unsigned long result3 = 0;
unsigned long inbyte3 = 0;
unsigned long result4 = 0;
unsigned long inbyte4 = 0;

long c1 = 0;
long c2 = 0;
long c3 = 0;
long c4 = 0;
long c5 = 0;
long c6 = 0;

unsigned long tempMSB = 0; //first byte of value
unsigned long tempLSB = 0; //last byte of value
unsigned long D2 = 0;

unsigned long presMSB = 0; //first byte of value
unsigned long presLSB = 0; //last byte of value
unsigned long D1 = 0;

long UT1;
long dT;
long TEMP;
long OFF;
long SENS;
long PCOMP;
float TEMPREAL;

//2nd order compensation only for T > 0°C
long dT2;
float TEMPCOMP;

void readCalibration() {
	resetsensor();

	spiSendByte(0x1D);
	spiSendByte(0x50);
	spiSetDataMode(1);
	result1 = spiSendByte(0x00);
	result1 = result1 << 8;
	inbyte1 = spiSendByte(0x00);
	result1 = result1 | inbyte1;

	resetsensor();

	spiSendByte(0x1D);
	spiSendByte(0x60);
	spiSetDataMode(1);
	result2 = spiSendByte(0x00);
	result2 = result2 << 8;
	inbyte2 = spiSendByte(0x00);
	result2 = result2 | inbyte2;

	resetsensor();

	spiSendByte(0x1D);
	spiSendByte(0x90);
	spiSetDataMode(1);
	result3 = spiSendByte(0x00);
	result3 = result3 << 8;
	inbyte3 = spiSendByte(0x00);
	result3 = result3 | inbyte3;

	resetsensor();

	spiSendByte(0x1D);
	spiSendByte(0xA0);
	spiSetDataMode(1);
	result4 = spiSendByte(0x00);
	result4 = result4 << 8;
	inbyte4 = spiSendByte(0x00);
	result4 = result4 | inbyte4;

	c1 = result1 >> 3 & 0x1FFF;
	c2 = ((result1 & 0x07) << 10) | ((result2 >> 6) & 0x03FF);
	c3 = (result3 >> 6) & 0x03FF;
	c4 = (result4 >> 7) & 0x07FF;
	c5 = ((result2 & 0x003F) << 6) | (result3 & 0x003F);
	c6 = result4 & 0x007F;
}

void readData() {
	resetsensor();

	spiSendByte(0x0F);
	spiSendByte(0x20);
	_delay_cycles(36000);
	spiSetDataMode(1);
	tempMSB = spiSendByte(0x00);
	tempMSB = tempMSB << 8;
	tempLSB = spiSendByte(0x00);
	D2 = tempMSB | tempLSB;

	resetsensor();

	spiSendByte(0x0F);
	spiSendByte(0x40);
	_delay_cycles(36000);
	spiSetDataMode(1);
	presMSB = spiSendByte(0x00);
	presMSB = presMSB << 8;
	presLSB = spiSendByte(0x00);
	D1 = presMSB | presLSB;

	UT1 = (c5 << 3) + 10000;
	dT = D2 - UT1;
	TEMP = 200 + ((dT * (c6 + 100)) >> 11);
	OFF = c2 + (((c4 - 250) * dT) >> 12) + 10000;
	SENS = (c1 / 2) + (((c3 + 200) * dT) >> 13) + 3000;
	PCOMP = (SENS * (D1 - OFF) >> 12) + 1000;
	TEMPREAL = TEMP / 10;

	//2nd order compensation only for T > 0°C
	dT2 = dT - ((dT >> 7 * dT >> 7) >> 3);
	TEMPCOMP = (200 + (dT2 * (c6 + 100) >> 11)) / 10;
}

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	P1SEL = BIT1 | BIT2 | BIT4;
	P1SEL2 = BIT1 | BIT2 | BIT4;

	UCA0CTL1 = UCSWRST;
	UCA0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0BR0 |= 0x02;                          // /2
	UCA0BR1 = 0;                              //
	UCA0MCTL = 0;                             // No modulation
	UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

	BCSCTL1 |= DIVA_0;
	BCSCTL3 |= XCAP_3;
	P1DIR |= BIT0;
	P1SEL |= BIT0;

	readCalibration();

	while (1)
	{
		readData();
	}
}
