<<<<<<< HEAD
//TODO: RECTIFY THE RETURN TYPES ON FUNCTIONS
=======
>>>>>>> 5d74284eb131f002567145e10f257a41c821fc38
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL   //SET CPU CLOCK
#endif
//Function declaration
double pulseIn(volatile uint8_t , uint8_t );
double microsecondsToInches(unsigned long );
double microsecondsToCentimeters(unsigned long );
void analogWrite(uint8_t ,uint8_t );
uint8_t analogRead(uint8_t);
void delay(unsigned long);
void delayMicroseconds(unsigned long);
double map(double,double,double,double,double);
double constrain(double,double,double);
void attachIntterupt(int, void *,int);
void (*cAllisr)(void); //function pointer used in ISR()
void pinMode(uint8_t , uint8_t );
static void turnOffPWM(uint8_t );
void digitalWrite(uint8_t , uint8_t );
int digitalRead(uint8_t );
	
void pinMode(uint8_t pIn, uint8_t mOde)
{
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mOde == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mOde == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}


static void turnOffPWM(uint8_t tImer)
{
	switch (tImer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

void digitalWrite(uint8_t pIn, uint8_t vAl)
{
	uint8_t tImer = digitalPinToTimer(pIn);
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (tImer != NOT_ON_TIMER) turnOffPWM(tImer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (vAl == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

int digitalRead(uint8_t pIn)
{
	uint8_t tImer = digitalPinToTimer(pIn);
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (tImer != NOT_ON_TIMER) turnOffPWM(tImer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 14 */
	PC,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
};
unsigned long microsecondsToInches(unsigned long mIcroseconds) 
{
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  return (mIcroseconds*0.00669/ 2);
}

unsigned long microsecondsToCentimeters(unsigned long microseconds) 
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (microseconds*0.17/ 2);
}

double pulseIn(volatile uint8_t pInno, uint8_t vAlue)
{
  TCCR2A = (1 << WGM21)|(1 << COM2A1)|(0 << COM2A0)|(0 << WGM20); //initializing in CTC mode
  TCCR2B = (1 << CS20)|(1<<FOC2A);
  unsigned long mAxloops = 500000;
  unsigned long wIdth = 0;
  // wait for any previous pulse to end
  while ( ((PIND)&&(pInno)) == vAlue)//remove PIND. It should be for every register.
	  {
		if (--mAxloops == 0)
		  return 0;
	  }
  // wait for the pulse to start  
  while ( ((PIND)&&(pInno)) != vAlue)
	  {
		if (--mAxloops == 0)
		return 0;
	  }
  // wait for the pulse to stop
  while ( ((PIND)&&(pInno)) == vAlue)
	  {
		if (++wIdth == mAxloops)
		  return 0;
	  }
  return wIdth;
}

class Serial
{       public:
	void start(void)
{
	UBRR0H = 0; (BaudRate >> 8);
	UBRR0L = BaudRate;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	UCSR0C =(1 << USBS0) | (3 << UCSZ00); //UCSR0C= (1<<URSEL)|(1<<UCSZ00)|(1<<UCSZ01); 0b00001110;
}
unsigned char get(void)
{
	while (!(UCSR0A & (1 << RXC0)))
	;
	return UDR0;
}
void send(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)))
	UDR0=data;
}
void flush(void)
{
	unsigned char dummy;
	while ( UCSR0A & (1 << RXC0))
	dummy= UDR0;
}
};

class Serial1
{
	public:
	void start( unsigned int uBrr){
		/*Set baud rate */
		UBRR1H = (unsigned char)(uBrr>>8);
		UBRR1L = (unsigned char)uBrr;
		/*Enable receiver and transmitter */
		UCSR1B = (1<<RXEN0)|(1<<TXEN0);
	}
	/* Set frame format: 8data, 2stop bit */
	void send( unsigned char data ){
		/* Wait for empty transmit buffer */
		while ( !( UCSR1A & (1<<UDRE)) )
		;
		/* Put data into buffer, sends the data */
		UDR1= data;
		_delay_ms(100);
	}
	unsigned char get( void ){
		/* Wait for data to be received */
		while ( !(UCSR1A & (1<<RXC1)) )
		;
		/* Get and return received data from buffer */
		return UDR1;
	}
	void flush(void){
		unsigned char dUmmy;
		while ( UCSR1A & (1<<RXC1) ) dUmmy = UDR1
		;
	}

	void end(void){
		flush();
		UCSR1B&=0xe7;	//disabling RXEN & TXEN
	}

};
uint16_t analogRead(uint8_t cHannel)
{  
    ADMUX=(1<<REFS0);				//aref=AVcc
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	ADMUX=(1<<REFS0)|(0<<REFS1);
	ADCSRA|=(1<<ADEN);
	ADMUX|=cHannel;  //choose value from 0 to 3 to chose adc pin accordingly
	ADCSRA|=(1<<ADEN);
	ADCSRA|=(1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	return (ADC);
}
void analogWrite(int pIn,int dUtycycle)
{
	//initialize TCCR0 as per requirement, say as follows
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);//initializing timer1
	TCCR1B |=(1<<CS10);
	TCNT1=0;
	if(pIn==1)
	{
		OCR1A=dUtycycle;
	}
	else if(pIn==2)
	{
		OCR1B=dUtycycle;
	}
}
float millis()//float and not int.
{   int x;
	float mIlli;
	mIlli=x*0.16+0.00000625*TCNT0;
	return mIlli;
}

void delay(unsigned long mIllisec)
{
	int i;
	for(i=0;i<mIllisec;i++)
	{
		_delay_ms(1);
	}
	return;
}
void delayMicroseconds(unsigned long mIcrosec)
{
	int i;
	for(i=0;i<mIcrosec;i++)
	{
		_delay_us(1);
	}
	return;
}

double map(double vAlue, double fromLow, double fromHigh, double toLow, double toHigh)
{
	return ((vAlue-fromLow)/abs(fromHigh-fromLow)*abs(toHigh+toLow));
}

double constrain(double nUm,double uPper,double lOwer)
{
	if(nUm<uPper){return uPper;}
	else if(nUm>lOwer){return lOwer;}
	else return nUm;	
}
void attachInterrupt(int intpin, void (*isrfunc)(void), int compare)		//cOmpare:LOW=0,HIGH1,RISING=2,FALLING=3
{
	sei();
	callisr=isrfunc;
	switch(intpin)	  //enabling interrupt pin
	{
		case 0: //enabling INT0
		EIMSK= 1<<INT0;
		switch(compare)
		{
	          	case 2: //RISING
			MCUCR|=(1<<ISC00)|(1<<ISC01);
			break;
			case 3: //FALLING
	    	        MCUCR|=(0<<ISC00)|(1<<ISC01);
			break;
			case 4: //CHANGE
			MCUCR|=(1<<ISC00)|(0<<ISC01);
			break;
         	        default:
			MCUCR|=(0<<ISC00)|(0<<ISC01);	
		}
		break;
		case 1: //enabling INT1
		EIMSK|=1<<INT1;
		switch(compare)
		{
			case 2: //RISING
			MCUCR|=(1<<ISC10)|(1<<ISC11);
			break;
			case 3: //FALLING
			MCUCR|=(0<<ISC10)|(1<<ISC11);
			break;
         	        case 4: //CHANGE
			MCUCR|=(1<<ISC10)|(0<<ISC11);
			break;
        	        default:
			MCUCR|=(0<<ISC00)|(0<<ISC01);
		}
		break;
		default:
		MCUCR|=(0<<ISC00)|(0<<ISC01);	
	}	
}
ISR(INT0_vect)
{
	callisr();
}
ISR(INT1_vect)
{
	callisr();
}
