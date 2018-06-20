/****************************************************************************
 Title	:   Serial library
 company: www.digitlabz.com
 File:	    lcd
 Software:  AVR-GCC 3.3 
 Target:    any AVR device

 DESCRIPTION
       
*****************************************************************************/


#ifndef SERIAL_H
#define SERIAL_H

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#define RECEIVE_BUFF_SIZE 300
#define USART_RXC_VECT USART_RXC_vect



volatile char URBuff[RECEIVE_BUFF_SIZE];	
volatile int8_t UQFront;
volatile int8_t UQEnd;


void serial_init(uint16_t baudrate)
{
uint16_t ubrrvalue = ((F_CPU+(baudrate*8L))/(baudrate*16L)-1);
	
	
	
	UQFront=UQEnd=-1;

	
	UBRRH=(unsigned char)(ubrrvalue>>8);
	UBRRL=(unsigned char)ubrrvalue;
	UCSRC=(1<<URSEL)|(3<<UCSZ0);
	UCSRB=(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	sei();

}

void serial_finit(uint16_t baudrate,uint8_t xtal)
{
uint16_t ubrrvalue = (((xtal*1000000)+(baudrate*8L))/(baudrate*16L)-1);
	
	UQFront=UQEnd=-1;

	
	UBRRH=(unsigned char)(ubrrvalue>>8);
	UBRRL=(unsigned char)ubrrvalue;
	UCSRC=(1<<URSEL)|(3<<UCSZ0);
	UCSRB=(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	sei();

}

unsigned char serial_read()
{
	char data;
	
	
	if(UQFront==-1)
		return 0;
	
	data=URBuff[UQFront];
	
	if(UQFront==UQEnd)
	{
	
	UQFront=UQEnd=-1;
	}
	else
	{
		UQFront++;

		if(UQFront==RECEIVE_BUFF_SIZE)
		UQFront=0;
	}

	return data;
}

void serial_char(char data)
{
	
	while(!(UCSRA & (1<<UDRE)));

	
	UDR=data;
}



void serial_numl(long num)
{
	char buffer[7];

 ltoa( num , buffer, 10);
    serial_string(buffer);
}

void serial_num(int num)
{
	char buffer[7];

 itoa( num , buffer, 10);
    serial_string(buffer);
}



void serial_string(char *str)
{
	while((*str)!='\0')
	{
		serial_char(*str);
		str++;
	}
}


void serial_string_P(const char *progmem_s)
{

register char c;

	
	
	while ( (c = pgm_read_byte(progmem_s++))!='\0' ) {
       serial_char(c);
		//progmem_s++;
    }
	
}


void s_readbuffer(void *buff,uint16_t len)
{
	uint16_t i;
	for(i=0;i<len;i++)
	{
		((char*)buff)[i]=serial_read();
	}
}

uint8_t serial_available()
{
	if(UQFront==-1) return 0;
	if(UQFront<UQEnd)
		return(UQEnd-UQFront+1);
	else if(UQFront>UQEnd)
		return (RECEIVE_BUFF_SIZE-UQFront+UQEnd+1);
	else
		return 1;
}





void s_clearbuffer()
{
	while(serial_available()>0)
	{
		serial_read();
	}
}







char serial_nreadw(char *msg,unsigned char length)
{

   while(serial_available()<length);		
	s_readbuffer(msg,length);
	return 1;
	
}


char serial_nread(char *msg,unsigned char length)
{

  if(serial_available()>=length)
{	
	s_readbuffer(msg,length);
	return 1;
	
}
else{
       msg[0]='\0';
	  return 0;
}


	
}






ISR(USART_RXC_VECT)
{
	
	char data=UDR;

	

	if(((UQEnd==RECEIVE_BUFF_SIZE-1) && UQFront==0) || ((UQEnd+1)==UQFront))
	{
		
		UQFront++;
		if(UQFront==RECEIVE_BUFF_SIZE) UQFront=0;
	}
	

	if(UQEnd==RECEIVE_BUFF_SIZE-1)
		UQEnd=0;
	else
		UQEnd++;


	URBuff[UQEnd]=data;

	if(UQFront==-1) UQFront=0;

}



void urclear()
{
   for(int i=0;i<128;i++)
   {
      URBuff[i]='\0';
   } 

}









#endif
