#include<avr/io.h>
#include<util/delay.h>


#include "serial.h"
#include "lcd.h"


void beep()
{
	write(porta,3,h);
	_delay_ms(300);
	write(porta,3,l);
}

int main()
{

	beep();
	char url[120],response[10];

	serial_init(9600);
	serial_select(3);

	lcd_init();

	lcd_string_P(PSTR("GPS Tracker"));

		set_timeout(30);	//Auto Reset If code gets blocked for more than 30 seconds

	gsm_init();
	
		reset_timeout();

	gprs_init();   
	
		reset_timeout();

	agps_init();

	//gps_init();
	
		reset_timeout();

	gprs_connect();
	
		reset_timeout();

	char lat[10];
	char lon[10];
	char speed[10];

	while (1)
	{
           		

		if(gps_fix()==1)
		{
		
			reset_timeout();
			
			gps_getcoordinates(lat,lon,speed);
		  
			lcd_clear();
			lcd_string(lat);
			lcd_gotoxy(0,1);
			lcd_string(lon);
			
			reset_timeout();
		
			sprintf(url,"api.clobous.com/Zck77/track?&lat=%s&long=%s&speed=%s",lat,lon,speed);
		 
			http_get(url,response);
			
			reset_timeout();
			
				lcd_clear();
				lcd_string(response);
				_delay_ms(1000);
		}
		
		else{
				lcd_clear();
				lcd_string("NO GPS FIX");
				_delay_ms(50);
			 }


		_delay_ms(500);



	}

}
