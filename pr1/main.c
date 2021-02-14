#define F_CPU 8000000UL
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

//alerts
char overHeat=0;
//timer work
int mainTimer=0;
int secTimer=0;
char cFireT=200;
char curFireT=0;
char cMenu=20;
char curMenu=0;
char cBut=150;
char curBut=0;
int cPumpE=0;
int curPumpE=0;
unsigned int cPump=65000;
unsigned int curPump=0;
//settings
char candleWork=0;
char motorSpeed=0;
int tempBort=0;
int tempHeat=20;
int tempFire=20;
int modeHeat=0;
char workPump=0;
int freqPumptest=0;
int freqPump1mode=160;
int freqPump2mode=230;
int freqPump3mode=350;
int freqPump4mode=450;
int currentFreqPump=0;
int timeCandle=10;
char working=0;
char heating=0;
//main states
char drawState=0;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
char attempt=0;
char err;
char mustWork=0;
char currentState=0; //1--test 2--motor full(15s) 3--candle(20) 4--pump full(20) 4--test temp 5--work 6--"stop" motor norm(3m), candle(30s)

//0-working 1-set pump 2-motor test
char menuState=0;


// LCD HD44780
#define LCD_RS  PD0
#define LCD_RW  PD1
#define LCD_E   PD2
#define LCD_BL  PD3
#define LCD_D4  PD4
#define LCD_D5  PD5
#define LCD_D6  PD6
#define LCD_D7  PD7

#define LCD_CLEAR       0x01
#define LCD_OFF         0x08
#define LCD_ON          0x0C
#define LCD_RETURN      0x02

//General Master staus codes
#define START       0x08    //START has been transmitted
#define REP_START   0x10
#define MT_DATA_ACK 0x28
#define MT_SLA_ACK  0x18

#define CMD 0 // command
#define DTA 1 // data

//Master Transmitter staus codes
#define MT_ADR_ACK  0x18    //SLA+W has been tramsmitted and ACK received
#define MT_ADR_NACK 0x20    //SLA+W has been tramsmitted and NACK received

#define MT_DATA_ACK 0x28    //Data byte has been tramsmitted and ACK received
#define MT_DATA_NACK 0x30   //Data byte has been tramsmitted and NACK received
#define MT_ARB_LOST 0x38    //Arbitration lost in SLA+W or data bytes

#define WRITE       0x00
#define READ        0x01

#define READ_END    0x01
#define READ_NOEND  0x00

#define ERROR       0x01
#define SUCCESS     0x00

#define I2C_ADR_PCF8574 0x27
// I2C
static uint8_t send_i2c(uint8_t value);
static uint8_t start_i2c(uint8_t d_adr);
static inline void stop_i2c();
static uint8_t write_i2c(uint8_t ADR, uint8_t value);
/// LCD
static int set_cursor(uint8_t r, uint8_t l);
static int  send_lcd(uint8_t value, uint8_t mode);
static int print_lcd(char* str);
static int init_lcd();
static int print_number(int number);
static void init_ports();

int main()
{
	// I2C setup
	TWBR = (F_CPU / 100000UL - 16)/2; // TWI bitrate
	// main loop
	init_ports();
	init_lcd();
	ADC_Init();
	MotorChange();
	send_lcd(LCD_CLEAR,CMD);
	while(1)
	{
		timerSys();
	};
	return 0;
}

void UpdateState()
{
	if (mustWork==1)
	{
		if (currentState==0)
		{
			currentState=1;
		}
	}
	else
	{
		if (currentState==6)
		{
			currentState=7;
		}
	}
}

void State1(int t)//1--test
{
	currentState=2;
	secTimer=0;
}
void State2(int t)//2--motor full(15s)
{
	if (t==1)
	{
		motorSpeed=4;
		MotorChange();
		drawState=1;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
	}
	if (t>15)
	{
		currentState=3;
		secTimer=0;
	}
}
void State3(int t)///3--candle(20)
{
	if (t==1)
	{
		candle_start();
	}
	if (t==20)
	{
		secTimer=0;
		currentState=4;
	}
}
void State4(int t)//4--pump full(20)
{
	if (t==1)
	{
		pump_timer(4);
	}
	if (t==20)
	{
		secTimer=0;
		currentState=5;
	}
}
void State5(int t)//5--test temp
{
	if (t<90)
	{
		GetTempFire();
		if (tempFire>=30)
		{
			secTimer=0;
			currentState=6;
		}
	}
	else
	{
		drawState=5;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
		err="lowT";
		secTimer=0;
		currentState=8;
	}
}
void State6(int t)//6--work
{
	if (t==1)
	{
		drawState=2;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
		candle_end();
	}
	if (overHeat==1)
	{
		drawState=6;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
		secTimer=0;
		workPump=0;
		pump_end();
		currentState=7;
	}
	if (tempFire<20)
	{
		attempt+=1;
	}
	if (tempFire>30)
	{
		attempt=0;
	}
	if (attempt>5)
	{
		drawState=4;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
		secTimer=0;
		//if (modeHeat==0)
		//{
		secTimer=0;
		workPump=0;
		pump_end();
		currentState=7;
		//}
	}
	if (t>10)
	{
		secTimer=0;
		if (motorSpeed!=modeHeat){
			motorSpeed=modeHeat;
			MotorChange();
			pump_timer(modeHeat);
		}
		if (modeHeat==0)
		{	
			secTimer=0;
			workPump=0;
			pump_timer(0);
			pump_end();
			currentState=7;
		}

	}
}
void State7(int t)//7--"stop" motor norm(3m), candle(30s)
{
	if (t==1)
	{
		drawState=3;//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
		workPump=0;
		pump_timer(0);
		pump_end();
		motorSpeed=1;
		MotorChange();
	}
	if (t==60)
	{
		candle_start();
	}
	if (t>90)
	{
		candle_end();
	}
	if (t>180)
	{
		motorSpeed=0;
		MotorChange();
		secTimer=0;
		currentState=0;
		mustWork=0;
	}
}
void State8(int t)
{
	if (t==1)
	{
		workPump=0;
		pump_timer(0);
		pump_end();
		candle_end();
		motorSpeed=1;
		MotorChange();
	}
	if (t==30)
	{
		candle_start();
	}
	if (t>60)
	{
		candle_end();
	}
	if (t>90)
	{
		motorSpeed=0;
		MotorChange();
		secTimer=0;
		currentState=0;
	}
}

void timerSys()
{
	mainTimer+=1;
	if (mainTimer>1000)
	{
		mainTimer=0;
		secTimer+=1;
		switch(currentState)
		{
			case(0):
			secTimer=0;
			break;
			case(1):
			State1(secTimer);
			break;
			case(2):
			State2(secTimer);
			break;
			case(3):
			State3(secTimer);
			break;
			case(4):
			State4(secTimer);
			break;
			case(5):
			State5(secTimer);
			break;
			case(6):
			State6(secTimer);
			break;
			case(7):
			State7(secTimer);
			break;
			case(8):
			State8(secTimer);
			break;
		}

	}
	curFireT+=1;
	curBut+=1;
	curMenu+=1;
	if (curMenu>cMenu)
	{
		curMenu=0;
		Draw();
	}
	if (curFireT>cFireT)
	{
		curFireT=0;
		GetTempFire();
	}
	if (curBut>cBut)
	{
		curBut=0;
		button_take();
	}
	if (workPump>0)
	{
		curPump+=1;
		curPumpE+=0;
		if (curPump>30000)
		{
			curPump=0;
		}
		if (curPump>cPump)
		{
			curPump=0;
			cPumpE=50;
			pump_work();
		}
		if (cPumpE>0)
		{
			curPumpE+=1;
			if (curPumpE>cPumpE)
			{
				cPumpE=0;
				curPumpE=0;
				pump_end();
			}
		}
	}
	_delay_us(1);
}

void GetTempFire()
{
	unsigned int t=0;
	t=ADC_convert();
	float n;
	//n=(float) t/200;
	tempFire=(int)(21*t)/(1024-t);
}

void MotorChange()
{
	switch(motorSpeed)
	{
		case (0):
		PORTD|=0b00000010;
		PORTD&=0b11110011;
		_delay_ms(10);
		break;
		case (1):
		PORTD|=0b00001010;
		PORTD&=0b11111011;
		_delay_ms(10);
		break;
		case (2):
		PORTD|=0b00001110;
		PORTD&=0b11111111;
		_delay_ms(10);
		break;
		case (3):
		PORTD|=0b00001000;
		PORTD&=0b11111001;
		_delay_ms(10);
		break;
		case (4):
		PORTD|=0b00001100;
		PORTD&=0b11111101;
		_delay_ms(10);
		break;
	}
}

void candle_start()
{
	PORTD&=0b11101111;
	candleWork=1;
}
void candle_end()
{
	PORTD|=0b00010000;
	_delay_ms(5);
}
void pump_timer(char a)
{
	//cPump=60000/freqPumptest;
	switch (a)
	{
		case 0:
		workPump=0;
		cPump=60000;
		break;
		case 1:
		workPump=1;
		cPump=60000/freqPump1mode;
		break;
		case 2:
		workPump=1;
		cPump=60000/freqPump2mode;
		break;
		case 3:
		workPump=1;
		cPump=60000/freqPump3mode;
		break;
		case 4:
		workPump=1;
		cPump=60000/freqPump4mode;
		break;
	}
}
void pump_work()
{
	PORTD&=0b11111110;
}
void pump_end()
{

	PORTD|=0b00000001;
	if (PINB&0b00000001)
	{
		overHeat=0;
	}else
	{
		overHeat=1;
	}
}

void button_take()
{
	if (!(PINB&0b00000010))
	{
		button_work("l");
	}
	if (!(PINB&0b00000100))
	{
		button_work("r");
	}
	if (!(PINB&0b00001000))
	{
		if (!(PINB&0b00010000))
		{
			button_work("f");
		}
		else
		{
			button_work("n");
		}
	}
	else
	{
		if (!(PINB&0b00010000))
		{
			button_work("s");
		}
	}
}

void button_work(char a)
{
	switch(menuState){
		case(0):
		ButtonMenu(a);
		break;
		case(1):
		ButtonSet1Mode(a);
		break;
		case(2):
		ButtonSet2Mode(a);
		break;
		case(3):
		ButtonSet3Mode(a);
		break;
		case(4):
		ButtonSet4Mode(a);
		break;
		case(5):
		ButtonTestMotor(a);
		break;
		case(6):
		ButtonTestPump(a);
		break;
		case(7):
		ButtonTestCandle(a);
		break;
		case(8):
		ButtonTestFireTemp(a);
		break;
	}
}
void ButtonMenu(char a)
{
	if (mustWork>0)
	{
		if (a=="n")
		{
			modeHeat-=1;
			if (modeHeat<1)
			{
				modeHeat=0;
				mustWork=0;

			}
		}
		if (a=="s")
		{
			modeHeat+=1;
			if (modeHeat>4)
			{
				modeHeat=4;
			}
		}
		UpdateState();
	}
	if (currentState==0)
	{
		if (a=="n")
		{
			menuState+=1;
			send_lcd(LCD_CLEAR,CMD);
		}
	}
	if (a=="l")
	{
		tempHeat-=1;
	}
	if (a=="r")
	{
		tempHeat+=1;
	}
	if (a=="f")
	{
		if (mustWork==0)
		{
			mustWork=1;
			modeHeat=4;
			UpdateState();
		}
	}

}
void ButtonSet1Mode(char a)
{
	if (a=="l")
	{
		freqPump1mode-=10;
		if (freqPump1mode<10)
		{
			freqPump1mode=10;
		}
	}
	if (a=="r")
	{
		freqPump1mode+=10;
		if (freqPump1mode>999)
		{
			freqPump1mode=990;
		}
	}
	if (a=="n")
	{
		menuState+=1;
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="s")
	{
		menuState=0;
		send_lcd(LCD_CLEAR,CMD);
	}
}
void ButtonSet2Mode(char a)
{
	if (a=="l")
	{
		freqPump2mode-=10;
		if (freqPump2mode<10)
		{
			freqPump2mode=10;
		}
	}
	if (a=="r")
	{
		freqPump2mode+=10;
		if (freqPump2mode>999)
		{
			freqPump2mode=990;
		}
	}
	if (a=="n")
	{
		menuState+=1;
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="s")
	{
		menuState=0;
		send_lcd(LCD_CLEAR,CMD);
	}
}
void ButtonSet3Mode(char a)
{
	if (a=="l")
	{
		freqPump3mode-=10;
		if (freqPump3mode<10)
		{
			freqPump3mode=10;
		}
	}
	if (a=="r")
	{
		freqPump3mode+=10;
		if (freqPump3mode>999)
		{
			freqPump3mode=990;
		}
	}
	if (a=="n")
	{
		menuState+=1;
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="s")
	{
		menuState=0;
		send_lcd(LCD_CLEAR,CMD);
	}
}
void ButtonSet4Mode(char a)
{
	if (a=="l")
	{
		freqPump4mode-=10;
		if (freqPump4mode<10)
		{
			freqPump4mode=10;
		}
	}
	if (a=="r")
	{
		freqPump4mode+=10;
		if (freqPump4mode>999)
		{
			freqPump4mode=990;
		}
	}
	if (a=="n")
	{
		menuState+=1;
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="s")
	{
		menuState=0;
		send_lcd(LCD_CLEAR,CMD);
	}
}
void ButtonTestMotor(char a)
{
	if (a=="s")
	{
		menuState=0;
		motorSpeed=0;
		MotorChange();
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="n")
	{
		menuState+=1;
		motorSpeed=0;
		MotorChange();
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="l")
	{
		if (motorSpeed>0)
		{
			motorSpeed-=1;
		}
	}
	if (a=="r")
	{
		if (motorSpeed<4)
		{
			motorSpeed+=1;
		}
	}
	MotorChange();
}
void ButtonTestPump(char a)
{
	if (a=="s")
	{
		menuState=0;
		freqPumptest=0;
		workPump=0;
		pump_end();
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="n")
	{
		menuState+=1;
		freqPumptest=0;
		workPump=0;
		pump_end();
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="l")
	{
		freqPumptest-=10;
		if (freqPumptest<10)
		{
			freqPumptest=0;
			workPump=0;
			pump_end();
		}
	}
	if (a=="r")
	{
		freqPumptest+=10;
		workPump=1;
		if (freqPumptest>999)
		{
			freqPumptest=990;
		}
	}
	if (workPump==1)
	{
		cPump=60000/freqPumptest;
	}
}
void ButtonTestCandle(char a)
{
	if (a=="s")
	{
		menuState=0;
		candle_end();
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="n")
	{
		menuState+=1;
		candle_end();
		send_lcd(LCD_CLEAR,CMD);
	}
	if (a=="l")
	{
		candle_end();
	}
	if (a=="r")
	{
		candle_start();
	}

}
void ButtonTestFireTemp(char a)
{
	if (a=="s")
	{
		menuState=0;
		send_lcd(LCD_CLEAR,CMD);
	}
	/*
	if (a=="n")
	{
	menuState+=1;
	send_lcd(LCD_CLEAR,CMD);
	}
	*/
}

void Draw()
{
	switch(menuState)
	{
		case(0):
		DrawMenu();
		break;
		case(1):
		DrawSet1Mode();
		break;
		case(2):
		DrawSet2Mode();
		break;
		case(3):
		DrawSet3Mode();
		break;
		case(4):
		DrawSet4Mode();
		break;
		case(5):
		DrawTestMotor();
		break;
		case(6):
		DrawTestPump();
		break;
		case(7):
		DrawTestCandle();
		break;
		case(8):
		DrawTestFireTemp();
		break;
	}
}
void DrawMenu()
{
	set_cursor(0,1);
	print_lcd("Temp:");
	set_cursor(5,1);
	print_number(tempBort);
	set_cursor(8,1);
	print_lcd("Heat:");
	set_cursor(13,1);
	print_number(tempHeat);
	set_cursor(0,2);
	print_lcd("Mode:");
	set_cursor(5,2);
	print_number(modeHeat);
	set_cursor(9,2);
	print_number(currentState);
	set_cursor(11,2);
	switch (drawState)//0-idle 1-strt 2-work 3-stop 4-lowT 5-noFr 6-OvrH
	{
		case 0:
		print_lcd("Idle");
		break;
		case 1:
		print_lcd("Strt");
		break;
		case 2:
		print_lcd("Work");
		break;
		case 3:
		print_lcd("Stop");
		break;
		case 4:
		print_lcd("LowT");
		break;
		case 5:
		print_lcd("NoFr");
		break;
		case 6:
		print_lcd("OverH");
		break;
	}
}
void DrawSet1Mode()
{
	set_cursor(0,1);
	print_lcd("Pump 1 mode");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	print_number(freqPump1mode);
}
void DrawSet2Mode()
{
	set_cursor(0,1);
	print_lcd("Pump 2 mode");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	print_number(freqPump2mode);
}
void DrawSet3Mode()
{
	set_cursor(0,1);
	print_lcd("Pump 3 mode");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	print_number(freqPump3mode);
}
void DrawSet4Mode()
{
	set_cursor(0,1);
	print_lcd("Pump 4 mode");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	print_number(freqPump4mode);
}
void DrawTestMotor()
{
	set_cursor(0,1);
	print_lcd("Motor Test");
	set_cursor(0,2);
	print_lcd("Speed:");
	set_cursor(6,2);
	print_number(motorSpeed);
}
void DrawTestPump()
{
	set_cursor(0,1);
	print_lcd("Pump test!");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	print_number(freqPumptest);
}
void DrawTestCandle()
{
	set_cursor(0,1);
	print_lcd("Candle test!");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	if (candleWork==1)
	{
		print_lcd("ON");
	}
	else
	{
		print_lcd("OFF");
	}
}
void DrawTestFireTemp()
{
	set_cursor(0,0);
	print_lcd("Test work temp");
	set_cursor(0,2);
	print_lcd("Value:");
	set_cursor(6,2);
	print_number(tempFire);
}

void init_ports()
{
	DDRB=0x00;
	PORTB = 0b00011110;
	DDRD=0xFF;
	PORTD = 0b00010001;
}

void ADC_Init(void)
{
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADMUX |= (1<<REFS1)|(1<<REFS0);
}
int ADC_convert ()
{
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADSC)));
	return ( int) ADC;
}

static int init_lcd()
{
	uint8_t LCD;
	// 4bit mode
	LCD=(1<<LCD_D5)|(1<<LCD_E); write_i2c((I2C_ADR_PCF8574<<1), LCD);
	LCD&=~(1<<LCD_E); write_i2c((I2C_ADR_PCF8574<<1), LCD);

	_delay_ms(50);

	send_lcd(0x28,CMD); // mode: 4bit, 2 lines
	send_lcd(LCD_OFF,CMD);
	send_lcd(LCD_CLEAR,CMD);
	send_lcd(0x06,CMD); // seek mode: right
	//send_lcd(0x0f,CMD); // display ON, Blink ON, Position ON
	send_lcd(0x0c,CMD); // display ON, Blink OFF, Position OFF
	return 0;
}
static int set_cursor(uint8_t r, uint8_t l)
{
	if(l==2){
		send_lcd(0x80|0x40+r,CMD);
		_delay_us(20);
		}else{
		send_lcd(0x80|r,CMD);
		_delay_us(20);
	}
}
static int print_lcd(char* str)
{
	uint8_t i=0;
	while(str[i] !=0 && i<255)
	send_lcd(str[i++],DTA);

	return i;
}
static int  send_lcd(uint8_t value, uint8_t mode)
{
	uint8_t LCD;

	LCD=(value & 0xF0)|(mode<<LCD_RS)|(1<<LCD_E)|(1<<LCD_BL); write_i2c((I2C_ADR_PCF8574<<1), LCD);
	LCD&=~(1<<LCD_E); write_i2c((I2C_ADR_PCF8574<<1), LCD);
	_delay_us(10);

	LCD=(value<<4)|(mode<<LCD_RS)|(1<<LCD_E)|(1<<LCD_BL); write_i2c((I2C_ADR_PCF8574<<1), LCD);
	LCD&=~(1<<LCD_E); write_i2c((I2C_ADR_PCF8574<<1), LCD);

	if (value == 0x01)
	_delay_ms(50);
	else
	_delay_us(50);

	return 0;
}
static int print_number(int number){  // display hex number on LCD followed by a space
	static const uint8_t symbol[16] ="0123456789ABCDEF";
	if (number>99)
	{
		int n1=number/100;
		int n2=(number-n1*100)/10;
		int n3=number-n1*100-n2*10;
		send_lcd(symbol[n1],DTA);
		send_lcd(symbol[n2],DTA);
		send_lcd(symbol[n3],DTA);
	}
	else
	{
		int n1=number/10;
		int n2=number-n1*10;
		send_lcd(symbol[n1],DTA);
		send_lcd(symbol[n2],DTA);
	}
	return 0;
}
static uint8_t write_i2c(uint8_t ADR, uint8_t value) {
	uint8_t ret;
	if (start_i2c(ADR) != ERROR)
	{
		ret=send_i2c(value);
		stop_i2c();
		} else {
		stop_i2c();
		ret=ERROR;
	}
	return ret;
}
uint8_t send_i2c(uint8_t value)
{
	TWDR = value;
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));
	// check value of TWI Status Register. Mask prescaler bits.

	value = TWSR & 0xF8;
	return (value == MT_SLA_ACK || value == MT_DATA_ACK) ? SUCCESS : ERROR;
}
static uint8_t start_i2c(uint8_t d_adr)
{
	TWCR=(1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // START
	while (!(TWCR & (1<<TWINT)));

	uint8_t twst = (TWSR & 0xF8); // check value of TWI Status Register. Mask prescaler bits.

	return ((twst != START) && (twst != REP_START)) ? ERROR : send_i2c(d_adr);
};
static inline void stop_i2c()
{
	TWCR=(1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}