#include "lcd.h"

Uart2 &u2 = Uart2::getInstance();
char lcd_init=0xFE;
char lcd_clear=0x51;
char cursor_right=0x56;
char cursor_blink = 0x4B;
char cursor_home = 0x46;
char set_cursor = 0x45;
char cursor_backspace = 0x4E;
char sensorBuffer[25];
char motorBuffer[20];
int cursorPosition;
int dPlace;

//int front_data = 30;
//int left_data = 40;
//int right_data = 50;
//int back_data = 60;
//char dir_data[8] = "Forward";
//int rpm_data = 250;

string thunderbolt_string="THUNDERBOLT";

enum NEW_LINE
{
	LINE_ONE = 0x00,
	LINE_ONE_END = 0x13,
	LINE_TWO = 0x40,
	LINE_TWO_END = 0x53,
	LINE_THREE = 0x14,
	LINE_FOUR = 0x54
};

void shiftNewLine(NEW_LINE line)
{
	u2.putChar(lcd_init);
	u2.putChar(set_cursor);
	u2.putChar(line);
}

void shiftCursorRight(int number)
{
	for(int i=0;i<number;i++)
	{
		u2.putChar(lcd_init);
		u2.putChar(cursor_right);
	}
}

void shiftCursorLeft(int number)
{
	for(int i=0;i<number;i++)
	{
		u2.putChar(lcd_init);
		u2.putChar(0x55);
	}
}

void displayString(string data)
{
	for(uint8_t i=0; i<data.length(); i++)
	{
	  u2.putChar(data[i]);
	}
}

void scroll(string data)
{
	cursorPosition = 40 - data.length();
	for(int i = cursorPosition; i <= cursorPosition + 20; i++)
	{
		clearLineTwo();
		shiftNewLine(LINE_TWO);
		shiftCursorRight(i);
		displayString(sensorBuffer);
		delay_ms(500);
	}

//	for(uint8_t i=0; i<data.length(); i++)
//	{
//		u2.putChar(data[i]);
//	}

	//u2.putChar(lcd_init);

//	if(cursorPosition > 19)
//	{
//		shiftNewLine(LINE_TWO);
//	}
	//shiftNewLine(LINE_TWO);

	//cursorPosition++;
}

void clearLineTwo(void)
{
	shiftNewLine(LINE_TWO_END);
	for(int i = 19; i > 0; i--)
	{
		u2.putChar(lcd_init);
		u2.putChar(cursor_backspace);
	}
}

void processLineOne(void)
{
	shiftNewLine(LINE_ONE);
	displayString(thunderbolt_string);
}

void processLineTwo(int leftVal, int frontVal, int rightVal, int backVal)
{
	//shiftNewLine(LINE_TWO);
//	clearLineTwo();
//	shiftNewLine(LINE_TWO);
	sprintf(sensorBuffer, "L:%d F:%d R:%d B:%d",leftVal, frontVal, rightVal, backVal);
//	if(cursorPosition > 18)
//	{
//		cursorPosition = 0;
//	}
//	else
//	{
//		cursorPosition++;
//	}
//	shiftCursorRight(cursorPosition);
	displayString(sensorBuffer);
//	scroll(sensorBuffer);

}

void processLineThree(char *direction, int speed)
{
	shiftNewLine(LINE_THREE);
	sprintf(motorBuffer, "%s Speed:%d",direction, speed);
	displayString(motorBuffer);
}
void setupLcd(void)
{
	Uart2::getInstance().init(9600,100,100);

	u2.putChar(lcd_init);
	u2.putChar(lcd_clear);

	delay_ms(200);

	// Display Thunderbolt
	displayString(thunderbolt_string);

	//Move cursor to next line
	shiftNewLine(LINE_TWO);

	//Blink cursor
	u2.putChar(lcd_init);
	u2.putChar(cursor_blink);

}

void refreshLcd(void)
{
//	processLineTwo(left_data, front_data, right_data, back_data);
//	processLineThree(dir_data, rpm_data);
}







