#include "lcd.h"

Uart2 &u2 = Uart2::getInstance();
char lcd_init=0xFE;
char lcd_clear=0x51;
char cursor_right=0x56;
char cursor_line_two = 0x40;
char cursor_line_one = 0x00;
char set_cursor = 0x45;

enum NEW_LINE
{
	LINE_ONE = 0x00,
	LINE_TWO = 0x40,
	LINE_THREE = 0x14,
	LINE_FOUR = 0x54
};

string thunderbolt_string="Thunderbolt";
string forward_string = "forward";
string left_string = "left";
string right_string = "right";

int length=thunderbolt_string.length();

void shiftNewLine(int number)
{
	u2.putChar(lcd_init);
	u2.putChar(set_cursor);
	u2.putChar(LINE_ONE);
}

void shiftCursorRight(int number)
{
	for(int i=0;i<number;i++)
	{
		u2.putChar(lcd_init);
		delay_ms(10);
		u2.putChar(cursor_right);
	}
}

void shiftCursorLeft(int number)
{
	// Shift left
	for(int i=0;i<number;i++)
	{
		u2.putChar(lcd_init);
		u2.putChar(0x55);
		delay_ms(1);
	}
}

void displayString(string data)
{
	for(int i=0;i<data.length();i++)
	{
	  u2.putChar(data[i]);
	}
}

void setupLcd(void)
{
	Uart2::getInstance().init(9600,100,100);
	Uart2 &u2 = Uart2::getInstance();

	u2.putChar(lcd_init);
	u2.putChar(lcd_clear);

	delay_ms(200);

	// Shift Right
	shiftCursorRight(4);

	// Print Thunderbolt
	displayString(thunderbolt_string);
	printf("thunderbolt");

	// set cursor to next line
	u2.putChar(lcd_init);
	u2.putChar(set_cursor);
	u2.putChar(0x40);

    //Shift left
	//shiftCursorLeft(4);
}

void refreshLcd(void)
{
	if(SW.getSwitch(1))
	{
		displayString(left_string);
	}

	if(SW.getSwitch(2))
	{
		displayString(forward_string);
	}

	if(SW.getSwitch(3))
	{
		displayString(right_string);
	}
}







