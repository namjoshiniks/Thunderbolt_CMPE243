#include "lcd.h"
#include <cstring>
#include <sstream>
#include <iostream>

Uart3 &u3 = Uart3::getInstance();
Uart2 &u2 = Uart2::getInstance();
char lcd_init=0xFE;
char lcd_clear=0x51;
char cursor_right=0x56;
char cursor_blink = 0x4B;
char cursor_home = 0x46;
char set_cursor = 0x45;
char cursor_backspace = 0x4E;
string sensorBuffer;
char motorBuffer[20];
char distanceBuffer[30];
int cursorPosition;
int dPlace;
string leftString, rightString, frontString, backString;
string motorBuffer1;

string thunderbolt_string="    THUNDERBOLT";



enum NEW_LINE
{
	LINE_ONE = 0x00,
	LINE_ONE_END = 0x13,
	LINE_TWO = 0x40,
	LINE_TWO_END = 0x53,
	LINE_THREE = 0x14,
	LINE_THREE_END = 0x27,
	LINE_FOUR = 0x54,
	LINE_FOUR_END = 0x67
};

void shiftNewLine(NEW_LINE line)
{
	u2.putChar(lcd_init);
	u2.putChar(set_cursor);
	u2.putChar(line);
}

void shiftLineTwoEnd(void)
{
	u2.putChar(lcd_init);
	u2.putChar(set_cursor);
	u2.putChar(0x53);
}

void clearLine(NEW_LINE line)
{
	shiftNewLine(line);
	for(int i = 19; i > 0; i--)
	{
		u2.putChar(lcd_init);
		u2.putChar(cursor_backspace);
	}
}

void shiftCursorRight(int number)
{
	for(int i=0;i<number;i++)
	{
		u2.putChar(lcd_init);
		u2.putChar(0x4A);
	}
}

void shiftCursorLeft(int number)
{
	for(int i=0;i<number;i++)
	{
		u2.putChar(lcd_init);
		u2.putChar(0x49);
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
	string temp = "";
	static int countVal1;
	if(countVal1 > 19)
	{
		temp = data.substr(countVal1-19,data.length());
	}
	else
	{
		for(int i = 0; i < (19 - countVal1); i++)
		{
			temp = temp + " ";
		}

		temp = temp + data;
	}
	displayString(temp);
//	clearLine(line);
	//cout << temp << endl;

	if(countVal1 == (18 + data.length()))
	{
		countVal1 = 0;
	}
	else {
		countVal1++;
	}
}

void processLineOne(void)
{
//	u2.putChar(lcd_init);
//	u2.putChar(lcd_clear);
	shiftNewLine(LINE_ONE);
	//clearLine(LINE_ONE_END);
//	static int countVal;
//	string temp = "";
//
//	if(countVal > 19)
//	{
//		temp = thunderbolt_string.substr(countVal-19,thunderbolt_string.length());
//	}
//	else
//	{
//		for(int i = 0; i < (19 - countVal); i++)
//		{
//			temp = temp + " ";
//		}
//
//		temp = temp + thunderbolt_string;
//	}
//
//	displayString(temp);
//	clearLine(LINE_THREE_END);
//	cout << temp << endl;
//
//
//	if(countVal == (18 + thunderbolt_string.length()))
//	{
//		countVal = 0;
//	}
//	else {
//		countVal++;
//	}
	displayString(thunderbolt_string);

}

void processLineTwo(int leftVal, int frontVal, int rightVal, int backVal)
{
//	printf("Line two\n");
//	u2.putChar(lcd_init);
//	u2.putChar(lcd_clear);
	shiftNewLine(LINE_TWO);
	clearLine(LINE_TWO_END);
//	string temp = "";
//	static int countVal1;
	stringstream ss1, ss2, ss3, ss4;
	ss1 << leftVal;
	leftString = ss1.str();
	ss2 << frontVal;
	frontString = ss2.str();
	ss3 << rightVal;
	rightString = ss3.str();
	ss4 << backVal;
	backString = ss4.str();
	sensorBuffer = "L: " + leftString + " F: " + frontString + " R: " + rightString;
	cout<<sensorBuffer<<endl;
	displayString(sensorBuffer);
	//scroll(sensorBuffer);
//	if(countVal1 > 19)
//	{
//		temp = sensorBuffer.substr(countVal1-19,sensorBuffer.length());
//	}
//	else
//	{
//		for(int i = 0; i < (19 - countVal1); i++)
//		{
//			temp = temp + " ";
//		}
//
//		temp = temp + sensorBuffer;
//	}
//	displayString(temp);
//	clearLine(LINE_ONE_END);
//	displayString(thunderbolt_string);
//	clearLine(LINE_FOUR_END);
	//displayString(distanceBuffer);
//	cout << temp << endl;
//
//	if(countVal1 == (18 + sensorBuffer.length()))
//	{
//		countVal1 = 0;
//	}
//	else {
//		countVal1++;
//	}
}

//void processLineThree(char *direction, int speed)
//{
//	shiftNewLine(LINE_THREE);
//	sprintf(motorBuffer, "%s Speed:%d",direction, speed);
//	displayString(motorBuffer);
//}

void processLineThree(string direction, int speed)
{
	clearLine(LINE_THREE_END);
	shiftNewLine(LINE_THREE);
	stringstream ss;
	ss << speed;
	string temp = ss.str();
	motorBuffer1 = direction + " Speed: " + temp;
   // cout<<motorBuffer1<<endl;
	//sprintf(motorBuffer, "%s Speed:%d",direction, speed);
	displayString(motorBuffer1);
	cout<<direction<<endl;
	cout<<motorBuffer1<<endl;
	//clearLine(LINE_FOUR_END);
    //scroll(motorBuffer1);
}

void processLineFour(float turnAngle, float disCheckpoint, float disDestination)
{
	clearLine(LINE_FOUR_END);
	shiftNewLine(LINE_FOUR);
	sprintf(distanceBuffer, "A:%0.0f C:%0.0f D:%0.0f", turnAngle, disCheckpoint, disDestination);
	//scroll(distanceBuffer);
	displayString(distanceBuffer);
}

void setupLcd(void)
{
	Uart2::getInstance().init(9600,100,100);
	u2.putChar(lcd_init);
	u2.putChar(lcd_clear);

	 //Display Thunderbolt
	displayString(thunderbolt_string);

	//Move cursor to next line
	//shiftNewLine(LINE_TWO);

	//Blink cursor
//	u2.putChar(lcd_init);
//	u2.putChar(cursor_blink);

}

void initGlcd(void)
{
	u3.init(9600,100,100);
}

void setupGlcd(char16_t a, char16_t b, char16_t c, char16_t d, char16_t e)
{
	char16_t graphicLcdData[6] = {a,b,c,d,e,0};
	char checksum = 0;
	//printf("Cmd ");
	//graphicLcdData = {a,b,c,d,e};
	for(int i = 0; i <= 4; i++)
	{
		graphicLcdData[5] ^= graphicLcdData[i];
		u3.putChar(graphicLcdData[i]);
		//printf(" %x ", graphicLcdData[i]);
	}
	u3.putChar(graphicLcdData[5] );
	//printf("%x\n",graphicLcdData[5] );
}

void refreshLcd(void)
{
//	processLineTwo(left_data, front_data, right_data, back_data);
//	processLineThree(dir_data, rpm_data);
}







