/*
 * LCD.h
 *
 *  Created on: Apr 12, 2022
 *      Author: donst
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_



#include <stdint.h>
#include <stdbool.h>


typedef enum
{
	_LCDDriver_SSD1351,
	_LCDDriver_ST7789,
} _eLCDDriver;

//SSD1351 Regs
enum
{
	SSD1351_CMD_SETCOLUMN       = 0x15,
	SSD1351_CMD_SETROW          = 0x75,
	SSD1351_CMD_WRITERAM        = 0x5C,
	SSD1351_CMD_READRAM         = 0x5D,
	SSD1351_CMD_SETREMAP        = 0xA0,
	SSD1351_CMD_STARTLINE       = 0xA1,
	SSD1351_CMD_DISPLAYOFFSET   = 0xA2,
	SSD1351_CMD_DISPLAYALLOFF   = 0xA4,
	SSD1351_CMD_DISPLAYALLON    = 0xA5,
	SSD1351_CMD_NORMALDISPLAY   = 0xA6,
	SSD1351_CMD_INVERTDISPLAY   = 0xA7,
	SSD1351_CMD_FUNCTIONSELECT  = 0xAB,
	SSD1351_CMD_DISPLAYOFF      = 0xAE,
	SSD1351_CMD_DISPLAYON       = 0xAF,
	SSD1351_CMD_PRECHARGE       = 0xB1,
	SSD1351_CMD_DISPLAYENHANCE  = 0xB2,
	SSD1351_CMD_CLOCKDIV        = 0xB3,
	SSD1351_CMD_SETVSL          = 0xB4,
	SSD1351_CMD_SETGPIO         = 0xB5,
	SSD1351_CMD_PRECHARGE2      = 0xB6,
	SSD1351_CMD_SETGRAY         = 0xB8,
	SSD1351_CMD_USELUT          = 0xB9,
	SSD1351_CMD_PRECHARGELEVEL  = 0xBB,
	SSD1351_CMD_VCOMH           = 0xBE,
	SSD1351_CMD_CONTRASTABC     = 0xC1,
	SSD1351_CMD_CONTRASTMASTER  = 0xC7,
	SSD1351_CMD_MUXRATIO        = 0xCA,
	SSD1351_CMD_COMMANDLOCK     = 0xFD,
	SSD1351_CMD_HORIZSCROLL     = 0x96,
	SSD1351_CMD_STOPSCROLL      = 0x9E,
	SSD1351_CMD_STARTSCROLL     = 0x9F,
};


#define _LCD_HPixels	128
#define _LCD_VPixels	128



void LCD_DisplayOn( void );
void LCD_DisplayOff( void );
uint8_t LCD_WriteFrameBuffer( void );
uint16_t *LCD_GetPixelBuffer( uint8_t row );
void LCD_SetBandMask( uint32_t mask );
int LCD_GetTick( void );
void LCD_Init( _eLCDDriver driver );
void LCD_InitRegs( unsigned char doClear );

bool LCD_IsBusy( void );
void LCD_EnableUpdates( void );
void LCD_DisableUpdates( void );
uint8_t LCD_GetTickCounter( void );

void LCD_Scroll1( uint8_t startRow );

//////////////////////////////////////////////////////////////////////
//	FONTS
#include "../Fonts/Fonts.h"


//inline uint16_t swapBE( uint16_t val ) { return ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8); }


//////////////////////////////////////////////////////////////////////
//	DEFINES
//		Most of these are used internally by the library, but might
//		be useful to user code, especially if the user is defining
//		their own frame buffer, etc..
#define GL_PixelsX			_LCD_HPixels
#define GL_PixelsY			_LCD_VPixels
#define	GL_BytesPerLine		GL_PixelsX
#define GL_FrameBufferSize	(_LCD_HPixels * _LCD_VPixels * 2)

#define GL_BLACK			0x0000
#define GL_WHITE			0xFFFF

#define GL_BLUE           0x001F
#define GL_GBLUE          0X07FF
#define GL_RED            0xF800
#define GL_MAGENTA        0xF81F
#define GL_GREEN          0x07E0
#define GL_CYAN           0x7FFF
#define GL_YELLOW         0xFFE0
#define GL_BROWN          0XBC40
#define GL_BRRED          0XFC07
#define GL_GRAY           0X8430

#define IMAGE_BACKGROUND    GL_WHITE
#define FONT_FOREGROUND     GL_BLACK
#define FONT_BACKGROUND     GL_WHITE


//////////////////////////////////////////////////////////////////////
//	TYPE DEFINITIONS
//		These are used internally by the library. Also, some functions
//		take these types as parameters.
typedef struct
	{
	int		x;
	int		y;
	} _gl_POINT;


//////////////////////////////////////////////////////////////////////
//	FUNCTIONS
//		These are available to user code.

int _gl_iInitialize( void );

int _gl_iClearScreen( void );
uint16_t _gl_iSetColor( uint16_t color );
uint16_t _gl_iSetColorIndex( uint8_t color );

int _gl_iFillScreenWithByte( unsigned char value );
int _gl_iFillScreenWithPattern( unsigned char *pattern, int countofbytes );
int _gl_iImageToScreen( const uint8_t *image, int x, int y );

int _gl_iPlotPixel( _gl_POINT *p );
void _gl_vPlotPixelXY( int x, int y );
int _gl_iPlotPixelAbsolute( _gl_POINT *p, int level );
void _gl_iPlotPixelAbsoluteXY( int x, int y, int level );


int _gl_iMoveTo( _gl_POINT *pNew, _gl_POINT *pOld );
int _gl_iMoveToXY( int x, int y );
int _gl_iLineTo( _gl_POINT *pNew );
int _gl_iLineToXY( int x, int y );
int _gl_iLine( _gl_POINT *p1, _gl_POINT *p2 );
int _gl_iLineXY( int x1, int y1, int x2, int y2 );

int _gl_iRectangle( _gl_POINT *pUpperLeft,  _gl_POINT *pLowerRight );
int _gl_iRectangleXY( int left, int top, int right, int bottom );
int _gl_iFilledRectangle( _gl_POINT *pUpperLeft,  _gl_POINT *pLowerRight );
int _gl_iFilledRectangleXY( int left, int top, int right, int bottom );

int _gl_iCircle( _gl_POINT *pCenter, int radius );
int _gl_iCircleXY( int x, int y, int radius );
int _gl_iFilledCircle( _gl_POINT *pCenter, int radius );
int _gl_iFilledCircleXY( int x, int y, int radius );

int _gl_iEllipse( _gl_POINT *pCenter, int axisx, int axisy );
int _gl_iEllipseXY( int x, int y, int axisx, int axisy );
int _gl_iFilledEllipse( _gl_POINT *pCenter, int axisx, int axisy );
int _gl_iFilledEllipseXY( int xc, int yc, int axisx, int axisy );

uint8_t _gl__DrawChar(uint16_t Xpoint, uint16_t Ypoint, const char Acsii_Char,
                    const sFONT* Font, uint16_t Color_Foreground, uint16_t Color_Background);
void _gl_DrawString(uint16_t Xstart, uint16_t Ystart, const char * pString,
                         const sFONT* Font, uint16_t Color_Foreground, uint16_t Color_Background);
uint16_t _gl_StringWidth( const char *pString, const sFONT *Font );
uint8_t _gl_CharWidth( const char ch, const sFONT *Font );

void LCD_DMATxComplete( void );


#endif /* INC_LCD_H_ */
