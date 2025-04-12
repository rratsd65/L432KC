/*
 * LCD.c
 *
 *  Created on: Apr 12, 2022
 *      Author: donst
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "spi.h"
#include "cmsis_os2.h"

#include "LCD.h"
#include "MiscDefs.h"

#include "main.h"



extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

#define _GPIO_Pin_HI(port,pin)	((port)->BSRR=(pin))
#define _GPIO_Pin_LO(port,pin)	((port)->BRR=(pin))

#define _LCD_Mode_Command	_GPIO_Pin_LO(LCD_DC_GPIO_Port,LCD_DC_Pin)
#define _LCD_Mode_Data		_GPIO_Pin_HI(LCD_DC_GPIO_Port,LCD_DC_Pin)

#define _LCD_RST_Active		_GPIO_Pin_LO(LCD_RST_GPIO_Port,LCD_RST_Pin)
#define _LCD_RST_Inactive	_GPIO_Pin_HI(LCD_RST_GPIO_Port,LCD_RST_Pin)

#define _LCD_CS_Active		_GPIO_Pin_LO(LCD_CS_GPIO_Port,LCD_CS_Pin)
#define _LCD_CS_Inactive	_GPIO_Pin_HI(LCD_CS_GPIO_Port,LCD_CS_Pin)

#define _LCD_NumBands		1		// to split frame buffer writes across interrupts

static struct
{
	uint8_t	 			pixelBuffer[_LCD_VPixels][_LCD_HPixels][2];

	uint8_t				tickCounter;

	uint32_t			bandUpdateMask;

	void				(*callback)( uint32_t arg );

	uint8_t				updatesEnabled : 1;
	uint8_t				dmaActive : 1;
	uint8_t				lcdBusy : 1;
	uint8_t				displayOn : 1;
	uint8_t				needDisplayOnOffCommand : 1;

} lcdData __attribute__ ((section (".xbss")));

void LCD_DisplayOn( void )
{
	lcdData.displayOn = 1;
	lcdData.needDisplayOnOffCommand = 1;
}
void LCD_DisplayOff( void )
{
	lcdData.displayOn = 0;
	lcdData.needDisplayOnOffCommand = 1;
}
uint16_t *LCD_GetPixelBuffer( uint8_t row )
{
	uint16_t *rv;

	rv = (uint16_t *)&lcdData.pixelBuffer[row][0][0];

	return rv;
}
void LCD_SetBandMask( uint32_t mask )
{
	lcdData.bandUpdateMask |= mask;
}
bool LCD_IsBusy( void )
{
	return lcdData.lcdBusy;
}
static inline void LCD_SetBandBit( uint8_t row )
{
	lcdData.bandUpdateMask |= (1U << (row / (_LCD_VPixels/_LCD_NumBands)));
}
static inline void LCD_ClearBandBit( uint8_t row )
{
	lcdData.bandUpdateMask &= ~(1U << (row / (_LCD_VPixels/_LCD_NumBands)));
}
static inline void LCD_SetAllBandBits( void )
{
	lcdData.bandUpdateMask = (1U << _LCD_NumBands) - 1;
}
static inline uint8_t LCD_IsBandBitSet_abs( uint8_t band )
{
	return (lcdData.bandUpdateMask & (1U << band)) ? 1 : 0;
}
static inline void LCD_ClearBandBit_abs( uint8_t band )
{
	lcdData.bandUpdateMask &= ~(1U << band);
}

static void SendPacket( uint8_t cmdByte, void *ptr, uint16_t len );

extern osEventFlagsId_t sync_eventHandle;

static inline uint8_t SPI_IsBusy( SPI_HandleTypeDef *hspi )
{
	return (hspi->Instance->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 ||
		   (hspi->Instance->SR & SPI_SR_BSY)
		   ? 1 : 0;
}


#define SPI_WAIT_SR_EMPTY(SPIx)		while ( ((SPIx)->Instance->SR & (SPI_SR_BSY))){}	// wait for shift register empty
#define SPI_WAIT_TX_EMPTY(SPIx)		while (!((SPIx)->Instance->SR & (SPI_SR_TXE))){}	// wait for tx register empty

int LCD_GetTick( void ) { return lcdData.tickCounter; }

void LCD_DMATxComplete( void )
{
	if ( lcdData.dmaActive )
	{
		SPI_WAIT_SR_EMPTY(&hspi1);

		_LCD_CS_Inactive;

	//	_GPIO_Pin_LO(GPIOB, GPIO_PIN_3);			// PB3 to time the display

		if ( ++lcdData.tickCounter >= _LCD_NumBands )
		{
			lcdData.tickCounter = 0;
		}

		lcdData.dmaActive = 0;

		if ( lcdData.callback != NULL )
		{
			lcdData.callback( LCD_CallbackReason_DMAComplete );
		}
	}

	lcdData.lcdBusy = false;
}
void LCD_DMATxHalfComplete( void )
{
//	BreakHere();
}
void LCD_DMATxError( void )
{
//	BreakHere();
}
void LCD_DMATxAbort( void )
{
//	BreakHere();
}



static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
typedef struct
{
	__IO uint32_t ISR;   /*!< DMA interrupt status register */
	__IO uint32_t Reserved0;
	__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

static void writeCommand( uint8_t byte )
{
	_LCD_CS_Active;
	_LCD_Mode_Command;

	// send command byte to SPI data reg
    *((__IO uint8_t *)&hspi1.Instance->DR) = byte;

    SPI_WAIT_SR_EMPTY(&hspi1);

	_LCD_CS_Inactive;
}
static void writeData( uint8_t byte )
{
	_LCD_CS_Active;
	_LCD_Mode_Data;

	// send command byte to SPI data reg
    *((__IO uint8_t *)&hspi1.Instance->DR) = byte;

    SPI_WAIT_SR_EMPTY(&hspi1);

	_LCD_CS_Inactive;
}
static void SendPacket( uint8_t cmdByte, void *ptr, uint16_t len )
	{
	// wait for previous command's data to be shifted out before flipping D/*C pin
	SPI_WAIT_SR_EMPTY(&hspi1);

	_LCD_Mode_Command;

	// send command byte to SPI data reg
    *((__IO uint8_t *)&hspi1.Instance->DR) = cmdByte;

	lcdData.dmaActive = 1;

#if 0
	SPI_WAIT_SR_EMPTY(&hspi1);

	// Set to 'data' mode...
	_LCD_Mode_Data;

	HAL_SPI_Transmit_DMA( &hspi1, ptr, len );

#else

    // command byte still being shifted out while we set up DMA

	__HAL_DMA_DISABLE(hspi1.hdmatx);

	// set bytes to write
	hspi1.hdmatx->Instance->CNDTR = len;

	// set source address
	hspi1.hdmatx->Instance->CMAR = (uint32_t)ptr;

	// Enable the TxCplt and Error ints
	__HAL_DMA_ENABLE_IT(hspi1.hdmatx, (DMA_IT_TC | DMA_IT_TE));

	// wait for transmitter empty before enabling DMA
	SPI_WAIT_TX_EMPTY(&hspi1);

	// enable DMA
	__HAL_DMA_ENABLE(hspi1.hdmatx);

	// back to data mode
	_LCD_Mode_Data;

	// enable the DMA request
	SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
#endif
	}

uint8_t LCD_GetTickCounter( void )
{
	return lcdData.tickCounter;
}
uint8_t LCD_WriteFrameBuffer( void )
{
	// allow main app to pause updates (reduce weird lines in rapidly changing displays)
	if ( !lcdData.updatesEnabled )
		{
		return 0;
		}

	// if current 'band' is unchanged, skip
	if ( ! LCD_IsBandBitSet_abs( lcdData.tickCounter ) )
	{
		if ( ++lcdData.tickCounter >= _LCD_NumBands )
		{
			lcdData.tickCounter = 0;
		}
		return 0;
	}

	lcdData.lcdBusy = true;

	// clear the changed flag
	LCD_ClearBandBit_abs( lcdData.tickCounter );

	_LCD_CS_Active;

	_LCD_Mode_Command;

	if ( lcdData.needDisplayOnOffCommand )
	{
		*((__IO uint8_t *)&hspi1.Instance->DR) = lcdData.displayOn ? SSD1351_CMD_DISPLAYON : SSD1351_CMD_DISPLAYOFF;

		SPI_WAIT_SR_EMPTY(&hspi1);

		lcdData.needDisplayOnOffCommand = 0;
	}

	*((__IO uint8_t *)&hspi1.Instance->DR) = SSD1351_CMD_SETROW;

	SPI_WAIT_SR_EMPTY(&hspi1);

	_LCD_Mode_Data;

	*((__IO uint8_t *)&hspi1.Instance->DR) = (lcdData.tickCounter * (_LCD_VPixels / _LCD_NumBands));

	*((__IO uint8_t *)&hspi1.Instance->DR) = (_LCD_HPixels-1);

    uint16_t ptrOffset = lcdData.tickCounter * (_LCD_VPixels / _LCD_NumBands) * 2 * _LCD_HPixels;

	SendPacket( SSD1351_CMD_WRITERAM, (uint8_t *)lcdData.pixelBuffer + ptrOffset, sizeof(lcdData.pixelBuffer)/_LCD_NumBands );

	return 1;
}

void LCD_EnableUpdates( void )
{
	lcdData.updatesEnabled = 1;
}

void LCD_DisableUpdates( void )
{
	lcdData.updatesEnabled = 0;
}

#if 1
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);

  /* DMA Normal Mode */
  if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
  {
    /* Disable ERR interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

    /* Disable Tx DMA Request */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);

    /* Clear overrun flag in 2 Lines communication mode because received data is not read */
    if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->TxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;

    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
      /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
      return;
    }
  }
  /* Call user Tx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->TxCpltCallback(hspi);
#else
  HAL_SPI_TxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}
#endif
void LCD_Init( void (*eventCallback)(uint32_t arg), _eLCDDriver driver )
	{
	memset( &lcdData, 0, sizeof(lcdData) );

	lcdData.callback = eventCallback;

	_LCD_CS_Inactive;
	uint8_t c = 0xCA;
	HAL_SPI_Transmit(&hspi1, &c, 1, 10 );
	HAL_Delay( 1 );

	_LCD_RST_Inactive;
	HAL_Delay(5);
	_LCD_RST_Active;
	HAL_Delay(5);
	_LCD_RST_Inactive;
	HAL_Delay(5);

	LCD_InitRegs(0);

	lcdData.displayOn = 1;
	lcdData.needDisplayOnOffCommand = 0;

	LCD_SetAllBandBits();

	lcdData.tickCounter = 0;
	lcdData.updatesEnabled = 1;

	  /* Init field not used in handle to zero */
	  hspi1.pRxBuffPtr  = (uint8_t *)NULL;
	  hspi1.TxISR       = NULL;
	  hspi1.RxISR       = NULL;
	  hspi1.RxXferSize  = 0U;
	  hspi1.RxXferCount = 0U;


	  /* Set the SPI TxDMA transfer complete callback */
	  hspi1.hdmatx->XferCpltCallback = SPI_DMATransmitCplt;

	  /* Set the SPI TxDMA Half transfer complete callback */
	  hspi1.hdmatx->XferHalfCpltCallback = NULL;

	  /* Set the DMA error callback */
	  hspi1.hdmatx->XferErrorCallback = NULL;

	  /* Set the DMA AbortCpltCallback */
	  hspi1.hdmatx->XferAbortCallback = NULL;

		hspi1.hdmatx->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hspi1.hdmatx->ChannelIndex & 0x1CU));

		/* Configure DMA Channel destination address */
		hspi1.hdmatx->Instance->CPAR = (uint32_t)&hspi1.Instance->DR;
	}

void LCD_InitRegs( unsigned char doClear )
	{
    writeCommand(0xfd);  // command lock
    writeData(0x12);
    writeCommand(0xfd);  // command lock
    writeData(0xB1);

    writeCommand(0xae);  // display off
    writeCommand(0xa4);  // Normal Display mode

    writeCommand(0x15);  //set column address
    writeData(0x00);     //column address start 00
    writeData(0x7f);     //column address end 127
    writeCommand(0x75);  //set row address
    writeData(0x00);     //row address start 00
    writeData(0x7f);     //row address end 127

    writeCommand(0xB3);
    writeData(0xF1);

    writeCommand(0xCA);
    writeData(0x7F);

    writeCommand(0xa0);  //set re-map & data format
    writeData(0x74);     //Horizontal address increment

    writeCommand(0xa1);  //set display start line
    writeData(0x00);     //start 00 line

    writeCommand(0xa2);  //set display offset
    writeData(0x00);

    writeCommand(0xAB);
    writeCommand(0x01);

    writeCommand(0xB4);
    writeData(0xA0);
    writeData(0xB5);
    writeData(0x55);

    writeCommand(0xC1);
    writeData(0xC8);
    writeData(0x80);
    writeData(0xC0);

    writeCommand(0xC7);
    writeData(0x0F);

    writeCommand(0xB1);
    writeData(0x32);

    writeCommand(0xB2);
    writeData(0xA4);
    writeData(0x00);
    writeData(0x00);

    writeCommand(0xBB);
    writeData(0x17);

    writeCommand(0xB6);
    writeData(0x01);

    writeCommand(0xBE);
    writeData(0x05);

    writeCommand(0xA6);

    HAL_Delay(200);

    writeCommand(0xAF);

	// Set to 'data' mode...
	_LCD_Mode_Data;
	}


void LCD_Scroll1( uint8_t startRow )
{
	uint8_t tmpbuf[128*2];

	memmove( tmpbuf, lcdData.pixelBuffer[startRow], 128*2 );
	memmove( lcdData.pixelBuffer[startRow], lcdData.pixelBuffer[startRow+1], 2*128*(128-startRow-1) );
	memmove( lcdData.pixelBuffer[127], tmpbuf, 128*2 );

	LCD_SetAllBandBits();
}


// Define the arrays for the LCD screen buffer
// this is defined to be static, so it won't interfere
// with user code (and so that user code can't touch
// it directly!!!)
uint8_t * _gl_cpCurrentFrameBuffer;

// Store the currently-selected level. This ranges from
// GL_BLACK to GL_WHITE
static uint16_t _gl_iCurrentColor, _gl_iCurrentColorBE;

// Store the current pixel locations used for drawing and text
static uint16_t _gl_iPixelX, _gl_iPixelY;


/***********************************************************************
 *
 * UNIT NAME:	_gl_iInitialize
 *
 * PURPOSE:
 *		Initializes the graphics library.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 **********************************************************************/
int _gl_iInitialize( void )
{
	_gl_cpCurrentFrameBuffer = (unsigned char *)lcdData.pixelBuffer;

	/* Set current level to black */
	_gl_iSetColor( GL_BLACK );

	/* fill the screen with white */
	_gl_iClearScreen();

	_gl_iPixelX = _gl_iPixelY = 0;

	return 1;
}

inline uint16_t swapBE( uint16_t val ) { return ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8); }

// The above, but:
//  * no return value
//  * doesn't set band bit (caller must do it)
//  * inline
static inline void _gl_iPlotPixelAbsoluteXY_min( int x, int y, int value )
{
	if ( x >= 0 && x < GL_PixelsX &&
	     y >= 0 && y < GL_PixelsY )
	{
		*(uint16_t *)(_gl_cpCurrentFrameBuffer + (y * GL_PixelsX + x) * 2) = swapBE( value );
	}
}

/***********************************************************************
 *
 * UNIT NAME:	_gl_iPlotPixel
 *
 * PURPOSE:
 *		Sets the specified pixel to the current gray scale value.
 *
 * RETURNS:
 *		The previous value of the pixel.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 **********************************************************************/
int _gl_iPlotPixel( _gl_POINT *p )
{
//	return
	_gl_iPlotPixelAbsoluteXY( p->x, p->y, _gl_iCurrentColor );
	return 0;
}

/***********************************************************************
 *
 * UNIT NAME:	_gl_vPlotPixelXY
 *
 * PURPOSE:
 *		Sets the specified pixel to the current gray scale value.
 *
 * RETURNS:
 *		none.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 **********************************************************************/
void _gl_vPlotPixelXY( int x, int y )
{
	_gl_iPlotPixelAbsoluteXY( x, y, _gl_iCurrentColor );
}

/***********************************************************************
 *
 * UNIT NAME:	_gl_iPlotPixelAbsolute
 *
 * PURPOSE:
 *		Sets the specified pixel to the specified gray scale value.
 *
 * RETURNS:
 *		The previous value of the pixel.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 **********************************************************************/
int _gl_iPlotPixelAbsolute( _gl_POINT *pt, int value )
	{
//	return
	_gl_iPlotPixelAbsoluteXY( pt->x, pt->y, value );
	return 0;
	}

/***********************************************************************
 *
 * UNIT NAME:	_gl_iPlotPixelAbsolute
 *
 * PURPOSE:
 *		Sets the specified pixel to the specified gray scale value.
 *
 * RETURNS:
 *		nothing
 *
 * PROCESSING:
 *
 * NOTES:
 *
 **********************************************************************/
void _gl_iPlotPixelAbsoluteXY( int x, int y, int value )
{
	if ( x < 0 || x >= GL_PixelsX ||
	     y < 0 || y >= GL_PixelsY )
		return;

	uint16_t *ptr = (uint16_t *)(_gl_cpCurrentFrameBuffer + (y * GL_PixelsX + x) * 2);

	*ptr = swapBE( value );

	LCD_SetBandBit(y);
}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFillScreenWithByte
 *
 * PURPOSE:
 *    Fills display memory with the byte specified.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFillScreenWithByte( unsigned char value )
	{
	memset( _gl_cpCurrentFrameBuffer, value, GL_FrameBufferSize );

	LCD_SetAllBandBits();

	return 1;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFillScreenWithPattern
 *
 * PURPOSE:
 *    Fills display memory with the fill pattern beginning with the
 *    start address.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFillScreenWithPattern( unsigned char *pattern, int countofbytes )
	{
	int whole, rem;
	unsigned char *pBuffer;
	int i;

	// get the number of memcpy()s we can do
	whole = GL_FrameBufferSize / countofbytes;
	// get the size of the remaining memset
	rem = GL_FrameBufferSize - (whole * countofbytes);

	pBuffer = _gl_cpCurrentFrameBuffer;

	for ( i=0; i<whole; i++, pBuffer += whole )
		memcpy( pBuffer, pattern, whole );

	memcpy( pBuffer, pattern, rem );

	LCD_SetAllBandBits();

	return 1;
	}



/***********************************************************************
 *
 * UNIT NAME:       _gl_iClearScreen
 *
 * PURPOSE:
 *    Fills the current frame buffer with WHITE (0xFF).
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iClearScreen( void )
{
	memset( _gl_cpCurrentFrameBuffer, 0, GL_FrameBufferSize );

	LCD_SetAllBandBits();

	return 1;
}


/***********************************************************************
 *
 * UNIT NAME:       _gl_iSetColor
 *
 * PURPOSE:
 *		Sets the gray scale level used for all subsequent drawing
 *		operations (except those with an absolute parameter).
 *
 * RETURNS:
 *		The previous gray scale level.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
uint16_t _gl_iSetColor( uint16_t color )
{
	uint16_t rv;

	rv = _gl_iCurrentColor;
	_gl_iCurrentColor = color;
	_gl_iCurrentColorBE = swapBE( _gl_iCurrentColor );


	return rv;
}

static const uint16_t colorMap[] =
{
	GL_BLACK,
	GL_WHITE,
	GL_BLUE,
	GL_GBLUE,
	GL_RED,
	GL_MAGENTA,
	GL_GREEN,
	GL_CYAN,
	GL_YELLOW,
	GL_BROWN,
	GL_BRRED,
	GL_GRAY,
};
uint16_t _gl_iSetColorIndex( uint8_t color )
{
	uint16_t rv, newColor;

	rv = _gl_iCurrentColor;

	if ( color >= sizeof(colorMap)/sizeof(colorMap[0]) )
		newColor = GL_WHITE;
	else
		newColor = colorMap[color];

	_gl_iCurrentColor = newColor;
	_gl_iCurrentColorBE = swapBE( _gl_iCurrentColor );

	return rv;
}



/***********************************************************************
 *
 * UNIT NAME:       _gl_iMoveTo
 *
 * PURPOSE:
 *		Sets the current draw position to <new>. If pOld is not NULL,
 *		returns the previous position there. <new> can be NULL.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iMoveTo( _gl_POINT *pNew, _gl_POINT *pOld )
	{
	if ( pOld != NULL )
		{
		pOld->x = _gl_iPixelX;
		pOld->y = _gl_iPixelY;
		}

	if ( pNew != NULL )
		{
		_gl_iPixelX = _min( GL_PixelsX-1, _max( 0, pNew->x ) );
		_gl_iPixelY = _min( GL_PixelsY-1, _max( 0, pNew->y ) );
		}

	return 1;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iMoveToXY
 *
 * PURPOSE:
 *		Sets the current draw position to <x,y>.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iMoveToXY( int x, int y )
	{
	_gl_iPixelX = _min( GL_PixelsX-1, _max( 0, x ) );
	_gl_iPixelY = _min( GL_PixelsY-1, _max( 0, y ) );

	return 1;
	}


/***********************************************************************
 *
 * UNIT NAME:       _gl_iLineTo
 *
 * PURPOSE:
 *		Draws a line from the current draw position to the
 *		specified position (inclusive).
 *
 *		Updates the current draw position.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iLineTo( _gl_POINT *pNew )
	{
	int rv;
	_gl_POINT Cur;

	Cur.x = _gl_iPixelX;
	Cur.y = _gl_iPixelY;

	rv = _gl_iLine( &Cur, pNew );

	_gl_iPixelX = pNew->x;
	_gl_iPixelY = pNew->y;

	return rv;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iLineToXY
 *
 * PURPOSE:
 *		Draws a line from the current draw position to the
 *		specified position (inclusive).
 *
 *		Updates the current draw position.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iLineToXY( int x, int y )
	{
	int rv;

	rv = _gl_iLineXY( _gl_iPixelX, _gl_iPixelY, x, y );

	_gl_iPixelX = x;
	_gl_iPixelY = y;

	return rv;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iLine
 *
 * PURPOSE:
 *		Draws a line from <p1> to <p2> (inclusive).
 *
 *		Does NOT update the current draw position.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iLine( _gl_POINT *p1, _gl_POINT *p2 )
{
	return _gl_iLineXY( p1->x, p1->y, p2->x, p2->y );
}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iLineXY
 *
 * PURPOSE:
 *		Draws a line from <x1,y1> to <x2,y2> (inclusive).
 *
 *		Does NOT update the current draw position.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iLineXY( int x1, int y1, int x2, int y2 )
	{
	// single point
	if ( x1 == x2 && y1 == y2 )
		{
		_gl_vPlotPixelXY( x1, y1 );
		return 1;
		}

	// horizontal line
	if ( y1 == y2 )
		{
		if ( y1 < 0 || y1 >= GL_PixelsY )
			return 0;

		int start = _min( x1, x2 );
		int stop = _max( x1, x2 );

		if ( start < 0 )
			start = 0;
		else
		if ( start >= GL_PixelsX )
			return 0;
		if ( stop < 0 )
			return 0;
		else
		if ( stop >= GL_PixelsX )
			stop = GL_PixelsX-1;

		uint16_t *ptr = (uint16_t *)(_gl_cpCurrentFrameBuffer + 2 * (y1 * GL_PixelsX + start));

		while ( start <= stop )
			{
			*ptr++ = _gl_iCurrentColorBE;
			start++;
			}

		LCD_SetBandBit(y1);
		}

	// vertical line
	else
	if ( x1 == x2 )
		{
		if ( x1 < 0 || x1 >= GL_PixelsX )
			return 0;

		int startY = _min( y1, y2 );
		int stopY = _max( y1, y2 );

		if ( startY < 0 )
			startY = 0;
		else
		if ( startY >= GL_PixelsY )
			return 0;
		if ( stopY < 0 )
			return 0;
		else
		if ( stopY >= GL_PixelsY )
			stopY = GL_PixelsY-1;

		uint16_t *ptr = (uint16_t *)(_gl_cpCurrentFrameBuffer + 2 * (x1 + GL_PixelsX * startY));

		while ( startY <= stopY )
			{
			*ptr = _gl_iCurrentColorBE;
			ptr += GL_PixelsX;

			LCD_SetBandBit( startY );

			startY++;
			}
		}

	// diagonal line
	else
		{
		int d, dx, dy;
		int Aincr, Bincr, yincr, xincr;
		int x, y, swap;

		if ( abs( x1-x2 ) >= abs( y1-y2 ) )	// |slope| < 1?
			{
			// force x1 < x2
			if ( x2 < x1 )
				{
				int i = x2;
				x2 = x1;
				x1 = i;
				i = y2;
				y2 = y1;
				y1 = i;
				}

			dx = x2 - x1;
			dy = abs( y2-y1 );

			yincr = y2 > y1 ? 1 : -1;

			d = 2 * dy - dx;

			Aincr = 2 * (dy - dx);
			Bincr = 2 * dy;

			x = x1;
			y = y1;

			_gl_vPlotPixelXY( x, y );

			for ( x=x1+1; x<=x2; x++ )
				{
				if ( d >= 0 )
					{
					y += yincr;
					d += Aincr;
					}
				else
					d += Bincr;

				_gl_vPlotPixelXY( x, y );
				}
			}
		else
			{
			// force y1 < y2
			if ( y2 < y1 )
				{
				swap = y2;
				y2 = y1;
				y1 = swap;
				swap = x2;
				x2 = x1;
				x1 = swap;
				}

			dx = abs( x2 - x1 );
			dy = y2 - y1;

			xincr = x2 > x1 ? 1 : -1;

			d = 2 * dx - dy;

			Aincr = 2 * (dx - dy);
			Bincr = 2 * dx;

			x = x1;
			y = y1;

			_gl_vPlotPixelXY( x, y );

			for ( y=y1+1; y<=y2; y++ )
				{
				if ( d >= 0 )
					{
					x += xincr;
					d += Aincr;
					}
				else
					d += Bincr;

				_gl_vPlotPixelXY( x, y );
				}
			}
		}

	return 1;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iRectangle
 *
 * PURPOSE:
 *		Draws a rectangle.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iRectangle( _gl_POINT *pUpperLeft,  _gl_POINT *pLowerRight )
	{
	return _gl_iRectangleXY( pUpperLeft->x, pUpperLeft->y,
							 pLowerRight->x, pLowerRight->y );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iRectangleXY
 *
 * PURPOSE:
 *		Draws a rectangle.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iRectangleXY( int left, int top, int right, int bottom )
	{
	_gl_iLineXY( left, top, right, top );
	_gl_iLineXY( right, top, right, bottom );
	_gl_iLineXY( right, bottom, left, bottom );
	_gl_iLineXY( left, bottom, left, top );

	return 1;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFilledRectangle
 *
 * PURPOSE:
 *		Draws a rectangle, filled with the current level.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFilledRectangle( _gl_POINT *pUpperLeft,  _gl_POINT *pLowerRight )
	{
	return _gl_iFilledRectangleXY( pUpperLeft->x, pUpperLeft->y,
								   pLowerRight->x, pLowerRight->y );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFilledRectangleXY
 *
 * PURPOSE:
 *		Draws a rectangle, filled with the current level.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFilledRectangleXY( int left, int top, int right, int bottom )
	{
	int i;

	for ( i=top; i<=bottom; i++ )
		_gl_iLineXY( left, i, right, i );

	return 1;
	}


/***********************************************************************
 *
 * UNIT NAME:       _gl_iCircle
 *
 * PURPOSE:
 *		Draws a circle from a point and a radius.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iCircle( _gl_POINT *pCenter, int radius )
	{
	return _gl_iCircleXY( pCenter->x, pCenter->y, radius );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iCircleXY
 *
 * PURPOSE:
 *		Draws a circle from a point and a radius.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iCircleXY( int x, int y, int radius )
	{
	return _gl_iEllipseXY( x, y, radius, radius );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFilledCircle
 *
 * PURPOSE:
 *		Draws a FILLED circle from a point and a radius.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFilledCircle( _gl_POINT *pCenter, int radius )
	{
	return _gl_iFilledCircleXY( pCenter->x, pCenter->y, radius );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFilledCircleXY
 *
 * PURPOSE:
 *		Draws a FILLED circle from a point and a radius.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFilledCircleXY( int x, int y, int radius )
	{
	return _gl_iFilledEllipseXY( x, y, radius, radius );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iEllipse
 *
 * PURPOSE:
 *		Draws an ellipse from a point and the two axes.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iEllipse( _gl_POINT *pCenter, int axisx, int axisy )
	{
	return _gl_iEllipseXY( pCenter->x, pCenter->y, axisx, axisy );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iEllipseXY
 *
 * PURPOSE:
 *		Draws an ellipse from a point and the two axes.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/

int _gl_iEllipseXY( int xc, int yc, int axisx, int axisy )
	{
	int x			= 0;
	int y			= axisy;

	int a			= axisx;
	int b			= axisy;

	int Asquared	= a * a;
	int TwoAsquared	= 2 * Asquared;
	int Bsquared	= b * b;
	int TwoBsquared	= 2 * Bsquared;

	int d, dx, dy;

	d = Bsquared - (Asquared * b) + (Asquared / 4);
	dx = 0;
	dy = TwoAsquared * b;

	while ( dx < dy )
		{
		_gl_vPlotPixelXY( xc+x, yc+y );
		_gl_vPlotPixelXY( xc-x, yc+y );
		_gl_vPlotPixelXY( xc+x, yc-y );
		_gl_vPlotPixelXY( xc-x, yc-y );

		if ( d > 0 )
			{
			y--;
			dy -= TwoAsquared;
			d -= dy;
			}

		x++;
		dx += TwoBsquared;
		d += Bsquared +dx;
		}

	d += (3 * (Asquared - Bsquared) / 2 - (dx + dy)) / 2;

	while ( y >= 0 )
		{
		_gl_vPlotPixelXY( xc+x, yc+y );
		_gl_vPlotPixelXY( xc-x, yc+y );
		_gl_vPlotPixelXY( xc+x, yc-y );
		_gl_vPlotPixelXY( xc-x, yc-y );

		if ( d < 0 )
			{
			x++;
			dx += TwoBsquared;
			d += dx;
			}

		y--;
		dy -= TwoAsquared;
		d += Asquared - dy;
		}

	return 1;
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFilledEllipse
 *
 * PURPOSE:
 *		Draws a FILLED ellipse from a point and the two axes.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFilledEllipse( _gl_POINT *pCenter, int axisx, int axisy )
	{
	return _gl_iFilledEllipseXY( pCenter->x, pCenter->y, axisx, axisy );
	}

/***********************************************************************
 *
 * UNIT NAME:       _gl_iFilledEllipse
 *
 * PURPOSE:
 *		Draws a FILLED ellipse from a point and the two axes.
 *
 * RETURNS:
 *		1 on success, 0 on failure.
 *
 * PROCESSING:
 *
 * NOTES:
 *
 *
 **********************************************************************/
int _gl_iFilledEllipseXY( int xc, int yc, int axisx, int axisy )
	{
	int x			= 0;
	int y			= axisy;

	int a			= axisx;
	int b			= axisy;

	int Asquared	= a * a;
	int TwoAsquared	= 2 * Asquared;
	int Bsquared	= b * b;
	int TwoBsquared	= 2 * Bsquared;

	int d, dx, dy;

	d = Bsquared - (Asquared * b) + (Asquared / 4);
	dx = 0;
	dy = TwoAsquared * b;

	while ( dx < dy )
		{
		_gl_iLineXY( xc-x, yc+y, xc+x, yc+y );
		_gl_iLineXY( xc-x, yc-y, xc+x, yc-y );

		if ( d > 0 )
			{
			y--;
			dy -= TwoAsquared;
			d -= dy;
			}

		x++;
		dx += TwoBsquared;
		d += Bsquared +dx;
		}

	d += (3 * (Asquared - Bsquared) / 2 - (dx + dy)) / 2;

	while ( y >= 0 )
		{
		_gl_iLineXY( xc-x, yc+y, xc+x, yc+y );
		_gl_iLineXY( xc-x, yc-y, xc+x, yc-y );

		if ( d < 0 )
			{
			x++;
			dx += TwoBsquared;
			d += dx;
			}

		y--;
		dy -= TwoAsquared;
		d += Asquared - dy;
		}

	return 1;
	}

int _gl_iImageToScreen( const uint8_t *image, int x, int y )
{
	int16_t xSrc, xDst, xLen;
	int16_t ySrc, yDst, yLen;

	if ( x >= GL_PixelsX || y >= GL_PixelsY )
	{
		return 0;
	}

	image++;
	image++;

	uint16_t imageWidth = swapBE( *(uint16_t *)image );
	image += 2;

	uint16_t imageHeight = swapBE( *(uint16_t *)image );
	image += 2;

	image++;
	image++;

	if ( x + imageWidth < 0 || y + imageHeight < 0 )
	{
		return 0;
	}

	if ( x < 0 )
	{
		xDst = 0;
		xSrc = -x;
		xLen = imageWidth - xSrc;
	}
	else
	{
		xDst = x;
		xSrc = 0;
		xLen = imageWidth;
	}
	xLen = _min( xLen, GL_PixelsX - xDst );
	if ( y < 0 )
	{
		yDst = 0;
		ySrc = -y;
		yLen = imageHeight - ySrc;
	}
	else
	{
		yDst = y;
		ySrc = 0;
		yLen = imageHeight;
	}
	yLen = _min( yLen, GL_PixelsY - yDst );

	if ( !xLen || !yLen )
		return 0;

	unsigned char *pDst = lcdData.pixelBuffer[yDst][xDst];
	const unsigned char *pSrc = image + ((ySrc * imageWidth) + xSrc) * 2;
	xLen *= 2;

	while ( yLen-- )
	{
		memmove( pDst, pSrc, xLen );
		pDst += GL_PixelsX * 2;
		pSrc += imageWidth * 2;

		LCD_SetBandBit( yDst );

		yDst++;
	}

	return 1;
}


uint8_t _gl__DrawChar(uint16_t Xpoint, uint16_t Ypoint, const char Ascii_Char,
                    const sFONT* Font, uint16_t Color_Foreground, uint16_t Color_Background)
{
    uint16_t row, col;

    if ( Ascii_Char >= Font->index_table_size )
    	return 0;

    uint8_t index = Font->index_table[ (int)Ascii_Char ];
    uint8_t width = Font->width_table[ index ];
    uint16_t offset = Font->offset_table[ index ];
    const uint8_t *ptr = Font->data_table + offset;

    for (row = 0; row < Font->height; row ++ )
    {
        for (col = 0; col < width; col ++ )
        {

			if (*ptr & (0x80 >> (col % 8)))
			{
				_gl_iPlotPixelAbsoluteXY_min(Xpoint + col, Ypoint + row, Color_Foreground);
			}
			else
			if ( Color_Background != Color_Foreground )
			{
				_gl_iPlotPixelAbsoluteXY_min(Xpoint + col, Ypoint + row, Color_Background);
			}
            //One pixel is 8 bits
            if (col % 8 == 7)
                ptr++;
        }// Write a line
        if (width % 8 != 0)
            ptr++;
    	LCD_SetBandBit( Ypoint + row );
    }// Write all

    return width;
}
void _gl_DrawString(uint16_t Xstart, uint16_t Ystart, const char * pString,
                         const sFONT* Font, uint16_t Color_Foreground, uint16_t Color_Background)
{
    uint16_t Xpoint = Xstart;
    uint16_t Ypoint = Ystart;

    if (Xstart >= GL_PixelsX || Ystart >= GL_PixelsY)
        return;

    while (* pString != '\0')
    {
        Xpoint += _gl__DrawChar(Xpoint, Ypoint, * pString, Font, Color_Foreground, Color_Background);

        //The next character of the address
        pString ++;
    }
}

uint8_t _gl_CharWidth( const char ch, const sFONT *Font )
{
	return Font->width_table[ Font->index_table[ (int) ch ] ];
}
uint16_t _gl_StringWidth( const char *pString, const sFONT *Font )
{
	uint16_t width = 0;

	while ( *pString )
	{
		width += Font->width_table[ Font->index_table[ (int) *pString++ ] ];
	}

	return width;
}
