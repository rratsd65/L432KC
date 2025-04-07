/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <time.h>
#include <math.h>

#include "spi.h"
#include "../lcd/lcd.h"
#include "../fonts/fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef float float32_t;

typedef struct
{
	float32_t	x;
	float32_t	y;
} fVector;
typedef struct
{
	uint8_t		x;
	uint8_t		y;
} bVector;
typedef struct
{
	int32_t		x;
	int32_t		y;
} iVector;

//#define pos_new	pos_old
//#define vel_new vel_old
typedef struct
{
	fVector		pos_old;
	fVector		pos_new;
	fVector		vel_old;
	fVector		vel_new;

	bVector		disp;

	uint16_t	color;
} Boid;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern RNG_HandleTypeDef hrng;

#define _NumBoids	172
#define _ShowStats	1






/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t * _gl_cpCurrentFrameBuffer;

Boid boids[ _NumBoids ];
uint16_t visible_boids;

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timer_mainTask */
osTimerId_t timer_mainTaskHandle;
const osTimerAttr_t timer_mainTask_attributes = {
  .name = "timer_mainTask"
};
/* Definitions for mutex_spi1 */
osMutexId_t mutex_spi1Handle;
const osMutexAttr_t mutex_spi1_attributes = {
  .name = "mutex_spi1"
};
/* Definitions for sync_event */
osEventFlagsId_t sync_eventHandle;
const osEventFlagsAttr_t sync_event_attributes = {
  .name = "sync_event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void memset_w( void *dst, uint8_t val, uint16_t len );
void memset_16( void *dst, uint16_t val, uint16_t len );
void memmove_w( void *dst, void *src, uint16_t len );

void InitBoids( void );
void MoveBoids( void );
void DrawBoids( void );

void requestSPI(uint8_t fastSPI);
void releaseSPI(void);

/* USER CODE END FunctionPrototypes */

void main_task(void *argument);
void timer_callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutex_spi1 */
  mutex_spi1Handle = osMutexNew(&mutex_spi1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timer_mainTask */
  timer_mainTaskHandle = osTimerNew(timer_callback, osTimerPeriodic, (void*) &timer_mainTaskHandle, &timer_mainTask_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart( timer_mainTaskHandle, 15 );
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(main_task, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of sync_event */
  sync_eventHandle = osEventFlagsNew(&sync_event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_main_task */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_task */
void main_task(void *argument)
{
  /* USER CODE BEGIN main_task */

	InitBoids();

	requestSPI( 1 );

	LCD_Init( _LCDDriver_SSD1351 );
	_gl_iInitialize();
	LCD_EnableUpdates();

	releaseSPI();

	uint32_t passes = 0;
	uint64_t totalTicks=0;
	uint16_t minTicks=0xFFFF, maxTicks=0;
	TickType_t curTick, lastTick, elapsedTicks;
	char buff[20];
	char tbuff[12];

	extern TIM_HandleTypeDef htim7;

	elapsedTicks = 0;
	curTick = lastTick = osKernelGetTickCount();

	htim7.Instance->CNT = 0;
	htim7.Instance->CR1 |= 1;
	htim7.Instance->CR1 &= ~1;
	htim7.Instance->CR1 &= ~1;
	htim7.Instance->CNT = 0;
	htim7.Instance->CNT = 0;

	// first, make sure the event is cleared, then wait one more time for it to set again
	// (hitting immediately would be OK, but I don't want to)
	osEventFlagsWait( sync_eventHandle, EVENT_FLAG_MAIN_TASK_TIMER_HIT, osFlagsWaitAll, osWaitForever );
	osEventFlagsWait( sync_eventHandle, EVENT_FLAG_MAIN_TASK_TIMER_HIT, osFlagsWaitAll, osWaitForever );

	/* Infinite loop */
	for(;;)
	{
		uint32_t flags = osEventFlagsWait( sync_eventHandle, EVENT_FLAG_MAIN_TASK_TIMER_HIT, osFlagsWaitAny, 100 );

		if ( !(flags & EVENT_FLAG_MAIN_TASK_TIMER_HIT) )
		{
			continue;
		}



		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);				// PB3 to time the display

		LCD_WriteFrameBuffer();


		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);			// PA12 to time the math
		htim7.Instance->CNT = 0;
		htim7.Instance->CR1 |= 1;

		// update boid positions while DMA is busy sending frame buffer to display
		// we have *lots* of time here (~13.1 milliseconds)
		MoveBoids();

		htim7.Instance->CR1 &= ~1;
		uint32_t elapsedMicroseconds = htim7.Instance->CNT;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);			// PA12 to time the math




		// wait for frame buffer write to be done (DMA complete interrupt/event)
		do
		{
			flags = osEventFlagsWait( sync_eventHandle, EVENT_FLAG_LCD_DMA_COMPLETE, osFlagsWaitAll, 100 );
		}
		while ( !(flags & EVENT_FLAG_LCD_DMA_COMPLETE) );

	//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);			// PB3 to time the display



#if 0
		_gl_iSetColor( 0xFFFF );
		_gl_iFilledRectangleXY(0,0,GL_PixelsX-1,GL_PixelsY-1);
#else
		// erase the stats text
		_gl_iSetColor( GL_BLACK );
		_gl_iFilledRectangleXY(0,0,GL_PixelsX-1,7);
		_gl_iFilledRectangleXY(0,GL_PixelsY-8,GL_PixelsX-1,GL_PixelsY-1);

#define _TearTest 0
#if _TearTest
		static uint8_t col=0;
		uint8_t row;
		for ( row=0; row<GL_PixelsY; row++ )
		{
			*((uint16_t *)_gl_cpCurrentFrameBuffer + row * GL_PixelsX + col) = GL_BLACK;
		}
		col = (col + 1) & (GL_PixelsX-1);
#endif

		// now that the display isn't busy (and won't be for about 1.8 milliseconds),
		// go update the frame buffer:
		//   * clear old boid pix
		//   * set new boid pix
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);				// PB0 to time the drawing
		DrawBoids();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);				// PA12 to time the drawing

#if _TearTest
		for ( row=0; row<GL_PixelsY; row++ )
		{
			*((uint16_t *)_gl_cpCurrentFrameBuffer + row * GL_PixelsX + col) = GL_WHITE;
		}
#endif

#if _ShowStats
		// write some "math time" text
		uint16_t thisTicks = (uint16_t)elapsedMicroseconds;
		if ( thisTicks < minTicks )	minTicks = thisTicks;
		if ( thisTicks > maxTicks ) maxTicks = thisTicks;
		totalTicks += thisTicks;
		passes++;
		uint16_t avgTicks = (uint16_t)(uint32_t)((float32_t)totalTicks / (float32_t)passes + 0.5f);


		sprintf( buff, "%d", minTicks );
		_gl_DrawString(  0, 0, buff, &FONT_FixedSys_8x8, GL_GREEN, GL_GREEN );

		sprintf( buff, "%d", avgTicks );
		uint8_t strx = _gl_StringWidth( buff, &FONT_FixedSys_8x8 );
		_gl_DrawString( (GL_PixelsX-strx)/2, 0, buff, &FONT_FixedSys_8x8, GL_YELLOW, GL_YELLOW );

		sprintf( buff, "%d", maxTicks );
		strx = _gl_StringWidth( buff, &FONT_FixedSys_8x8 );
		_gl_DrawString( (GL_PixelsX-strx), 0, buff, &FONT_FixedSys_8x8, GL_RED, GL_RED );

		sprintf( buff, "%3d", visible_boids );
		_gl_DrawString( 0, GL_PixelsY-8, buff, &FONT_FixedSys_8x8, GL_BLUE, GL_BLUE );

		curTick = osKernelGetTickCount();
		elapsedTicks += curTick - lastTick;
		lastTick = curTick;
		static uint8_t lastSS = 0xFF;
		uint8_t SS, MM;
		uint16_t HH;
		uint32_t secs = elapsedTicks / 1000;
		HH = secs / 3600;
		MM = (secs % 3600) / 60;
		SS = secs % 60;
		if ( SS != lastSS )
		{
			lastSS = SS;
			sprintf( tbuff, "%4d:%02d:%02d", HH, MM, SS );
		}
		_gl_DrawString( GL_PixelsX-80, GL_PixelsY-8, tbuff, &FONT_FixedSys_8x8, GL_BLUE, GL_BLUE );
#endif
#endif

		// force display update
		LCD_SetBandMask( 0x1 );
	}


  /* USER CODE END main_task */
}

/* timer_callback function */
void timer_callback(void *argument)
{
  /* USER CODE BEGIN timer_callback */
	osTimerId_t *timerHandle = (osTimerId_t *)argument;

	if ( *timerHandle == timer_mainTaskHandle )
	{
		osEventFlagsSet( sync_eventHandle, EVENT_FLAG_MAIN_TASK_TIMER_HIT );
	}

  /* USER CODE END timer_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void requestSPI(uint8_t fastSPI)
{
	// defaults to 'fast' (20Mbps) in .IOC file
	static uint8_t prevFast = 1;

	osMutexAcquire( mutex_spi1Handle, osWaitForever );

	if ( fastSPI != prevFast )
	{
		// choose SPI clock rate:
		//   * 10 Mbps for nRF24L01 radio & clock rate of 10MHz
		//   * 20 Mbps for SSD1351 LCD controller @ 20MHz (max = 25MHz)
		hspi1.Instance->CR1 = ((hspi1.Instance->CR1 & ~(SPI_CR1_BR_Msk)) |
				(fastSPI ? SPI_BAUDRATEPRESCALER_4 : SPI_BAUDRATEPRESCALER_16) );

		prevFast = fastSPI;
	}
}
void releaseSPI(void)
{
	osMutexRelease( mutex_spi1Handle );
}
volatile uint16_t BreakCounter = 0;
void BreakHere( void )
{
	BreakCounter++;
}


void memset_w( void *dst, uint8_t val, uint16_t len )
{
	uint32_t *ptr_w = dst;
	uint16_t len_w = len / 4;
	// make a 32-bit version of the value
	uint32_t val_w =
			((uint32_t)val << 24) |
			((uint32_t)val << 16) |
			((uint32_t)val <<  8) |
			((uint32_t)val <<  0);

	while ( len_w-- )
	{
		*ptr_w++ = val_w;
	}
}
void memset_16( void *dst, uint16_t val, uint16_t len )
{
	uint32_t *ptr_w = dst;
	uint16_t len_w = len / 2;
	// make a 32-bit version of the value
	uint32_t val_w =
			((uint32_t)val << 16) |
			((uint32_t)val <<  0);

	while ( len_w-- )
	{
		*ptr_w++ = val_w;
	}
}
void memmove_w( void *dst, void *src, uint16_t len )
{
	uint32_t *dst_w = dst;
	uint32_t *src_w = src;
	uint16_t len_w = len / 4;

	while ( len_w-- )
	{
		*dst_w++ = *src_w++;
	}
}


// initialized data (vs. const) so they're in fast(er) RAM instead of FLASH
#if _TestBoids
const float32_t margin = 1.0f;
const float32_t attractrange = 1000.0f;
const float32_t avoidrange = 0.0f;
const float32_t maxspeed = -10000.0f;
const float32_t minspeed = -10000.0f;
#else
const float32_t margin = 100.0f;
const uint32_t attractrange = 60;
const uint32_t avoidrange = 8;
const float32_t maxspeed = 7.5f;
const float32_t minspeed = 3.0f;
#endif

const fVector Window = { -400.0f, 400.0f };	// visible window coord limit
const float32_t turnfactor = 0.22f;

const float32_t attractrange_squared = (attractrange * attractrange);
const float32_t avoidrange_squared = (avoidrange * avoidrange);

const float32_t centeringfactor = 0.0005f;
const float32_t avoidfactor = 0.05f;
const float32_t matchingfactor = 0.05f;

const float32_t maxbias = 0.1f;//0.01f;
const float32_t bias_increment = 0.00004f;
const float32_t biasval_1 = 0.001f, biasval_2 = 0.001;

inline uint32_t hrand(void) { return HAL_RNG_GetRandomNumber(&hrng) & RAND_MAX; }
inline uint16_t swapBE( uint16_t val ) { return ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8); }
#define VectorAdd( a1, a2, s ) { s.x=a1.x+a2.x; s.y=a1.y+a2.y; }
#define VectorSub( s1, s2, d ) { d.x=s1.x-s2.x; d.y=s1.y-s2.y; }
#define VectorDiv( dd, dr, q ) { q.x=dd.x/dr; q.y=dd.y/dr; }

__attribute__((always_inline)) static inline void vsqrt_f32( const float32_t in, float32_t *out )
{
    __ASM( "VSQRT.F32	%[result], %[input]"
    			: [result]"=t" (*out)
				: [input]"t" (in)
    	);
}
__attribute__((always_inline)) static inline void vabs_f32( const float32_t in, float32_t *out )
{
    __ASM( "VABS.F32	%[result], %[input]"
    			: [result]"=t" (*out)
				: [input]"t" (in)
		);
}
__attribute__((always_inline)) static inline void vcvt_f32_u32( const float32_t in, uint32_t *out )
{
    __ASM(	"VCVT.U32.F32	%[result], %[input]"
    			: [result]"=t" (*out)
				: [input]"t" (in)
		);
}
__attribute__((always_inline)) static inline void sum_squares_f32( const float32_t in1, const float32_t in2, float32_t *out )
{
	float32_t tmp;

	__ASM(	"VMUL.F32	%[result], %[inp1], %[inp2]"
    		: [result]"=t" (*out)
    		: [inp1]"t"(in1), [inp2]"t"(in1) );

	__ASM(	"VMUL.F32	%[result], %[inp1], %[inp2]"
    		: [result]"=t" (tmp)
    		: [inp1]"t"(in2), [inp2]"t"(in2) );

	__ASM(	"VADD.F32	%[result], %[inp1], %[inp2]"
    		: [result]"=t" (*out)
    		: [inp1]"t"(*out), [inp2]"t"(tmp) );
}
void InitBoids( void )
{
	uint16_t i;

	for ( i=0; i<_NumBoids; i++ )
	{
#define _SingleRed	1

#if !_SingleRed
		switch ( i % 4 )
		{
		default:
		case 0:	// upper left
			boids[i].color = swapBE( GL_RED );
			break;
		case 1:	// upper right
			boids[i].color = swapBE( GL_GREEN );
			break;
		case 2:	// lower left
			boids[i].color = swapBE( GL_BLUE );
			break;
		case 3:	// lower right
			boids[i].color = swapBE( GL_YELLOW );
			break;
		}
#else
		boids[i].color = GL_WHITE;
#endif

#if _TestBoids
		boids[i].vel_new.x = 0.0f;		boids[i].vel_old.x = boids[i].vel_new.x;
		boids[i].vel_new.y = 0.0f;		boids[i].vel_old.y = boids[i].vel_new.y;
		boids[i].pos_new.x = Window.y;	boids[i].pos_old.x = boids[i].pos_new.x;
		boids[i].pos_new.y = Window.y;	boids[i].pos_old.y = boids[i].pos_new.y;
#else
		// first, give them all random positions within the Window area
		float32_t random_x = ((float32_t)hrand() / (float32_t)RAND_MAX) * (Window.y - Window.x);
		float32_t random_y = ((float32_t)hrand() / (float32_t)RAND_MAX) * (Window.y - Window.x);
		boids[i].pos_new.x = Window.x + random_x;
		boids[i].pos_new.y = Window.x + random_y;
		boids[i].pos_old = boids[i].pos_new;
		boids[i].disp.x = 128;

		// now give them all random velocities
		boids[i].vel_new.x = ((float32_t)hrand() / (float32_t)RAND_MAX) * (maxspeed - minspeed) + minspeed * ((hrand() % 2) ? 1.0f : -1.0f);
		boids[i].vel_new.y = ((float32_t)hrand() / (float32_t)RAND_MAX) * (maxspeed - minspeed) + minspeed * ((hrand() % 2) ? 1.0f : -1.0f);
		// make sure we're not going too fast
		float32_t speed_squared = boids[i].vel_new.x * boids[i].vel_new.x + boids[i].vel_new.y * boids[i].vel_new.x;
		float32_t speed;
		vsqrt_f32( speed_squared, &speed );
		if ( speed > maxspeed )
		{
			boids[i].vel_new.x = (boids[i].vel_new.x / speed) * maxspeed;
			boids[i].vel_new.y = (boids[i].vel_new.y / speed) * maxspeed;
		}
		boids[i].vel_old = boids[i].vel_new;
#endif
	}

#if _SingleRed
	boids[i-1].color = swapBE( GL_RED );
#endif
}
void DrawBoids( void )
{
	uint16_t i;
	Boid *p;

	// first, erase all the old boid pixels
	for ( p = boids, i=0; i<_NumBoids; p++, i++ )
	{
		if ( !(0x80 & (p->disp.x | p->disp.y)) )
		{
			*((uint16_t *)_gl_cpCurrentFrameBuffer + p->disp.y * GL_PixelsX + p->disp.x) = GL_BLACK;
		}
	}

	visible_boids = 0;

	// re-draw the boid pixels and save the coords
	for ( p = boids, i=0; i<_NumBoids; p++, i++ )
	{
		// scale Window to 0 - GL_PixelsX/Y
		// ("outside display bounds" handled in draw code)
		p->disp.x = (uint8_t)(0xFF & (int32_t)(((p->pos_new.x - Window.x) * (float32_t)(GL_PixelsX - 1)) / (Window.y - Window.x)));
		p->disp.y = (uint8_t)(0xFF & (int32_t)(((p->pos_new.y - Window.x) * (float32_t)(GL_PixelsY - 1)) / (Window.y - Window.x)));

		if ( !(0x80 & (p->disp.x | p->disp.y)) )
		{
			*((uint16_t *)_gl_cpCurrentFrameBuffer + p->disp.y * GL_PixelsX + p->disp.x) = p->color;

			visible_boids++;
		}

		// copy new to old here so that they'll all be ready on the next Move call
		p->pos_old = p->pos_new;
		p->vel_old = p->vel_new;
	}
}

void MoveBoids( void )
{
	uint16_t i, j;
	Boid *p1, *p2;

	for ( p1=boids, i=0; i<_NumBoids; p1++, i++ )
	{
		fVector attract_avg_pos = { 0.0f, 0.0f };
		fVector attract_avg_vel = { 0.0f, 0.0f };
		fVector dist_avoids = { 0.0f, 0.0f };
		uint16_t boids_within_attract_range = 0;

		for ( p2=boids, j=0; j<_NumBoids; p2++, j++ )
		{
			if ( j != i )
			{
				// distance vector from current p1 to p2
				fVector dist;
				VectorSub( p1->pos_old, p2->pos_old, dist );

				// get square of distance for comparison
				float32_t dist_squared = (dist.x * dist.x + dist.y * dist.y);

				// comparisons done in this order so that it's "sometimes 2" vs. "always 2"
				// if within "attract" range
				if ( dist_squared < avoidrange_squared )
				{
					// sum up distances to nearby boids
					VectorAdd( dist_avoids, dist, dist_avoids )
				}
				else
				// otherwise, if within attract range
				if ( dist_squared < attractrange_squared )
				{
					// Add other (p2)  boid's x/y-coord and x/y vel to accumulator variables
					VectorAdd( attract_avg_pos, p2->pos_old, attract_avg_pos );
					VectorAdd( attract_avg_vel, p2->vel_old, attract_avg_vel );

					// Increment number of boids within attract range
					boids_within_attract_range++;
				}
			}
		}

		// copy old velocity to new so we can update new
		// (don't need to copy old position, since we'll create a new value at the end)
		p1->vel_new = p1->vel_old;

		// if any were in attract range of this boid
		// this doesn't include boids that were within the avoid range
		if ( boids_within_attract_range )
		{
			// divide acc vars by neighbor count
			VectorDiv( attract_avg_pos, (float32_t)boids_within_attract_range, attract_avg_pos );
			VectorDiv( attract_avg_vel, (float32_t)boids_within_attract_range, attract_avg_vel );

			// first, subtract this boid's position and velocity from the averages,
			// since we want deltas from this boid's values
			VectorSub( attract_avg_pos, p1->pos_old, attract_avg_pos );
			VectorSub( attract_avg_vel, p1->vel_old, attract_avg_vel );

			// now multiply/accumulate
			p1->vel_new.x += attract_avg_pos.x * centeringfactor + attract_avg_vel.x * matchingfactor;
			p1->vel_new.y += attract_avg_pos.y * centeringfactor + attract_avg_vel.y * matchingfactor;
		}

		// add avoidance contribution to velocity
		p1->vel_new.x += dist_avoids.x * avoidfactor;
		p1->vel_new.y += dist_avoids.y * avoidfactor;

		// turn away from window boundary based on position
		if ( p1->pos_old.x < Window.x + margin )		p1->vel_new.x += turnfactor;
		else
		if ( p1->pos_old.x > Window.y - margin )		p1->vel_new.x -= turnfactor;
		if ( p1->pos_old.y < Window.x + margin )		p1->vel_new.y += turnfactor;
		else
		if ( p1->pos_old.y > Window.y - margin )		p1->vel_new.y -= turnfactor;

		// make sure we're not going too fast or too slow
		float32_t speed = sqrtf( p1->vel_new.x * p1->vel_new.x + p1->vel_new.y * p1->vel_new.y );
		if ( speed < minspeed )
		{
			p1->vel_new.x = (p1->vel_new.x / speed) * minspeed;
			p1->vel_new.y = (p1->vel_new.y / speed) * minspeed;
		}
		else
		if ( speed > maxspeed )
		{
			p1->vel_new.x = (p1->vel_new.x / speed) * maxspeed;
			p1->vel_new.y = (p1->vel_new.y / speed) * maxspeed;
		}

		// move the boid
		VectorAdd( p1->pos_old, p1->vel_new, p1->pos_new );
	}
}

#if _TestBoids
volatile uint64_t totalTicks=0;
volatile uint32_t minTicks=0xFFFFFFFFL, maxTicks=0L, avgmicro;

uint32_t TestBoids( void )
{
	extern TIM_HandleTypeDef htim7;

	// ensure TIM7 is not running and CNT is 0
	htim7.Instance->CR1 &= ~1;
	htim7.Instance->CNT = 0;
	htim7.Instance->CR1 |= 1;
	htim7.Instance->CR1 &= ~1;
	htim7.Instance->CNT = 0;

	// don't let timer 1 run (1ms timer)
	extern TIM_HandleTypeDef htim1;
	htim1.Instance->CR1 &= ~1;

//		InitBoids();

#define _MaxTestLoops	100L		// 100k loops @ 6.5ms = ~11 minutes
	uint32_t loops;
	for ( loops=0; loops < _MaxTestLoops; loops++ )
	{
		InitBoids();

		htim7.Instance->CNT = 0;
		htim7.Instance->CR1 |= 1;

		MoveBoids();

		htim7.Instance->CR1 &= ~1;
		uint32_t thisTicks = htim7.Instance->CNT;

		if ( thisTicks < minTicks )	minTicks = thisTicks;
		if ( thisTicks > maxTicks ) maxTicks = thisTicks;
		totalTicks += thisTicks;

		DrawBoids();
	}
	avgmicro = totalTicks / _MaxTestLoops;

	return avgmicro;
}
#endif

/* USER CODE END Application */

