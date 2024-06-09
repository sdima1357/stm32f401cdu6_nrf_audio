/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#ifndef USBD_AUDIO_FREQ
#define USBD_AUDIO_FREQ 44100
#endif
#define LOW_LATENCY

#ifdef LOW_LATENCY
#define SPDIF_FRAMES (192/8)
#else
#define SPDIF_FRAMES (192)
#endif
#define LINEAR_INTERPOLATION

#define NUM_RCV_BFRS 4
/*
 * best config
#define NUM_RCV_BFRS 4
#define SPDIF_FRAMES (192/8)
#define TIME_FILT_LEN 6
int timeForRecivedSamples_mean[TIME_FILT_LEN];

#define TIME_BIT_SCALE_FACT 17
#define TIME_SCALE_FACT     (1<<TIME_BIT_SCALE_FACT)

#define MAX_CORR  (TIME_SCALE_FACT>>12)
float scaleOffs = 1;
float scaleDiff = 2;
*/
#define OLD
#ifdef OLD
#define TIME_FILT_LEN 6
int timeForRecivedSamples_mean[TIME_FILT_LEN];

#define TIME_BIT_SCALE_FACT 17
#define TIME_SCALE_FACT     (1<<TIME_BIT_SCALE_FACT)

#define MAX_CORR  (TIME_SCALE_FACT>>12)
float scaleOffs = 1;
float scaleDiff = 2;
#else
#define TIME_FILT_LEN 6
int timeForRecivedSamples_mean[TIME_FILT_LEN];

#define TIME_BIT_SCALE_FACT 18
#define TIME_SCALE_FACT     (1<<TIME_BIT_SCALE_FACT)

#define MAX_CORR  (TIME_SCALE_FACT>>14)
float scaleOffs = 2;
float scaleDiff = 1;
#endif
//float scaleOffs = 1;
//float scaleDiff = 2;

#define TIMER_CLOCK_FREQ 1000000.0f



void delay_uS(int mkS)
{
	uint16_t lastAudioUsbTimeStampNew = TIM3->CNT+mkS;
	while(((uint16_t)(TIM3->CNT - lastAudioUsbTimeStampNew))&0x8000);
}



#define NUM_PAKS_IN_SUP 25
#define CHANEL_NUM      76
#define PAYLOAD 32
#define PACKET_SIZE (PAYLOAD-1)
static const uint8_t tx_address[5] = {0x17,0x27,0x37,0x47,0x57};



//#ifdef  USE_SPDIF
//#define N_SIZE      (SPDIF_FRAMES*4)
//#else
//#define N_SIZE_BITS (8)
//#define N_SIZE (1<<N_SIZE_BITS)
//#endif
#define N_SIZE      (SPDIF_FRAMES*4)

//#define N_SIZE_BITS (8)
//#define N_SIZE (1<<N_SIZE_BITS)

//#define N_SIZE      (SPDIF_FRAMES*4)
#define ASBUF_SIZE     (N_SIZE*2)
//__attribute__((section(".noncacheable")))
int16_t baudio_buffer[ASBUF_SIZE*2];

#define INTERP_BITS  8
enum modess {NONE=-1,PWM=0,I2S,SPDIF};

enum modess OUT_MODE = NONE;
int FREQ = 44100;
int MAX_VOL = 65536;


void retarget_put_char(char p)
{
	HAL_UART_Transmit(&huart1, &p, 1, 0xffffff); // send message via UART
}
#define BSIZE 0x2000
__attribute__((section(".noncacheable"))) volatile int8_t circ_buff[BSIZE];
volatile int c_write = 0;
volatile int c_read  = 0;

uint16_t crsize()
{
	return (c_write-c_read)&(BSIZE-1);
}

int  crget(char* r)
{
	if(!crsize()) return 0;
	*r = circ_buff[c_read];
	c_read ++;
	c_read&=BSIZE-1;
	return 1;

}

void cr_flush()
{
	char rr;
	//int ii = 1000;
	while(crsize())
	{
		crget(&rr),HAL_UART_Transmit(&huart1, &rr, 1, 0xffffff); // send message via UART
	}
}
void cr_flush_par()
{
	char rr;
	//int ii = 1000;
	if(crsize())
	{
		crget(&rr),HAL_UART_Transmit(&huart1, &rr, 1, 0xffffff); // send message via UART
	}
}

void crput(char r)
{
	if(crsize()<(BSIZE-1))
	{
		circ_buff[c_write] = r;
		c_write++;
		c_write&=BSIZE-1;
	}
}


int _write(int fd, char* ptr, int len)
{
    (void)fd;
    int i = 0;
    while (ptr[i] && (i < len))
    {
    	if (ptr[i] == '\r')
    	{

    	}
    	else
    	{
    		crput(ptr[i]);
			//retarget_put_char((int)ptr[i]);
			if (ptr[i] == '\n')
			{
				crput('\r');
				//retarget_put_char((int)'\r');
			}
    	}
        i++;
    }
    return len;
}
#include "tm_stm32f1_nrf24l01.h"
//#include "nrf24l01.h"


extern SPI_HandleTypeDef hspi1;

uint8_t  old_counter = 0;

int cntOk;
int cntLost;

int nrfOnline=0;

//static const uint8_t rx_address[5] = {0x1e,0x2e,0x3e,0x4e,0x5e};
volatile int  iReady = 1;

#define RCV_SIZE ( PACKET_SIZE*(NUM_PAKS_IN_SUP-1))
uint8_t rcv_buff[RCV_SIZE*NUM_RCV_BFRS];
volatile int rcv_buff_write = 0;


struct sAuPacket
{
	uint8_t num;
	uint8_t data[PACKET_SIZE];
} APacketQ[NUM_PAKS_IN_SUP];

struct sAuPacket ATemp;



int  median3(int  a, int  b, int  c)
{
   return (b<a)
              ?   (b<c)  ?  (c<a) ? c : a  :  b
              :   (a<c)  ?  (c<b) ? c : b  :  a;
}

int tfl_mean[3];
int pnt_mean = 0;
int       lastDmaPos;
uint16_t  lastDmaAccessTime;
uint32_t  readPositionXScaled  = 0;  //readPosition<<12;
uint32_t  readSpeedXScaled     = 1;
volatile int UsbSamplesAvail = 0;
uint32_t  samplesInBuffScaled  = 0;

uint32_t   samplesInBuff        = 0;
int32_t   samplesInBuffH       = 0;
void * usb_SndBuffer        = 0;
#define VOL_BITS   6
#define VOL_SCALE  (1<<VOL_BITS)

int volume = VOL_SCALE;

int corrQ[3];
int corrQpos = 0;
int bytesPerSampleLR = 4;
void AUDIO_OUT_Start(void* pBuffer, uint32_t Size,int sampleBits)
{
	printf("AUDIO_OUT_Start %p Size = %d sampleBits=%d DMA_B %d \n",pBuffer,Size,sampleBits,ASBUF_SIZE*4);
	bytesPerSampleLR = 2*sampleBits/8;
	samplesInBuff = Size/bytesPerSampleLR; //(L & R channels short)
	samplesInBuffH = samplesInBuff/2;

	usb_SndBuffer = pBuffer;
	//AUDIO_OUT_Play_Counter++;

	samplesInBuffScaled = samplesInBuff<<TIME_BIT_SCALE_FACT;
	readPositionXScaled = samplesInBuffScaled/2;
	int timeForRecivedSamples = samplesInBuff*TIMER_CLOCK_FREQ/USBD_AUDIO_FREQ;
	for(int k=0;k<TIME_FILT_LEN;k++)
	{
		timeForRecivedSamples_mean[k] = timeForRecivedSamples;
	}
	//inputSpeed = samplesInBuff*TIMER_CLOCK_FREQ/timeForRecivedSamples
}
uint16_t  lastAudioUsbTimeStamp;


int pnt_timeForRecivedSamples_mean = 0;

float inputSpeed = 0;
int prevPos = -1 ;
int appDistErr = 0;
int sForcedSpeed = 0;
float outSpeed;

struct STD_VAR
{
	float summ;
	float summQ;
	int cnt;
}sPos,sDif;
void clearV(struct STD_VAR * pnt)
{
	pnt->summ = 0;
	pnt->summQ = 0;
	pnt->cnt = 0;
};
void addV(struct STD_VAR * pnt,float val)
{
	pnt->summ+= val;
	pnt->summQ+= val*val;
	pnt->cnt++;
}
void calcV(struct STD_VAR * pnt,float* pmean,float*pvariance)
{
	float div = (pnt->cnt==0)?1.0f:(1.0f/pnt->cnt);
	float mean = pnt->summ *div;
	float variance = pnt->summQ *div  - mean * mean;
	*pmean = mean;
	*pvariance =  sqrtf(variance);
}
int forceCounter = 0;

#ifdef OLD
void AUDIO_OUT_Periodic(uint16_t* pBuffer)
{
	//AUDIO_PeriodicTC_FS_Counter++;
	if(!usb_SndBuffer || OUT_MODE == NONE) return ;
	int cPos  = (pBuffer - (uint16_t*)usb_SndBuffer)/2;
	uint16_t lastAudioUsbTimeStampNew = TIM3->CNT;
	if(cPos==0)
	{

		uint16_t timeForRecivedSamples = lastAudioUsbTimeStampNew - lastAudioUsbTimeStamp;

		timeForRecivedSamples_mean [pnt_timeForRecivedSamples_mean++] = timeForRecivedSamples;
		pnt_timeForRecivedSamples_mean%=TIME_FILT_LEN;


        lastAudioUsbTimeStamp = lastAudioUsbTimeStampNew;

		//


		//int  timeForRecivedSamples = median3(timeForRecivedSamples_mean[0],timeForRecivedSamples_mean[1],timeForRecivedSamples_mean[2]);
		int summ_t = timeForRecivedSamples_mean[0];
		int max_t = timeForRecivedSamples_mean[0];
		int min_t = timeForRecivedSamples_mean[0];
		for(int k=1;k<TIME_FILT_LEN;k++)
		{
			if(timeForRecivedSamples_mean[k]<min_t) min_t =timeForRecivedSamples_mean[k];
			if(timeForRecivedSamples_mean[k]>max_t) max_t =timeForRecivedSamples_mean[k];
			summ_t+= timeForRecivedSamples_mean[k];
		}
		summ_t -=max_t+min_t;
		summ_t/= TIME_FILT_LEN-2;
		timeForRecivedSamples  = summ_t;
		//printf("timeForRecivedSamples %d\n");
		//(TIMER_CLOCK_FREQ*(float)outDmaSpeedScaled)/TIME_SCALE_FACT;
		if(timeForRecivedSamples)
		{
			inputSpeed = samplesInBuff*TIMER_CLOCK_FREQ/timeForRecivedSamples;
			///*
#if 0
			float outSpeed = (TIMER_CLOCK_FREQ*(float)outDmaSpeedScaled)/TIME_SCALE_FACT;
			//float outSpeed = OUT_MODE==SPDIF?(FREQ/4):FREQ;
			int sForcedSpeedA = (int)(inputSpeed*TIME_SCALE_FACT/outSpeed);
			int res = readSpeedXScaled-sForcedSpeedA;
			if(res<0) res = -res;
			if((res*100/sForcedSpeedA)>20)
			{
				printf("Force A!!!\n");
				sForcedSpeed = sForcedSpeedA;
				readSpeedXScaled = sForcedSpeed;
				readPositionXScaled = samplesInBuffScaled/2;
			}
#endif
			//*/

		}


		//sForcedSpeed = (int)(samplesInBuff*TIME_SCALE_FACT*mean/(timeForRecivedSamples*N_SIZE);

		uint16_t timeFromLastDMA = lastAudioUsbTimeStampNew - lastDmaAccessTime;


		//where i am ?

		//int approximateSamplesOutedFromLastDMA  = 0;//((float)timeFromLastDMA/TIMER_CLOCK_FREQ)*inputSpeed;
		int approximateSamplesOutedFromLastDMA  = ((float)timeFromLastDMA/TIMER_CLOCK_FREQ)*inputSpeed;



		int appDistance  =  (int)(lastDmaPos + approximateSamplesOutedFromLastDMA )-cPos;
		appDistance = (appDistance+samplesInBuff)%samplesInBuff;

		int offset_C = appDistance - samplesInBuffH;

        if(UsbSamplesAvail>0)
		{

        	if(timeForRecivedSamples)
        	{
				int diff_C = appDistance - prevPos;
				while(diff_C>samplesInBuffH)  diff_C-=samplesInBuff;
				while(diff_C<-samplesInBuffH) diff_C+=samplesInBuff;
				appDistErr = offset_C;
				addV(&sPos,offset_C);
				addV(&sDif,diff_C);

			//	int fact = outSpeed/(inputSpeed+1);
				int corr = (offset_C*scaleOffs + diff_C*scaleDiff);
				//corrQ[corrQpos] = corr; corrQpos =(corrQpos+1)%3;
				//corr = median3(corrQ[0],corrQ[1],corrQ[2]);
				//printf("err %d\n",err);
				if(offset_C > samplesInBuffH/4 || offset_C <-samplesInBuffH/4 ) //seems completely lost sync , force set frequency
				{
#if 1
					//float outSpeed = OUT_MODE==SPDIF?(FREQ/4):FREQ;
					//printf("forceB %d %d %d %d \n",appDistErr,appDistance,lastDmaPos,approximateSamplesOutedFromLastDMA);
					sForcedSpeed = (int)(inputSpeed*TIME_SCALE_FACT/outSpeed);
					readSpeedXScaled = sForcedSpeed;
					forceCounter ++;
					readPositionXScaled = (samplesInBuffScaled/2+3*readPositionXScaled)/4;
#endif
				}
				else
				{
					//ok - only phase tune
/*
					if(dC + (err/16)>0)
					{
						readSpeedXScaled--;
					}
					else
					{
						readSpeedXScaled++;
					}
*/

					if(corr > MAX_CORR)
					{
						corr = MAX_CORR;
					}
					if(corr < -MAX_CORR)
					{
						corr  =-MAX_CORR;
					}

/*
					if(corr<32&& corr>-32)
					{
					}
					else
					{
						corr/=8;
					}
*/
					readSpeedXScaled -= corr;
					//readSpeedXScaled -= (dC*4 + (err/4));
					//readSpeedXScaled -= ((dC*16 + (err/32)));// + 8*err/samplesInBuff ;//- ((err>0)?1:(err<0)?-1:0);
					//readSpeedXScaled -= ((dC*32 + (err/8)));// + 8*err/samplesInBuff ;//- ((err>0)?1:(err<0)?-1:0);
				}
        	}
		}
        else
        {
        	readPositionXScaled = samplesInBuffScaled/2;
        }
		prevPos =  appDistance;
		int outSpeed = OUT_MODE==SPDIF?(FREQ/4):FREQ;
		UsbSamplesAvail = 8.0f*samplesInBuff*(outSpeed+1)/(inputSpeed+1);
	}

}
#else

void AUDIO_OUT_Periodic(uint8_t* pBuffer)
{
	//AUDIO_PeriodicTC_FS_Counter++;
	if(!usb_SndBuffer || OUT_MODE == NONE) return ;
	uint16_t lastAudioUsbTimeStampNew = TIM3->CNT;
	int cPos  = (pBuffer - (uint8_t*)usb_SndBuffer)/bytesPerSampleLR;
	{
	int outSpeed = OUT_MODE==SPDIF?(FREQ/4):FREQ;
	UsbSamplesAvail = 8.0f*samplesInBuff*(outSpeed+1)/(inputSpeed+1);
	}

		uint16_t timeForRecivedSamples = lastAudioUsbTimeStampNew - lastAudioUsbTimeStamp;

		timeForRecivedSamples_mean [pnt_timeForRecivedSamples_mean++] = timeForRecivedSamples;
		pnt_timeForRecivedSamples_mean%=TIME_FILT_LEN;


        lastAudioUsbTimeStamp = lastAudioUsbTimeStampNew;


		//int  timeForRecivedSamples = median3(timeForRecivedSamples_mean[0],timeForRecivedSamples_mean[1],timeForRecivedSamples_mean[2]);
		int summ_t = timeForRecivedSamples_mean[0];
		int max_t = timeForRecivedSamples_mean[0];
		int min_t = timeForRecivedSamples_mean[0];
		for(int k=1;k<TIME_FILT_LEN;k++)
		{
			if(timeForRecivedSamples_mean[k]<min_t) min_t =timeForRecivedSamples_mean[k];
			if(timeForRecivedSamples_mean[k]>max_t) max_t =timeForRecivedSamples_mean[k];
			summ_t+= timeForRecivedSamples_mean[k];
		}
		summ_t -=max_t+min_t;
		summ_t/= TIME_FILT_LEN-2;
		timeForRecivedSamples  = summ_t;

		inputSpeed = samplesInBuff*TIMER_CLOCK_FREQ/timeForRecivedSamples/NUM_RCV_BFRS;

		uint16_t timeFromLastDMA = lastAudioUsbTimeStampNew - lastDmaAccessTime;


		//where i am ?

		//int approximateSamplesOutedFromLastDMA  = 0;//((float)timeFromLastDMA/TIMER_CLOCK_FREQ)*inputSpeed;
		int approximateSamplesOutedFromLastDMA  = inputSpeed*timeFromLastDMA/TIMER_CLOCK_FREQ;

		int appDistance  = lastDmaPos + approximateSamplesOutedFromLastDMA-cPos;

		while(appDistance<0)
			appDistance+=samplesInBuff;
		while(appDistance>=samplesInBuff)
			appDistance-=samplesInBuff;


		int offset_C     = appDistance - samplesInBuffH;

        if(UsbSamplesAvail>0)
		{
				int diff_C = appDistance - prevPos;
				while(diff_C>samplesInBuffH)
					diff_C-=samplesInBuff;
				while(diff_C<-samplesInBuffH)
					diff_C+=samplesInBuff;

				appDistErr = offset_C;
				addV(&sPos,offset_C);
				addV(&sDif,diff_C);

				int corr = (offset_C*scaleOffs + diff_C*scaleDiff);
				if(offset_C > samplesInBuffH/4 || offset_C <-samplesInBuffH/4 ) //seems completely lost sync , force set frequency
				{
#if 1
					//printf("forceB %d %d %d %d \n",appDistErr,appDistance,lastDmaPos,approximateSamplesOutedFromLastDMA);
					sForcedSpeed = (int)(inputSpeed*TIME_SCALE_FACT/outSpeed);
					readSpeedXScaled = sForcedSpeed;
					forceCounter ++;
					readPositionXScaled = samplesInBuffScaled/2;
					//readPositionXScaled = (samplesInBuffScaled/2+3*readPositionXScaled)/4;
#endif
				}
				else
				{
					if(corr > MAX_CORR)
					{
						corr = MAX_CORR;
					}
					if(corr < -MAX_CORR)
					{
						corr  =-MAX_CORR;
					}
					readSpeedXScaled -= corr;
				}
		}
        else
        {
        	if(!cPos)
        	readPositionXScaled = samplesInBuffScaled/2;
        }
		prevPos =  appDistance;
}
#endif
#if 0
void buff_init(uint8_t* buffer,int buffer_size)
{
	int bytesPerSampleLR = 2*sampleBits/8;
	samplesInBuff = Size/bytesPerSampleLR; //(L & R channels short)
	samplesInBuffH = samplesInBuff/2;

	usb_SndBuffer = pBuffer;
	AUDIO_OUT_Play_Counter++;

	samplesInBuffScaled = samplesInBuff<<TIME_BIT_SCALE_FACT;
	readPositionXScaled = samplesInBuffScaled/2;
}
#endif

struct LR
{
	int32_t L;
	int32_t R;
};
struct LR16
{
	int16_t l;
	int16_t r;
};
struct LR32
{
	uint8_t bts[8];
};


inline struct LR getNextSampleLR32B()
{
	struct LR res;
	int32_t L  = 0;
	int32_t R  = 0;
	if(UsbSamplesAvail)
	{
		//HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		readPositionXScaled += readSpeedXScaled;
		if(readPositionXScaled >= samplesInBuffScaled)
		{
			readPositionXScaled -= samplesInBuffScaled;
		}

		uint32_t readPositionIntC = readPositionXScaled>>TIME_BIT_SCALE_FACT;
#ifdef B24
			struct LR24 lrc = ((struct LR24*)usb_SndBuffer)[readPositionIntC];
			{
				uint8_t * LP = &L;
				uint8_t * RP = &R;
				LP[1] = lrc.bts[0];
				LP[2] = lrc.bts[1];
				LP[3] = lrc.bts[2];

				RP[1] = lrc.bts[3];
				RP[2] = lrc.bts[4];
				RP[3] = lrc.bts[5];
			}
			L>>=8;
			R>>=8;
#ifdef LINEAR_INTERPOLATION
			uint32_t readPositionIntN = readPositionIntC+1;
			if(readPositionIntN>=samplesInBuff)
			{
				readPositionIntN -= samplesInBuff;
			}
			int32_t LS  = 0;
			int32_t RS  = 0;
			struct LR24 lrcs = ((struct LR24*)usb_SndBuffer)[readPositionIntN];
			{
				uint8_t * LP = &LS;
				uint8_t * RP = &RS;
				LP[1] = lrcs.bts[0];
				LP[2] = lrcs.bts[1];
				LP[3] = lrcs.bts[2];

				RP[1] = lrcs.bts[3];
				RP[2] = lrcs.bts[4];
				RP[3] = lrcs.bts[5];
			}
			LS>>=8;
			RS>>=8;

			int32_t W1  =  (readPositionXScaled>>(TIME_BIT_SCALE_FACT-INTERP_BITS)) & ((1<<INTERP_BITS)-1);
			int32_t W  =  (1<<INTERP_BITS)-W1;
			L = (volume*(((L)*W+(LS)*W1)>>INTERP_BITS)/VOL_SCALE);
			R = (volume*(((R)*W+(RS)*W1)>>INTERP_BITS)/VOL_SCALE);
#else
			L = volume*L/VOL_SCALE;
			R = volume*R/VOL_SCALE;
#endif
#else
		struct LR16 lrc =((struct LR16*)usb_SndBuffer)[readPositionIntC];
		L  = lrc.l;
		R  = lrc.r;
#ifdef LINEAR_INTERPOLATION
		uint32_t readPositionIntN = readPositionIntC+1;
		if(readPositionIntN>=samplesInBuff)
		{
			readPositionIntN -= samplesInBuff;
		}
		struct LR16 lrcs =((struct LR16*)usb_SndBuffer)[readPositionIntN];
		int32_t LS  = lrcs.l;
		int32_t RS  = lrcs.r;

		//int16_t LS  = usb_SndBuffer[readPositionIntN*2+0];//+usb_SndBuffer[readPositionIntN*2+0];
		//int16_t RS  = usb_SndBuffer[readPositionIntN*2+1];//+usb_SndBuffer[readPositionIntN*2+1];

		int32_t W1  =  (readPositionXScaled>>(TIME_BIT_SCALE_FACT-INTERP_BITS)) & ((1<<INTERP_BITS)-1);
		int32_t W  =  (1<<INTERP_BITS)-W1;
		L = 256*(volume*((L*W+LS*W1)>>INTERP_BITS)/VOL_SCALE);
		R = 256*(volume*((R*W+RS*W1)>>INTERP_BITS)/VOL_SCALE);
#else
		L = 256*(volume*L/VOL_SCALE);
		R = 256*(volume*R/VOL_SCALE);
#endif
#endif

	}
	res.L = L;
	res.R = R;
	return res;
}


void checkTime()
{
	uint16_t prevTime;
	uint16_t  tfl;
	prevTime = lastDmaAccessTime;
	lastDmaAccessTime = TIM3->CNT;
	//lastDmaPos  = readPositionXScaled>>TIME_BIT_SCALE_FACT;
	//tfl = lastDmaAccessTime -prevTime;
	//outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE/(tfl);

	lastDmaPos  = readPositionXScaled>>TIME_BIT_SCALE_FACT;
	tfl      = lastDmaAccessTime -prevTime;

	tfl_mean[pnt_mean] = tfl;
	pnt_mean++;
	if(pnt_mean>2) pnt_mean =0;

    int mean = median3(tfl_mean[0],tfl_mean[1],tfl_mean[2]);
    if (mean)
    {
		float outDmaSpeedScaled;

		if(OUT_MODE==SPDIF)
		{
			outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE/4/(mean);
		}
		else
		{
			outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE/(mean);
		}
		outSpeed = (TIMER_CLOCK_FREQ*(float)outDmaSpeedScaled)/TIME_SCALE_FACT;
    }
   // else
  //  {
    //	outDmaSpeedScaled =  TIME_SCALE_FACT*N_SIZE/(mean);
  //  }
}

void i2s_fill(struct LR32 * bfr)
{
	for(int k=0;k<ASBUF_SIZE/4;k++)
	{
		struct LR bf= getNextSampleLR32B(/*k+ASBUF_SIZE/4*/);
		struct LR32* bfp =(struct LR32*)&bf;
#if 1
		bfr[k].bts[0] = bfp->bts[1];
		bfr[k].bts[1] = bfp->bts[2];
		bfr[k].bts[2] = 0;
		bfr[k].bts[3] = bfp->bts[0];

		bfr[k].bts[4] = bfp->bts[1+4];
		bfr[k].bts[5] = bfp->bts[2+4];
		bfr[k].bts[6] = 0;
		bfr[k].bts[7] = bfp->bts[0+4];
#else
		bfr[k].bts[0] =0;
		bfr[k].bts[1] =0;
		bfr[k].bts[2] = (bf.L>>16)&0xff;
		bfr[k].bts[3] = 0;
		bfr[k].bts[4] = 0;
		bfr[k].bts[5] = 0;
		bfr[k].bts[6] = (bf.R>>16)&0xff;
		bfr[k].bts[7] = 0;
#endif
		//bfr[k].L = (int)bf.l;
		//bfr[k].R = (int)bf.r;
	}
}
int TransferComplete_CallBack_FS_Counter = 0;
int HalfTransfer_CallBack_FS_Counter = 0;

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    checkTime();
	TransferComplete_CallBack_FS_Counter++;
	    if(UsbSamplesAvail > N_SIZE/2)
	    {
	    	UsbSamplesAvail -= N_SIZE/2;
	    }
	    else
	    {
	    	UsbSamplesAvail = 0;
	    }
	  //  LR16 getNextSampleLR16()
#if 0
		struct LR16 * bfr = ((struct LR16 *) baudio_buffer)+ASBUF_SIZE/4;
		for(int k=0;k<ASBUF_SIZE/4;k++)
		{
			bfr[k] = getNextSampleLR16(/*k+ASBUF_SIZE/4*/);

		}
#else
		struct LR32 * bfr = ((struct LR32 *) baudio_buffer)+ASBUF_SIZE/4;
		i2s_fill(bfr);
#endif
}

//void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    HalfTransfer_CallBack_FS_Counter++;
	    if(UsbSamplesAvail > N_SIZE/2)
	    {
	    	UsbSamplesAvail -= N_SIZE/2;
	    }
	    else
	    {
	    	UsbSamplesAvail = 0;
	    }
#if 0
		struct LR16 * bfr = ((struct LR16 *) baudio_buffer);
		for(int k=0;k<ASBUF_SIZE/4;k++)
			bfr[k] = getNextSampleLR16(/*k*/);
#else
		struct LR32 * bfr = ((struct LR32 *) baudio_buffer);
		i2s_fill(bfr);
#endif
}

//stm32f401       	PCM1502
//PB15 I2S_SD     	DIN
//PB12 I2S_SW     	LCK
//PB10 I2S_CK     	BCK
//GND  (MUST BE 0)  SCK
//GND    			GND
//+5                VIN

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
	printf("HAL_I2S_ErrorCallback !!!!!!!!!!!!!!!\n");
  //BSP_AUDIO_OUT_Error_CallBack();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//printf("HAL_GPIO_EXTI_Callback %d\n",GPIO_Pin);
	if(GPIO_Pin==NRF_IRQ_Pin)
	{
		iReady = 1;
		//clearInt();
		//nrf_irq_handler(&nrf);
	}
	//if(GPIO_Pin==NRF1_IRQ_Pin)
	{
		//nrf_irq_handler(&nrf1);
	}
}
void reinit_vars(enum modess mode)
{
	OUT_MODE = mode;
	HAL_I2S_DMAStop(&hi2s2);

	FREQ = I2S_AUDIOFREQ_96K;
	outSpeed = FREQ;
	HAL_I2S_DeInit(&hi2s2);
	  hi2s2.Instance = SPI2;
	  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
	  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	  hi2s2.Init.AudioFreq = FREQ;
	  hi2s2.Init.CPOL = I2S_CPOL_LOW;
	  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	readSpeedXScaled     = 1.001*TIME_SCALE_FACT*USBD_AUDIO_FREQ/(float)FREQ;
	HAL_I2S_Transmit_DMA(&hi2s2, baudio_buffer,ASBUF_SIZE);

}
void example()
{
    //uint32_t rx_data;
	 TIM3->PSC  = (int)(HAL_RCC_GetSysClockFreq()/TIMER_CLOCK_FREQ)-1;
	  HAL_TIM_Base_Start(&htim3);

    	nrfOnline = nRF24_Check();
    	printf("nrfOnline %d\n",nrfOnline);cr_flush();
		TM_NRF24L01_Init(CHANEL_NUM, PAYLOAD);
		/* Set 2MBps data rate and -18dBm output power */
		TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_2M, TM_NRF24L01_OutputPower_0dBm);

		/* Set my address, 5 bytes */
		TM_NRF24L01_SetMyAddress(tx_address);

		/* Set TX address, 5 bytes */
		TM_NRF24L01_SetTxAddress(tx_address);
		TM_NRF24L01_Transmit_Status_t transmissionStatus;
	    int start = HAL_GetTick();
	    TM_NRF24L01_PowerUpRx();
	    int prev_summ =0;
		int a0=0;
		int a1=0;
		int berr = 0;
		int cntCorr =0;
		uint16_t st=0;
		uint16_t rt=0;
		uint8_t *addr = 0;
	    while(1)
	    {

	    	//clearInt();
			//while(TM_NRF24L01_GetStatus()&14) //TM_NRF24L01_RxFifoEmpty()
	    	while(TM_NRF24L01_RxFifoEmpty())
			{
	    		delay_uS(2);
				//tm++;
				cr_flush_par();
			};
	    	uint8_t bytes = TM_NRF24L01_ReadRXBytes();
	    	while(bytes!=32)
	    	{
				a0 = bytes;
				berr++;
				delay_uS(5);
	    		TM_NRF24L01_PowerUpRx();
				delay_uS(5);
		    	while(TM_NRF24L01_RxFifoEmpty())
				{
		    		delay_uS(2);
					//tm++;
					cr_flush_par();
				};
		    	bytes = TM_NRF24L01_ReadRXBytes();
	    	}

			//PKT.counter = 0;
			TM_NRF24L01_GetData(&ATemp);
			if(ATemp.num>NUM_PAKS_IN_SUP-1)
			{
				//NUM_PAKS_IN_SUP = PKT.counter+1;
				//cntOk  = 0;
				//cntLost  = 0;
				//a0=0;
				//a1=0;
			}

			if(addr)
			{
				//AUDIO_OUT_Periodic(addr);
				addr = 0;
			}
			int  dfr = (ATemp.num+NUM_PAKS_IN_SUP-1)%NUM_PAKS_IN_SUP;
#if 1
			if(ATemp.num < old_counter)
			{
				// TBD flush_prev;
				//addr = &rcv_buff[RCV_SIZE*((rcv_buff_write)%NUM_RCV_BFRS)];
				AUDIO_OUT_Periodic(&rcv_buff[RCV_SIZE*((rcv_buff_write)%NUM_RCV_BFRS)]);
				int cnt_lerr = 0;
				int lost_packet_num = -1;
				for(int k=0;k<NUM_PAKS_IN_SUP;k++)
				{
					if(APacketQ[k].num!=k)
					{
						lost_packet_num = k;
						cnt_lerr++;
					}
				}
				if(cnt_lerr==1)
				{
					// here we can correct error!!!
					if(lost_packet_num==NUM_PAKS_IN_SUP-1)
					{
						//do nothing , only xor_packet was broken;
					}
					else
					{
						// data packet missed
						// init with xor_packet
				    	st = TIM3->CNT;
						struct sAuPacket* lostPacket = &APacketQ[lost_packet_num];

						*lostPacket = APacketQ[NUM_PAKS_IN_SUP-1];
						for(int k=0;k<NUM_PAKS_IN_SUP-1;k++)
						{
							if(k!=lost_packet_num)
							{
								struct sAuPacket* tmpPacket = &APacketQ[k];
								for(int t=0;t<PACKET_SIZE;t++)
								{
									lostPacket->data[t]^=tmpPacket->data[t];
								}
							}
						}
						st = TIM3->CNT-st;
					}
					cntCorr++;
				}
				else if(cnt_lerr>1)
				{
					cntLost++;
				}
				else
				{
					cntOk += NUM_PAKS_IN_SUP;
				}
				if(cnt_lerr<2)
				{
//#define RCV_SIZE ( PACKET_SIZE*(NUM_PAKS_IN_SUP-1))
//uint8_t rcv_buff[RCV_SIZE*NUM_RCV_BFRS];
//volatile int rcv_buff_write = 0;
					// move packets to buffer
					rt = TIM3->CNT;
					for(int k=0;k<NUM_PAKS_IN_SUP-1;k++)
					{
						struct sAuPacket* tmpPacket = &APacketQ[k];
						uint8_t* pnt = &rcv_buff[RCV_SIZE*rcv_buff_write+PACKET_SIZE*k];
						for(int t=0;t<PACKET_SIZE;t++)
						{
								pnt[t] = tmpPacket->data[t];
						}
					}

					//AUDIO_OUT_Periodic(&rcv_buff[RCV_SIZE*rcv_buff_write]);
					rcv_buff_write = (rcv_buff_write+1)%NUM_RCV_BFRS;
					rt = TIM3->CNT-rt;
				}
				int summ = cntOk+cntLost;
				int curr = HAL_GetTick();
				if((curr-start)>=2000)
				{
					//printf("Ok %d,Lost %d, berr%d  %d mS, %d KB/s %d err %d %d %d %d %d %d appDistErr %d\n",cntOk,cntLost,berr,curr-start,(int)((summ-prev_summ)*1000*31/(curr-start)),bytes,cnt_lerr,rt,st,cntCorr,(int)inputSpeed,(int)outSpeed,appDistErr);//cr_flush();
					float sPos_mean,sPos_dev;
					float sDif_mean,sDif_dev;
					calcV(&sPos,&sPos_mean,&sPos_dev);
					calcV(&sDif,&sDif_mean,&sDif_dev);
					printf("%d mS  lost = %d fc %d corr=%d posM %d posS %d  difM %d difS %d %d %d %d\n",(int)((summ-prev_summ)*1000*31/(curr-start)),cntLost,forceCounter,cntCorr,(int)sPos_mean,(int)sPos_dev,(int)sDif_mean,(int)sDif_dev,(int)inputSpeed,(int)outSpeed,appDistErr);
					clearV(&sPos);
					clearV(&sDif);
					//printf("%d KB/s %d %d appDistErr %d\n",(int)((summ-prev_summ)*1000*31/(curr-start)),(int)inputSpeed,(int)outSpeed,appDistErr);//cr_flush();
					start = curr;
					prev_summ = summ;
				}
				// init new
				// clear entries
				for(int k=0;k<NUM_PAKS_IN_SUP;k++)
				{
					APacketQ[k].num = NUM_PAKS_IN_SUP;
				}

				APacketQ[ATemp.num] = ATemp;

			}
			else
			{
				if(ATemp.num<NUM_PAKS_IN_SUP)
				{
					APacketQ[ATemp.num] = ATemp;
				}
				else
				{
					// ?? unknown
				}
				delay_uS(10);
				//delay_uS(40);
			}
#endif

#if 0
			if(old_counter==dfr)
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				cntOk++;
			}
			else
			{
				//a0 = ATemp.num;
				a1 = old_counter;
				cntLost+= (dfr+NUM_PAKS_IN_SUP-old_counter)%NUM_PAKS_IN_SUP;
			}
			int summ = cntOk+cntLost;
			int curr = HAL_GetTick();
			if((curr-start)>=2000)
			{
				printf("Ok %d,Lost %d, berr%d  %d mS, %d KB/s %d %d %d %d\n",cntOk,cntLost,berr,curr-start,(int)((summ-prev_summ)*1000*31/(curr-start)),a0,a1,NUM_PAKS_IN_SUP,bytes);//cr_flush();
				start = curr;
				prev_summ = summ;
			}
#endif
			old_counter = ATemp.num;//%NUM_PAKS_IN_SUP;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
  cr_flush();
  printf("\nStart program sys clk=%d MHz\n",HAL_RCC_GetSysClockFreq()/1000000);
  cr_flush();
  AUDIO_OUT_Start(rcv_buff, RCV_SIZE*NUM_RCV_BFRS,16);
  reinit_vars(I2S);

  cr_flush();
  example();
  cr_flush();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRF_CE_Pin|NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
