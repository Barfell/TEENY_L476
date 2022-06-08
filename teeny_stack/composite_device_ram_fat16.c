/*       
 *         _______                    _    _  _____ ____  
 *        |__   __|                  | |  | |/ ____|  _ \ 
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ < 
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/ 
 *                               __/ |                    
 *                              |___/                     
 *
 * TeenyUSB - light weight usb stack for micro controllers
 * 
 * Copyright (c) 2020 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "teeny_usb.h"
#include "teeny_usb_desc.h"
#include "teeny_usb_util.h"
#include "string.h"
#include "tusb_platform_stm32.h"
#include "board_config.h"

#include "tusbd_user.h"
#include "tusbd_hid.h"
#include "tusbd_cdc.h"
#include "tusbd_msc.h"

//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"

//#include "bsp_driver_sd.h"
//#include "cmsis_os.h"
//#include "usr_bsp.h"
//#include "fatfs.h"

#define USER_RX_EP_SIZE   32
#define CDC_RX_EP_SIZE    32
#define HID_RX_EP_SIZE    16
extern const uint8_t COMP_ReportDescriptor_if0[];
#define HID_REPORT_DESC         COMP_ReportDescriptor_if0
#define HID_REPORT_DESC_SIZE    24

// allocate more buffer for better performance
__ALIGN_BEGIN uint8_t user_buf[USER_RX_EP_SIZE*4] __ALIGN_END;

/*extern SD_HandleTypeDef hsd1; 
extern DMA_HandleTypeDef hdma_sdmmc1_tx;
extern DMA_HandleTypeDef hdma_sdmmc1_rx;
static HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd);
static HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd);*/

int user_recv_data(tusb_user_device_t* raw, const void* data, uint16_t len);
int user_send_done(tusb_user_device_t* raw, const void* data, uint16_t len);

void StartUSBTask(void const * argument);

tusb_user_device_t user_dev = {
  .backend = &user_device_backend,
  .ep_in = 3,
  .ep_out = 3,
  .on_recv_data = user_recv_data,
  .on_send_done = user_send_done,
  .rx_buf = user_buf,
  .rx_size = sizeof(user_buf),
};

// The HID recv buffer size must equal to the out report size
__ALIGN_BEGIN uint8_t hid_buf[HID_RX_EP_SIZE] __ALIGN_END;
int hid_recv_data(tusb_hid_device_t* hid, const void* data, uint16_t len);
int hid_send_done(tusb_hid_device_t* hid, const void* data, uint16_t len);

tusb_hid_device_t hid_dev = {
  .backend = &hid_device_backend,
  .ep_in = 2,
  .ep_out = 2,
  .on_recv_data = hid_recv_data,
  .on_send_done = hid_send_done,
  .rx_buf = hid_buf,
  .rx_size = sizeof(hid_buf),
  .report_desc = HID_REPORT_DESC,
  .report_desc_size = HID_REPORT_DESC_SIZE,
};

// The CDC recv buffer size should equal to the out endpoint size
// or we will need a timeout to flush the recv buffer
__ALIGN_BEGIN uint8_t cdc_buf[CDC_RX_EP_SIZE] __ALIGN_END;

int cdc_recv_data(tusb_cdc_device_t* cdc, const void* data, uint16_t len);
int cdc_send_done(tusb_cdc_device_t* cdc, const void* data, uint16_t len);
void cdc_line_coding_change(tusb_cdc_device_t* cdc);

tusb_cdc_device_t cdc_dev = {
  .backend = &cdc_device_backend,
  .ep_in = 1,
  .ep_out = 1,
  .ep_int = 8,
  .on_recv_data = cdc_recv_data,
  .on_send_done = cdc_send_done,
  .on_line_coding_change = cdc_line_coding_change,
  .rx_buf = cdc_buf,
  .rx_size = sizeof(cdc_buf),
};


int msc_get_cap(tusb_msc_device_t* msc, uint8_t lun, uint32_t *block_num, uint32_t *block_size);
int msc_block_read(tusb_msc_device_t* msc, uint8_t lun, uint8_t *buf, uint32_t block_addr, uint16_t block_len);
int msc_block_write(tusb_msc_device_t* msc, uint8_t lun, const uint8_t *buf, uint32_t block_addr, uint16_t block_len);

tusb_msc_device_t msc_dev = {
  .backend = &msc_device_backend,
  .ep_in = 4,
  .ep_out = 4,
  .max_lun = 0, // 1 logic unit
  .get_cap = msc_get_cap,
  .block_read = msc_block_read,
  .block_write = msc_block_write,
};

// make sure the interface order is same in "composite_desc.lua"
static tusb_device_interface_t* device_interfaces[] = {
  (tusb_device_interface_t*)&hid_dev,
  (tusb_device_interface_t*)&cdc_dev, 0,   // CDC need two interfaces
  (tusb_device_interface_t*)&user_dev,
  (tusb_device_interface_t*)&msc_dev,
};

tusb_device_config_t device_config = {
  .if_count = sizeof(device_interfaces)/sizeof(device_interfaces[0]),
  .interfaces = &device_interfaces[0],
};

void tusb_delay_ms(uint32_t ms)
{
  uint32_t i,j;
  for(i=0;i<ms;++i)
    for(j=0;j<200;++j);
}

static int user_len = 0;
int user_recv_data(tusb_user_device_t* raw, const void* data, uint16_t len)
{
  user_len = (int)len;
  return 1; // return 1 means the recv buffer is busy
}

int user_send_done(tusb_user_device_t* raw, const void* data, uint16_t len)
{
  tusb_set_rx_valid(raw->dev, raw->ep_out);
  return 0;
}

static int hid_len = 0;
int hid_recv_data(tusb_hid_device_t* hid, const void* data, uint16_t len)
{
  hid_len = (int)len;
  return 1; // return 1 means the recv buffer is busy
}

int hid_send_done(tusb_hid_device_t* hid, const void* data, uint16_t len)
{
  tusb_set_rx_valid(hid->dev, hid->ep_out);
  return 0;
}

static int cdc_len = 0;
int cdc_recv_data(tusb_cdc_device_t* cdc, const void* data, uint16_t len)
{
  cdc_len = (int)len;
  return 1; // return 1 means the recv buffer is busy
}

int cdc_send_done(tusb_cdc_device_t* cdc, const void* data, uint16_t len)
{
  tusb_set_rx_valid(cdc->dev, cdc->ep_out);
  return 0;
}

void cdc_line_coding_change(tusb_cdc_device_t* cdc)
{
    //cdc->line_coding.bitrate
    //cdc->line_coding.parity
    //cdc->line_coding.databits
    //cdc->line_coding.stopbits
    TUSB_LOGD("App Todo: set real line coding\n");
}

static tusb_device_t g_dev;
void stdio_init(void);

#ifdef STM32L476xx
extern void SystemClock_Config(void);

extern void PeriphCommonClock_Config(void);
#endif
//int main(void)
////void init_usb(void)
////void StartUSBTask(void const * argument)
//{
//#ifdef STM32L476xx
//  /* Configure the system clock */
//  SystemClock_Config();

///* Configure the peripherals common clocks */
//  PeriphCommonClock_Config();
//#endif
//  stdio_init();
//  TUSB_LOGD("Composite device begin\n");
//  SetDescriptor(&g_dev, &COMP_descriptors);
//  tusb_set_device_config(&g_dev, &device_config);
//  tusb_open_device(&g_dev, TUSB_DEFAULT_DEV_PARAM);
//  while(!g_dev.config){
//      // wait device ready
//  }
//  // Device configured, ready to work
//  //while(1){
//	for(;;){
////		osDelay(100);
//    if(user_len){
//      for(int i=0;i<user_len;i++){
//        user_buf[i]+=1;
//      }
//      tusb_user_device_send(&user_dev, user_buf, user_len);
//      user_len = 0;
//    }

//    if(hid_len){
//      for(int i=0;i<hid_len;i++){
//        hid_buf[i]+=2;
//      }
//      tusb_hid_device_send(&hid_dev, hid_buf, hid_len);
//      hid_len = 0;
//    }

//    if(cdc_len){
//      for(int i=0;i<cdc_len;i++){
//        cdc_buf[i]+=3;
//      }
//      tusb_cdc_device_send(&cdc_dev, cdc_buf, cdc_len);
//      cdc_len = 0;
//    }

//    tusb_msc_device_loop(&msc_dev);
//  }
//}

void init_usb(void)
//void StartUSBTask(void const * argument)
{
//  stdio_init();
  TUSB_LOGD("Composite device begin\n");
  SetDescriptor(&g_dev, &COMP_descriptors);
  tusb_set_device_config(&g_dev, &device_config);
  tusb_open_device(&g_dev, TUSB_DEFAULT_DEV_PARAM);
  while(!g_dev.config){
      // wait device ready
  }
  // Device configured, ready to work
  //while(1){
	for(;;){
		//osDelay(100);
    if(user_len){
      for(int i=0;i<user_len;i++){
        user_buf[i]+=1;
      }
      tusb_user_device_send(&user_dev, user_buf, user_len);
      user_len = 0;
    }

    if(hid_len){
      for(int i=0;i<hid_len;i++){
        hid_buf[i]+=2;
      }
      tusb_hid_device_send(&hid_dev, hid_buf, hid_len);
      hid_len = 0;
    }

    if(cdc_len){
      for(int i=0;i<cdc_len;i++){
        cdc_buf[i]+=3;
      }
      tusb_cdc_device_send(&cdc_dev, cdc_buf, cdc_len);
      cdc_len = 0;
    }

    tusb_msc_device_loop(&msc_dev);
  }
}

#if  defined(STM32L476xx)
//#define BLOCK_SIZE   512
// the stack is start at RAM end in GCC linker script, reserve the last 2 blocks
//uint8_t g_ramBuf[20*1024] = {0x00};

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x81000
#define STORAGE_BLK_SIZ                  0x200

#include <stdint.h>
typedef uint8_t Byte;
typedef struct MasterBootRecord
{
    Byte    checkRoutionOnx86[446];
    struct
    {
        Byte    bootDescriptor;             /* 0x80: bootable device, 0x00: non-bootable */
        Byte    firstPartitionSector[3];    /* 1st sector number */
        Byte    fileSystemDescriptor;       /* 1:FAT12, 4:FAT16(less than 32MB), 5:Extended-DOS Partition,
                                                                                        6:FAT16(more 32MB), 0xb:FAT32(more 2GB),
                                                                                        0xc:FAT32 Int32h, 0xe:FAT16 Int32h,
                                                                                        0xf:5:Extended-DOS Partition Int32h */
        Byte    lastPartitionSector[3];
        Byte    firstSectorNumbers[4];      /* first sector number (link to BPB sector) */
        Byte    numberOfSectors[4];
    }   partitionTable[4];
    Byte    sig[2];                         /* 0x55, 0xaa */
}   MBRecord;

typedef struct FAT16BPB_t
{
    /* FAT16 or FAT12 BPB */
    Byte    jmpOpeCode[3];          /* 0xeb ?? 0x90 */
    Byte    OEMName[8];
    /* FAT16 */
    Byte    bytesPerSector[2];      /* bytes/sector */
    Byte    sectorsPerCluster;      /* sectors/cluster */
    Byte    reservedSectors[2];     /* reserved sector, beginning with sector 0 */
    Byte    numberOfFATs;           /* file allocation table */
    Byte    rootEntries[2];         /* root entry (512) */
    Byte    totalSectors[2];        /* partion total secter */
    Byte    mediaDescriptor;        /* 0xf8: Hard Disk */
    Byte    sectorsPerFAT[2];       /* sector/FAT (FAT32 always zero: see bigSectorsPerFAT) */
    Byte    sectorsPerTrack[2];     /* sector/track (not use) */
    Byte    heads[2];               /* heads number (not use) */
    Byte    hiddenSectors[4];       /* hidden sector number */
    Byte    bigTotalSectors[4];     /* total sector number */
    /* info */
    Byte    driveNumber;
    Byte    unused;
    Byte    extBootSignature;
    Byte    serialNumber[4];
    Byte    volumeLabel[11];
    Byte    fileSystemType[8];      /* "FAT16   " */
    Byte    loadProgramCode[448];
    Byte    sig[2];                 /* 0x55, 0xaa */
}   BPBlock; // BIOS Parameter Block

typedef struct DirEntry_t
{
    Byte    name[8];            /* file name */
    Byte    extension[3];       /* file name extension */
    Byte    attribute;          /* file attribute
                                                                bit 4    directory flag
                                                                bit 3    volume flag
                                                                bit 2    hidden flag
                                                                bit 1    system flag
                                                                bit 0    read only flag */
    Byte    reserved;           /* use NT or same OS */
    Byte    createTimeMs;       /* VFAT 10millsec (0   199) */
    Byte    createTime[2];      /* VFAT */
    Byte    createDate[2];      /* VFAT */
    Byte    accessDate[2];      /* VFAT */
    Byte    clusterHighWord[2]; /* FAT32 MSB 16 bits */
    Byte    updateTime[2];
    Byte    updateDate[2];
    Byte    cluster[2];         /* start cluster number */
    Byte    fileSize[4];        /* file size in bytes (directory is always zero) */
}   DirEntry;

#pragma anon_unions  //keil��Ĭ���ǲ�֧�������ṹ��ģ���Ҫ����ָ��#pragma anon_unionsָ����
typedef struct _DirEntTime
{
    union
    {
        uint16_t W;
        struct
        {
            uint16_t second : 5;
            uint16_t minutes : 6;
            uint16_t hour : 5;
        } B;
    };
} DirEntTime;

typedef struct _DirEntDate
{
    union
    {
        uint16_t W;
        struct
        {
            uint16_t day : 5;
            uint16_t month : 4;
            uint16_t year : 7;
        } B;
    };
} DirEntDate;
#pragma no_anon_unions


const MBRecord sectMBR =
{
    .checkRoutionOnx86 = { 0x00 },
    .partitionTable = {
        {
            .bootDescriptor = 0x00,
            .firstPartitionSector = { 0x02, 0x21, 0x00 },
            .fileSystemDescriptor = 0x06, //FAT16
            .lastPartitionSector = { 0xC2, 0x22, 0x20 },
            .firstSectorNumbers = { 0x00, 0x08, 0x00, 0x00 },
            .numberOfSectors = { 0x00, 0x00, 0x08, 0x00 },
        },//[0]
        { 0 },//[1]
        { 0 },//[2]
        { 0 },//[3]
    },
    .sig = { 0x55, 0xAA },
 };
const BPBlock sectBPB =
{
    .jmpOpeCode = { 0xEB, 0x00, 0x90 },
    .OEMName = { ' ',' ',' ',' ',' ',' ',' ',' ' },
    .bytesPerSector = { 0x00, 0x02 },
    .sectorsPerCluster = 0x08, // 4KB/sectors
    .reservedSectors = { 0x08, 0x00 },
    .numberOfFATs = 0x02,
    .rootEntries = { 0x00, 0x02 },
    .totalSectors = { 0x00, 0x00 },
    .mediaDescriptor = 0xF8, // HDD
    .sectorsPerFAT = { 0x00, 0x01 },
    .sectorsPerTrack = { 0x3F,0x00 },
    .heads = { 0xFF,0x00 },
    .hiddenSectors = { 0x00, 0x08, 0x00, 0x00 },
    .bigTotalSectors = { 0x00,0x00,0x08, 0x00 },
    .driveNumber = 0x80,
    .unused = 0,
    .extBootSignature = 0x29,
    .serialNumber = { 0x78,0x56,0x34,0x12 },
    .volumeLabel = { 'N','O',' ','N','A','M','E',' ',' ',' ',' ' },
    .fileSystemType = { 'F','A','T','1','6',' ',' ',' ' },
    .loadProgramCode = { 0 },
    .sig = { 0x55, 0xAA },
};

#define SECTOR_MBR  (0x0000)
#define SECTOR_PBR  (0x0800)
#define SECTOR_FAT1 (0x0808)
#define SECTOR_FAT2 (0x0908)
#define SECTOR_ROOT (0x0A08)
#define SECTOR_DATA (0x0A28)

void _handleFatClusterChain(uint32_t sect_offset, uint8_t *buf)
{
    uint16_t *bufW = (uint16_t *)&buf[0];
    if (sect_offset == 0)
    {
        bufW[0] = 0xfff8;
        bufW[1] = 0xffff;
        bufW[2] = 0xffff; //������һ���ļ���1�ء�
    }
}

void _handleRoot(uint32_t sect_offset, uint8_t *buf)
{
    // 1 sector(512bytes) has 16 entries
    DirEntry *pDir = (DirEntry *)buf;
    if (sect_offset == 0)
    {
        memset(pDir, 0x00, sizeof(DirEntry));
        sprintf((char *)pDir->name, "TEXT_123");
        pDir->extension[0] = 'T';
        pDir->extension[1] = 'X';
        pDir->extension[2] = 'T';
        pDir->attribute = 0x00;
        {
            DirEntTime *pT = (DirEntTime *)&pDir->updateTime[0];
            DirEntDate *pD = (DirEntDate *)&pDir->updateDate[0];
            pT->B.hour = 12;
            pT->B.minutes = 34;
            pT->B.second = 56 / 2;
            pD->B.year = 2017 - 1980;
            pD->B.month = 1;
            pD->B.day = 12;
        }

        *(uint16_t*)&pDir->cluster = 0x0002;
        *(uint32_t*)&pDir->fileSize = 123;
    }
}

void _handleData(uint32_t sect_offset, uint8_t *buf)
{
    memset(buf, 'A', 512);
    sprintf((char *)buf, "Hello World!\r\n");
    buf[14]='>';
}

uint32_t _ReadSector(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    switch (blk_addr)
    {
    case SECTOR_MBR:
        memcpy(buf, (const void *)&sectMBR, 512);
        break;
    case SECTOR_PBR:
        memcpy(buf, (const void *)&sectBPB, 512);
        break;
    default:
        memset(buf, 0x00, 512);
        //FAT cluster chain
        if ((SECTOR_FAT1 <= blk_addr) && (blk_addr < SECTOR_ROOT))//�жϵ�ַ SECTOR_FAT1~SECTOR_ROOT
        {
            if (blk_addr >= SECTOR_FAT2)//  >SECTOR_FAT2
            {
                blk_addr -= (SECTOR_FAT2 - SECTOR_FAT1);//blk_addr=blk_addr-(SECTOR_FAT2 - SECTOR_FAT1)
            }
            _handleFatClusterChain(blk_addr - SECTOR_FAT1, buf);//blk_addr=blk_addr-SECTOR_FAT2

        }
        else if ((SECTOR_ROOT <= blk_addr) && (blk_addr < SECTOR_DATA))//SECTOR_ROOT~SECTOR_DATA
        {
            _handleRoot(blk_addr - SECTOR_ROOT, buf);

        }
        else if (SECTOR_DATA <= blk_addr)//>SECTOR_DATA
        {
            _handleData(blk_addr - SECTOR_DATA, buf);

        }
        break;
    }
    return 0;
}

#if defined(STM32F723xx)
#define BLOCK_COUNT  ((256-64-2)*2)
#elif defined(STM32F767xx)
#define BLOCK_COUNT  ((512-64-2)*2)
#elif defined(STM32H743xx)
#define START_ADDR   (uint8_t*)(0x24000000)
#define BLOCK_COUNT  ((512-64-2)*2)
#else
//#define BLOCK_COUNT   0x10000//((128-64-2)*2)
#endif

/*#ifndef START_ADDR
#define START_ADDR   (uint8_t*)(0x20000000ul + 64*1024ul)
#endif*/

int msc_get_cap(tusb_msc_device_t* msc, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return 0;
	
/*	BSP_SD_CardInfo CardInfo;
//	int8_t ret = USBD_FAIL;  
	 
  if(BSP_SD_IsDetected() != SD_NOT_PRESENT)
	{
			 
			BSP_SD_GetCardInfo(&CardInfo);
			*block_num = (CardInfo.LogBlockNbr) - 1;
			*block_size = CardInfo.LogBlockSize;
//			ret = USBD_OK;//printf("USB SD GET CARDINFO OK\r\n");
	}
  return 0;	*/	
}

int msc_block_read(tusb_msc_device_t* msc, uint8_t lun, uint8_t *buf, uint32_t block_addr, uint16_t block_len)
{
  /*uint32_t len = block_len*BLOCK_SIZE;
  uint32_t offset = block_addr*BLOCK_SIZE;
  memcpy(buf, START_ADDR + offset, len);
  return (int)len;*/

	_ReadSector(buf, block_addr, block_len);
  return block_len*STORAGE_BLK_SIZ;
	
//	  uint32_t timeout = 100000;
//    uint32_t ret = 0; 
//		/* Invalidate the dma rx handle*/
//		hsd1.hdmarx = NULL;
////		xSemaphoreTake( xMutex_SD_OPRATE, portMAX_DELAY );
//    if(SD_DMAConfigRx(&hsd1) != HAL_OK)
//		{
////			xSemaphoreGive( xMutex_SD_OPRATE );
//			return 0;			
//		}
//	
//	
//  if(BSP_SD_IsDetected() != SD_NOT_PRESENT)
//  {  
//    /* Read block(s) in DMA transfer mode */
//				ret = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)buf, block_addr, block_len);
//        if(ret == 0)
//        {		
//					/* Wait until transfer is complete */
//					while(BSP_SD_GetCardState() != SD_TRANSFER_OK)
//					{
//						if (timeout-- == 0)
//						{
////							xSemaphoreGive( xMutex_SD_OPRATE );
//							return 0;
//						}
//					}
//					ret = block_len*BLOCK_SIZE;
//				}
//  }
//	
//  return ret;
}

int msc_block_write(tusb_msc_device_t* msc, uint8_t lun, const uint8_t *buf, uint32_t block_addr, uint16_t block_len)
{
  /*uint32_t len = block_len*BLOCK_SIZE;
  uint32_t offset = block_addr*BLOCK_SIZE;
  memcpy(START_ADDR + offset, buf, len);
  return (int)len;*/

	/*if((block_addr+1)*BLOCK_SIZE <= sizeof(g_ramBuf)){
		for(int blk_index=0; blk_index<block_len; ++blk_index){
			for(int index=0; index<BLOCK_SIZE; ++index){
				g_ramBuf[block_addr*BLOCK_SIZE+blk_index*BLOCK_SIZE+index] = 
				buf[blk_index*BLOCK_SIZE+index];
			}
		}
	}*/
  return block_len*STORAGE_BLK_SIZ;
	
//  uint32_t ret = 0;//USBD_FAIL; 
//  uint32_t timeout = 100000; 

//  /* Invalidate the dma tx handle*/
//  hsd1.hdmatx = NULL;
////	xSemaphoreTake( xMutex_SD_OPRATE, portMAX_DELAY );
//  if(SD_DMAConfigTx(&hsd1) != HAL_OK)
//	{
////		xSemaphoreGive( xMutex_SD_OPRATE );
//		return 0;
//	}
//	
//	
//  if(BSP_SD_IsDetected() != SD_NOT_PRESENT)
//  { 
//    /* Write block(s) in DMA transfer mode */
//				ret = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)buf, block_addr, block_len);
//				
//        /* Wait until transfer is complete */
//        if(ret == 0)
//        {
//					while(BSP_SD_GetCardState() != SD_TRANSFER_OK)
//					{
//						if (timeout-- == 0)
//						{
//							return 0;
//						}
//					}
//					ret = block_len*BLOCK_SIZE;;
//        }
//  }

//  return ret;	
}

///**
//  * @brief Configure the DMA to receive data from the SD card
//  * @retval
//  *  HAL_ERROR or HAL_OK
//  */
//static HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd)
//{
//  static DMA_HandleTypeDef hdma_rx;
//  HAL_StatusTypeDef status = HAL_ERROR;
//  
//  /* Configure DMA Rx parameters */
//  hdma_rx.Init.Request             = DMA_REQUEST_7;
//  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
//  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
//  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
//  hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;

//  hdma_rx.Instance = DMA2_Channel4;
//  hdma_rx.Instance->CNDTR = 0;//
//  hdma_rx.Instance->CCR = 0;//
//  /* Associate the DMA handle */
//  __HAL_LINKDMA(hsd, hdmarx, hdma_rx);

//  /* Stop any ongoing transfer and reset the state*/
//  HAL_DMA_Abort(&hdma_rx);
//  
//  /* Deinitialize the Channel for new transfer */
//  HAL_DMA_DeInit(&hdma_rx);

//  /* Configure the DMA Channel */
//  status = HAL_DMA_Init(&hdma_rx);
//  
//  /* NVIC configuration for DMA transfer complete interrupt */
//  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
//  
//  return (status);
//}

///**
//  * @brief Configure the DMA to transmit data to the SD card
//  * @retval
//  *  HAL_ERROR or HAL_OK
//  */
//static HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd)
//{
//  static DMA_HandleTypeDef hdma_tx;
//  HAL_StatusTypeDef status;
//  
//  /* Configure DMA Tx parameters */
//  hdma_tx.Init.Request             = DMA_REQUEST_7;
//  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
//  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
//  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
//  hdma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
//  
//  hdma_tx.Instance = DMA2_Channel4;
//  hdma_tx.Instance->CNDTR = 0;//
//  hdma_tx.Instance->CCR = 0;//  
//  /* Associate the DMA handle */
//  __HAL_LINKDMA(hsd, hdmatx, hdma_tx);
//  
//  /* Stop any ongoing transfer and reset the state*/
//  HAL_DMA_Abort(&hdma_tx);
//  
//  /* Deinitialize the Channel for new transfer */
//  HAL_DMA_DeInit(&hdma_tx);
//  
//  /* Configure the DMA Channel */
//  status = HAL_DMA_Init(&hdma_tx); 
//  
//  /* NVIC configuration for DMA transfer complete interrupt */
//  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

//  return (status);
//}

#elif  defined(STM32F723xx) || defined(STM32F767xx) || defined(STM32F407xx) || defined(STM32H743xx)
#define BLOCK_SIZE   512
// the stack is start at RAM end in GCC linker script, reserve the last 2 blocks
#if defined(STM32F723xx)
#define BLOCK_COUNT  ((256-64-2)*2)
#elif defined(STM32F767xx)
#define BLOCK_COUNT  ((512-64-2)*2)
#elif defined(STM32H743xx)
#define START_ADDR   (uint8_t*)(0x24000000)
#define BLOCK_COUNT  ((512-64-2)*2)
#else
#define BLOCK_COUNT  ((128-64-2)*2)
#endif

#ifndef START_ADDR
#define START_ADDR   (uint8_t*)(0x20000000ul + 64*1024ul)
#endif

int msc_get_cap(tusb_msc_device_t* msc, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
  *block_num = BLOCK_COUNT;
  *block_size = BLOCK_SIZE;
  return 0;
}

int msc_block_read(tusb_msc_device_t* msc, uint8_t lun, uint8_t *buf, uint32_t block_addr, uint16_t block_len)
{
  uint32_t len = block_len*BLOCK_SIZE;
  uint32_t offset = block_addr*BLOCK_SIZE;
  memcpy(buf, START_ADDR + offset, len);
  return (int)len;
}

int msc_block_write(tusb_msc_device_t* msc, uint8_t lun, const uint8_t *buf, uint32_t block_addr, uint16_t block_len)
{
  uint32_t len = block_len*BLOCK_SIZE;
  uint32_t offset = block_addr*BLOCK_SIZE;
  memcpy(START_ADDR + offset, buf, len);
  return (int)len;
}
#else

#if defined(FLASH_SIZE)
#define START_ADDR   (const uint8_t*)(0x08000000ul + 20*1024ul)
#define BLOCK_SIZE FLASH_PAGE_SIZE
#define BLOCK_COUNT  ((FLASH_SIZE - 20*1024ul) / FLASH_PAGE_SIZE)
int msc_get_cap(tusb_msc_device_t* msc, uint8_t lun, uint32_t *block_num, uint32_t *block_size) {
  *block_size = BLOCK_SIZE;
  *block_num =  BLOCK_COUNT;
  return 0;
}

int msc_block_read(tusb_msc_device_t* msc, uint8_t lun, uint8_t *buf, uint32_t block_addr, uint16_t block_len)
{
  uint32_t len = block_len*BLOCK_SIZE;
  memcpy(buf, START_ADDR+block_addr*BLOCK_SIZE, len);
  return len;
}

int msc_block_write(tusb_msc_device_t* msc, uint8_t lun, const uint8_t *buf, uint32_t block_addr, uint16_t block_len)
{
  uint32_t len = block_len*BLOCK_SIZE;
  flash_write( (uint32_t)START_ADDR+block_addr*BLOCK_SIZE, buf, len);
  return len;
}

#else
#warning MSC storage not set
#endif

#endif

#if MSC_DATA_PACKET_LENGTH < BLOCK_SIZE
#error MSC data packet buffer is small than the block size
#endif
