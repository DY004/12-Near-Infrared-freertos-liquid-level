#include "flash_write_read.h"


uint16_t water_flag_fill_flag = 0;
 
//根据读取的地址，取出内容
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr; 
}
 
//返回当前要操作的扇区
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;   
	return FLASH_SECTOR_11;	
}
 
//写入flash
void STMFLASH_Write(uint32_t Addr,uint32_t *pBuffer,uint32_t Num)	
{ 
	FLASH_EraseInitTypeDef FlashEraseInit;
	HAL_StatusTypeDef FlashStatus=HAL_OK;
	uint32_t SectorError=0;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
	uint32_t WriteAddr=Addr;
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//错误地址
    
	HAL_FLASH_Unlock();         //解锁flash
	addrx=WriteAddr;			//写入起始地址
	endaddr=WriteAddr+Num*4;	//写入结束地址
    
	if(addrx<0X080C1000)
	{
		while(addrx<endaddr)		
		{
			 if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)		
			 {   
				FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //扇区擦除
				FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //擦除的扇区
				FlashEraseInit.NbSectors=1;                             //擦除1个扇区
				FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //电压范围2.7-3.6
				if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
				{
					break;//写入错误
				}
			 }
			 else addrx+=4;
			 FLASH_WaitForLastOperation(FLASH_WAITETIME);                //等待操作完成
		}
	}
	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);         //等待操作完成
	if(FlashStatus==HAL_OK)
	{
		 while(WriteAddr<endaddr)//写数据
		 {
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)
			{ 
				break;
			}
			WriteAddr+=4;
			pBuffer++;
		}  
	}
	HAL_FLASH_Lock();           //flash上锁
} 
 
void STMFLASH_Read(uint32_t Addr,uint32_t *pBuffer,uint32_t size)   	
{
	uint32_t i;
	uint32_t ReadAddr=Addr;
	for(i=0;i<size;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4字节
		ReadAddr+=4;//地址移动4字节
	}
}









