#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "memctl.h"
#include "spi-generic.h"
#include "gspi.h"

#define CMD_JEDEC_ID 0x9F
#define CMD_UNIQ_ID 0x4B
#define CMD_WRITE_ENABLE 0x06
#define CMD_WRITE_DISABLE 0x04
#define CMD_DUMMY 0

#define CS_HI() gspi_cs_deassert(spidev)
#define CS_LO() gspi_cs_assert(spidev)

int lock;

void w25q_lock(uint8_t enable)
{
    if (enable)
        lock = 1;
    else
        lock = 0;
}

uint8_t w25q_get_lock()
{
    return lock;
}

int w25q_send_cmd(unsigned char cmd)
{
    int ret;
    unsigned char data;

    ret = gspi_ctrlr_xfer(&cmd, 1, &data, 0);
    if (ret) {
        printf("send CMD fail\r\n");
        return -1;
    }
    return ret;
}

int w25q_get_data(unsigned char *data, unsigned char count)
{
    int ret;
    unsigned char cmd;

    ret = gspi_ctrlr_xfer(&cmd, 0, data, count);
    if (ret) {
        printf("%s fail\r\n", __func__);
        return -1;
    }

    return ret;
}

void w25qxx_read_uniqid(const struct spi_slave *spidev)
{
    uint8_t	i = 0;
    uint8_t data[8];

    printf("READ UNIQID ID CMD 0x4B\r\n");
    gspi_cs_deassert(spidev);

    w25q_send_cmd(CMD_UNIQ_ID);

	for(i = 0; i < 4; i++)
		w25q_send_cmd(CMD_DUMMY);

	for(i = 0; i < 8; i++)
		w25q_get_data(data, 8);

    gspi_cs_assert(spidev);

    for(i = 0; i < 8; i++)
        printf("data[%d] 0x%x\r\n", i, data[i]);

}

uint32_t w25qxx_read_id(const struct spi_slave *spidev)
{
    unsigned char data[3];
    int ret;

    printf("READ ID CMD 0x9F\r\n");
    gspi_cs_deassert(spidev);

    ret = w25q_send_cmd(CMD_JEDEC_ID);
    ret = w25q_get_data(data, 3);

    gspi_cs_assert(spidev);

    printf("ID 0x%x\r\n", data[0] << 16 | data[1] << 8 | data[2]);
    return ret;
}
#define WRITE_ENABLE 1
#define WRITE_DISABLE 0

void w25qxx_write_en(const struct spi_slave *spidev, uint8_t enable)
{
    CS_HI();

    if (enable)
        w25q_send_cmd(CMD_WRITE_ENABLE);
    else
        w25q_send_cmd(CMD_WRITE_DISABLE);

    CS_LO();
    Sleep(1);
}

void w25qxx_wait_for_write_end(void)
{
    uint8_t status_1;

    Sleep(1);
    CS_HI();
    w25q_send_cmd(0x05);

    do {
        status_1 = w25q_get_data(data, 1);
		Sleep(1);
    } while ((status_1 & 0x01) == 0x01);
    CS_LO();
}

#define LOCK_ENABLE 1
#define LOCK_DISABLE 0
void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while (w25q_get_lock() == 1)
		Sleep(1);

    w25q_lock(LOCK_ENABLE);	

	#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();	
	printf("w25qxx EraseSector %d Begin...\r\n", SectorAddr);
	#endif

	w25qxx_wait_for_write_end();
	//SectorAddr = SectorAddr * w25qxx.SectorSize;
    w25qxx_write_en(spidev, W);
    
    CS_HI();
    w25q_send_cmd(0x20);
	if(w25qxx.ID >= W25Q256)
		W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);

    //W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
    //W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
    //W25qxx_Spi(SectorAddr & 0xFF);
    CS_LO();

    W25qxx_WaitForWriteEnd();

	#if (_W25QXX_DEBUG == 1)
    printf("w25qxx EraseSector done after %d ms\r\n",HAL_GetTick()-StartTime);
	#endif

	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}

int main(void)
{
    int ret;
    spi_slave spidev;
 
    printf("start\r\n");
    spidev.cs = 0;
    
    ret = gspi_ctrlr_setup(&spidev);
    if (ret)
        printf("gspi_ctrlr_setup fail\r\n");

    w25qxx_read_id(&spidev);
    w25qxx_read_uniqid(&spidev);

    getchar();
    printf("end\r\n");
    return 0;
}
