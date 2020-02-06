#include <stdio.h>
#include <stdlib.h>
#include "memctl.h"
#include "spi-generic.h"
#include "gspi.h"

#define CMD_JEDEC_ID 0x9F
#define CMD_DUMMY 0
uint32_t w25qxx_read_id(const struct spi_slave *spidev)
{
    unsigned char cmd, cmd_num;
    unsigned char data[3], data_num;
    int ret;

    printf("READ ID CMD 0x9F\r\n");
    gspi_cs_deassert(spidev);

    cmd = CMD_JEDEC_ID;
    cmd_num = 1;
    data_num = 0;
    ret = gspi_ctrlr_xfer(&cmd, cmd_num, &data, data_num);
    if (ret) {
        printf("send CMD fail\r\n");
        return -1;
    }

    cmd = CMD_DUMMY;
    cmd_num = CMD_DUMMY;
    data_num = 3;
    ret = gspi_ctrlr_xfer(&cmd, cmd_num, data, data_num);
    if (ret) {
        printf("send dummy 1 fail\r\n");
        return -1;
    }
    gspi_cs_assert(spidev);

    printf("ID 0x%x\r\n", data[0] << 16 | data[1] << 8 | data[2]);
    return 0;
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

    getchar();
    printf("end\r\n");
    return 0;
}
