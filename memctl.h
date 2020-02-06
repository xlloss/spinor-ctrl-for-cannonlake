#ifndef _MEMCTL_H_
#define _MEMCTL_H_

#include <stdint.h>

#ifdef _WINDOWS
int Init_memctl(void);
void Release_memctl(void);
#else
#include <stddef.h>
void* mem_mmap(size_t base, size_t len, const char* devmem, int* devmem_point);
int mem_munmap(void* addr, size_t base, size_t length, int* devmem_point);
#endif
int ReadMemDW(uint32_t PhysicalAddr, uint32_t* pData);
int WriteMemDW(uint32_t PhysicalAddr, uint32_t Data);

#ifdef _WINDOWS
#define MEMCTL_TYPE 40000
#define MEMCTL_COMMAND_BUFFERED \
    CTL_CODE( MEMCTL_TYPE, 0x900, METHOD_BUFFERED, FILE_ANY_ACCESS  )

#define CMD_MEM_READ_BYTE           0x11
#define CMD_MEM_WRITE_BYTE          0x12
#define CMD_MEM_READ_WORD           0x13
#define CMD_MEM_WRITE_WORD          0x14
#define CMD_MEM_READ_DWORD          0x15
#define CMD_MEM_WRITE_DWORD         0x16
#define CMD_MEM_READ_BLOCK          0x17

#pragma pack(1)
typedef struct _MEM_COMMAND {
	unsigned long PhysicalAddr;
	unsigned long OutputValue;
	unsigned long InputValue;
	unsigned long OutputBlock[16];
	unsigned long CommandCode;
} MEM_COMMAND;
#pragma pack()
#endif

#define MEM_SUCCESSFUL             0
#define MEM_FAILED                 -1
#define MEMORY_ACCESS_ERROR    0xFFFFFFFF
#define MEM_DRIVER_ERROR       0xFFFFF8FF



#endif  /* _MEMCTL_H_ */

