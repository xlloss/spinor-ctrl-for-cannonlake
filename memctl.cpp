#include "memctl.h"
#include <stdio.h>
#include <Windows.h>

static HANDLE hDevice = INVALID_HANDLE_VALUE;

int ReadMemDW(uint32_t PhysicalAddr, uint32_t* Data)
{
    MEM_COMMAND Command;
    ULONG bytesReturned;
    BOOL bRc;

    Command.CommandCode = CMD_MEM_READ_DWORD;
    Command.PhysicalAddr = PhysicalAddr;

    bRc = DeviceIoControl(hDevice,
        (DWORD)MEMCTL_COMMAND_BUFFERED,
        &Command,
        sizeof(Command),
        &Command,
        sizeof(Command),
        &bytesReturned,
        NULL
    );

    if (!bRc)
    {
        //ShowMessage ( "Error in DeviceIoControl : " + GetLastError());
        return MEMORY_ACCESS_ERROR;
    }

    *Data = Command.OutputValue;
    return 0;
}

int WriteMemDW(uint32_t PhysicalAddr, uint32_t Data)
{
    MEM_COMMAND Command;
    ULONG bytesReturned;
    BOOL bRc;

    Command.CommandCode = CMD_MEM_WRITE_DWORD;
    Command.PhysicalAddr = PhysicalAddr;
    Command.InputValue = Data;

    bRc = DeviceIoControl(hDevice,
        (DWORD)MEMCTL_COMMAND_BUFFERED,
        &Command,
        sizeof(Command),
        &Command,
        sizeof(Command),
        &bytesReturned,
        NULL
    );

    if (!bRc)
    {
        //ShowMessage ( "Error in DeviceIoControl : " + GetLastError());
        return MEMORY_ACCESS_ERROR;

    }

    return 0;
}

int Init_memctl(void)
{
    hDevice = CreateFile("\\\\.\\MemCtl",
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    if (hDevice == INVALID_HANDLE_VALUE) {
        printf("NO MemCtl Driver\n");
        return MEM_DRIVER_ERROR;
    }

    return MEM_SUCCESSFUL;
}

void Release_memctl(void)
{
    CloseHandle(hDevice);
}
