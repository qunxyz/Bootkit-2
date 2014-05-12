
#include <windows.h>

#include "..\Public\Public.h"
#include <strsafe.h>
#include "Install.h"

int SubMain()
{
	HANDLE hDevice;
	MBR_SECTOR MbrSector;
	DWORD dwRetReadByte;
	DWORD dwCount;

	dwCount = 0;
	dwRetReadByte = 0;
	RtlZeroMemory(&MbrSector,sizeof(MBR_SECTOR));
	hDevice = CreateFile(FIRST_DISK, \
		GENERIC_READ, \
		FILE_SHARE_READ | FILE_SHARE_WRITE, \
		NULL, \
		OPEN_EXISTING, \
		0,
		NULL);
	if (hDevice == INVALID_HANDLE_VALUE)
	{
		OutputDebugString(TEXT("Open device failed\r\n"));
		return FALSE;
	}
	if (DeviceIoControl(hDevice, \
		FSCTL_LOCK_VOLUME, \
		NULL, \
		0, \
		NULL, \
		0, \
		&dwCount, \
		NULL) == FALSE || \
		ReadFile(hDevice, \
		&MbrSector, \
		512, \
		&dwRetReadByte, \
		NULL) == FALSE || dwRetReadByte < 512)
	{
		DeviceIoControl(hDevice, \
			FSCTL_UNLOCK_VOLUME, \
			NULL, \
			0, \
			NULL, \
			0, \
			&dwCount, \
			NULL);
		CloseHandle(hDevice);
		return FALSE;
	}
	return 0;
}







































































