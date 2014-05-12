
#pragma pack(1) //字节对齐

typedef struct _PARTITION_ENTRY//分区表结构
{
    UCHAR active; 
	//状态（是否被激活）   重要
    UCHAR StartHead; 
	//分区起始磁头号   
    //UCHAR StartSector; 
    //分区起始扇区和柱面号,高2位为柱面号的第 9,10 位, 高字节为柱面号的低 8 位  
    //UCHAR StartCylinder;
    // 起始磁盘柱面 
    USHORT StartSecCyli; 
	//与63相位与得出的是开始扇区，把它右移6位就是开始柱面
    UCHAR PartitionType; 
	// 分区类型   重要 
    UCHAR EndHead; 
	//分区结束磁头号
    //UCHAR EndSector;
    //分区结束扇区   
    //UCHAR EndCylinder;
    // 结束柱面号
    USHORT EndSecCyli; 
	//与63相位与得出的就是结束扇区，把它右移6位就是结束柱面
    ULONG StartLBA; 
	// 扇区起始逻辑地址   重要
    ULONG TotalSector;
	// 分区大小      重要
} PARTITION_ENTRY, *PPARTITION_ENTRY;

//引导区512BYTE结构

typedef struct _MBR_SECTOR
{
    UCHAR BootCode[440];
    //启动记录440 Byte
    ULONG DiskSignature;
    //磁盘签名
    USHORT NoneDisk;
    //二个字节
    PARTITION_ENTRY Partition[4];
   //分区表结构64 Byte
    USHORT Signature;
   //结束标志2 Byte 55 AA
} MBR_SECTOR, *PMBR_SECTOR;

#pragma pack()


#define FIRST_DISK L"\\\\.\\PHYSICALDRIVE0"








































































