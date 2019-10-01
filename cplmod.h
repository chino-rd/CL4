/*
*CL4 Driver
*Copyright (C) 2019 AR'S
*
*licence:GPLv2
*/


#ifndef CPLMOD_H
#define CPLMOD_H

/**		CPL*************************/

#define MEM_SHARED_BASE 0x3F809800
#define MEM_SHARED_SIZE 0x7F6800
#define MEM_SHARED_END			(MEM_SHARED_BASE + MEM_SHARED_SIZE)

#define CPL_PACKET_SIZE			1280
#define CPL_UDP_PACKET_SIZE	1282

#define CPL_FRAME_ALLOC	      1280*128
#define CPL_OFF_FRAME1	      0
#define CPL_OFF_FRAME2	      (1*CPL_FRAME_ALLOC)			// 0x3F831800
#define CPL_OFF_FRAMESELECT	  (2*CPL_FRAME_ALLOC)			// 0x3F859800


#define CPL_WEBDATA_ALLOC		1024

#define CPL_WEBDATA_TX_OFF		(MEM_SHARED_SIZE - sizeof(webdata_t))			// To baremetal
#define CPL_WEBDATA_RX_OFF		(MEM_SHARED_SIZE - (2*sizeof(webdata_t)))	// From baremetal
#define CPL_WEBDATA_TX_BASE		(MEM_SHARED_BASE + CPL_WEBDATA_TX_OFF)
#define CPL_WEBDATA_RX_BASE		(MEM_SHARED_BASE + CPL_WEBDATA_RX_OFF)


#define CMD_SET_IP			"SETIP"
#define CMD_SET_PORT		"SETPORT"
#define CMD_SET_MAC			"SETMAC"


#define CPL_LINE_PACKET_SIZE	1284
#define XMAX	320
#define YMAX	240
#define FRAME_SET	4
#define FRAME_SET_MAX	(YMAX / (2*FRAME_SET))
#define BRAM_SIZE	16384
#define BRAM_FRAME1	0
#define BRAM_FRAME2	BRAM_SIZE/2
#define CPL_LINE_FRAME_ALLOC	CPL_LINE_PACKET_SIZE * 128

//#define WEB_IMAGE_OFFSET	0x000F6800

#define WEB_BRAM_ADDRESS	0x12000000
#define WEB_BRAM_SIZE	0x29000
#define READ_BRAM_ADDRESS	0x12030000
#define READ_BRAM_SIZE	0x1000
#define SENSORS_BRAM_ADDRESS	0x12031000
#define SENSORS_BRAM_SIZE	0x1000


typedef struct {
	char cstring[32];
	char clen;
	int (*handler)(const char*, size_t);
} cpl_command_t;

typedef struct {
	unsigned char pending;
	unsigned int dlen;
	unsigned int etc;
} webdata_header_t;

typedef struct {
	u32	pending;
	u32	dlen;
	u32	etc;
	char	data[CPL_WEBDATA_ALLOC];
} webdata_t;

typedef struct {
	unsigned char data[CPL_PACKET_SIZE];
	u32	reserve;
} packetdata_t;


int cpl_init(struct platform_device *pdev);
int cpl_sendpacket(char* payload, int pno, int len);
int cpl_sendpacket_b(char* payload, int pno, int len);
int cpl_sendpacket_udp(char* payload, u32 len);
ssize_t baremetal_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count);
ssize_t baremetal_data_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count);
ssize_t webimage_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count);

int cpl_setip(const char*, size_t);
int cpl_setport(const char*, size_t);
int cpl_setmac(const char*, size_t);

#endif
