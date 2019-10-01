/*
*CL4 Driver
*Copyright (C) 2019 AR'S
*
*Based on origin Zynq Remote Processor driver  
*Copyright (C) 2012 Michal Simek <monstr@monstr.eu>
*Copyright (C) 2012 PetaLogix
*
*licence:GPLv2
*/


/*
 * Zynq Remote Processor driver
 *
 * Copyright (C) 2012 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2012 PetaLogix
 *
 * Based on origin OMAP Remote Processor driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */




#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/smp.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/outercache.h>
#include <linux/slab.h>
#include <linux/cpu.h>

#include "remoteproc_internal.h"
#include <_cpl/cplmod.h>
#include <linux/of_address.h>

#include <linux/skbuff.h>
#include <linux/udp.h>
#include <linux/ip.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>

//#include <stdio.h>
//#include <stdlib.h>
//#include <linux/mm.h>
//#include <asm/types.h>
//#include <asm/stat.h>
//#include <fcntl.h>
//#include <linux/fs.h>
#include <linux/init.h> //add for gpio
#include <linux/io.h>   //add for gpio
#include <linux/gpio.h> //add for gpio

#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/dmaengine.h>
#include <asm/io.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>

#define PRINT 0
#define DDR_SELECT 0

extern int zynq_cpun_start(u32 address, int cpu);

/* Module parameter */
static char *firmware;

/* Structure for storing IRQs */
struct irq_list {
	int irq;
	struct list_head list;
};

/* Private data */
struct zynq_rproc_pdata {
	struct irq_list mylist;
	struct rproc *rproc;
	u32 vring0;
	u32 vring1;
	u32 mem_start;
	u32 mem_end;
};

/* Store rproc for IPI handler */
static struct platform_device *remoteprocdev;
static struct work_struct workqueue;


static void __iomem *ddr_shared;        // Pointer to shared memory region
static void __iomem *bram_shared;	  // Pointer to shared bram memory region
static void __iomem *web_shared;        // Pointer to shared web memory region
static void __iomem *sensors_shared;        // Pointer to shared web memory region

static struct net_device *dev;  // Pointer to network interface device ("ETH0")
static u32 frame_ready_irq;

static int port_src = 10000;
static int port_dst = 10000;

static char destmac[6] = {0}; // = {0x2c, 0x41, 0x38, 0x8e, 0x62, 0xe1};
static char srcmac[6] = {0x00, 0x0c, 0x35, 0x00, 0x1e, 0x53};

static u32 ip_dest=0;
//static char ip_src[4];

static int line_set_count = 0;

#define IPSRC "192.168.11.72"
#define IPDST "192.168.11.2"

static const cpl_command_t cpl_command_list[] = {
	{CMD_SET_IP, strlen(CMD_SET_IP), &cpl_setip},
	{CMD_SET_PORT, strlen(CMD_SET_PORT), &cpl_setport},
	{CMD_SET_MAC, strlen(CMD_SET_MAC), &cpl_setmac},
	{"NULL",0, NULL}
};

//static void __iomem *ddr_cpl_addr;

/*** END CPL ******************************/


static void handle_event(struct work_struct *work)
{
	struct zynq_rproc_pdata *local = platform_get_drvdata(remoteprocdev);

	if (rproc_vq_interrupt(local->rproc, 0) == IRQ_NONE)
		dev_dbg(&remoteprocdev->dev, "no message found in vqid 0\n");
}

static void frame_ready(void){
	int i, bufselect;
	if (ip_dest == 0){
		// no destination IP assigned so just return
		return;
	}
	if (destmac[0] == 0 && memcmp(destmac, destmac+1, sizeof(destmac)-1) == 0){
		// MAC address is zero
		return;
	}

	// Read which buffer to transmit
	bufselect = *(char*)(ddr_shared+CPL_OFF_FRAMESELECT);
	char* framedata = ddr_shared + (bufselect*CPL_FRAME_ALLOC);

	// Queue frame data for eth transmit
	for (i = 0; i < 121; i++){
		cpl_sendpacket(framedata + CPL_PACKET_SIZE*i, i, CPL_PACKET_SIZE);
	}
}

static void line_ready(void){

	outer_inv_range(0x42000000,0x42000000+0x4000);

	int i,t;
	u32* bufselect = (u32*)(bram_shared+BRAM_FRAME2 - 4);
	u32* modeselect = (u32*)(bram_shared+BRAM_FRAME2 - 8);
	char dummyData[808] = {0};

	if (ip_dest == 0){
		// no destination IP assigned so just return
		return;
	}
	if (destmac[0] == 0 && memcmp(destmac, destmac+1, sizeof(destmac)-1) == 0){
		// MAC address is zero
		return;
	}

	char* framedata;
	framedata = bram_shared + BRAM_FRAME1;

	int work = 0;

	if(*modeselect == 0)
	{
		if(*bufselect == 0)
		{
			//printk("bufselect: %d \r\n", *bufselect);

			cpl_sendpacket(sensors_shared, 120, CPL_UDP_PACKET_SIZE);
		}
		else
		{
			work = (int)*bufselect;

			if(*bufselect % 2 != 0)
			{
				framedata = bram_shared + BRAM_FRAME1;
			}
			else
			{
				framedata = bram_shared + BRAM_FRAME2;
			}

			for(i=0;i<4;i++)
			{
				cpl_sendpacket(framedata + CPL_LINE_PACKET_SIZE*i, 4*(work - 1) + i, CPL_UDP_PACKET_SIZE);
			}

		}
	}
	else
	{

#if DDR_SELECT == 0
		if(*bufselect == 0)
		{
			//printk("bufselect: %d \r\n", *bufselect);

			cpl_sendpacket(sensors_shared, 288, 52);//CPL_PACKET_SIZE
		}
		else
		{
			work = (int)*bufselect;

			if(*bufselect % 2 != 0)
			{
				framedata = bram_shared + BRAM_FRAME1;
			}
			else
			{
				framedata = bram_shared + BRAM_FRAME2;
			}

			if((*bufselect<=24)||(265 <= *bufselect))
			{
				cpl_sendpacket(dummyData, work - 1, 808);//CPL_PACKET_SIZE
				//printk("bufselect: %d \r\n", *bufselect);
			}
			else
			{
				cpl_sendpacket(framedata, work - 1, 808);//CPL_PACKET_SIZE
			}
		}
#else
		framedata = ddr_shared;

		for(t=0;t<289;t++)
		{

			if(t<24)
			{
				cpl_sendpacket_b(dummyData, t, 808);//CPL_PACKET_SIZE
			}
			else if((24<=t)&&(t<264))
			{
				cpl_sendpacket_b(framedata + 800*(t-24), t, 808);//CPL_PACKET_SIZE
			}
			else if((264<=t)&&(t<288))
			{
				cpl_sendpacket_b(dummyData, t, 808);//CPL_PACKET_SIZE
			}
			else if(t==288)
			{
				cpl_sendpacket(sensors_shared, 288, 52);//CPL_PACKET_SIZE
			}
			else
			{
				cpl_sendpacket(sensors_shared, 288, 52);//CPL_PACKET_SIZE
			}
		}
#endif
	}
}

//volatile static u32 tmpheader[3];
//volatile static u8 dbyte;
static void ipi_kick(void)
{
	//gpio_set_value(937, 0);
	printk("ipi kick\r\n");
	// Force Linux to read from RAM instead of L2 cache for the received data
	outer_inv_range(CPL_WEBDATA_RX_BASE, CPL_WEBDATA_RX_BASE+sizeof(webdata_t));

	volatile webdata_t *baremetal_data = (webdata_t*)(ddr_shared + CPL_WEBDATA_RX_OFF);

#if PRINT==1
	printk("BM -> Linux kick\r\n");
	printk("Data Address: 0x%08p\r\n", baremetal_data);
	printk("data len: 0x%08lx bytes\r\n", baremetal_data->dlen);
	printk("data[0]: 0x%02x\r\n", baremetal_data->data[0]);
#endif

	cpl_sendpacket_udp(baremetal_data->data,baremetal_data->dlen);

	baremetal_data->pending = 0;

  //gpio_set_value(936, 0);
  //gpio_set_value(937, 0);
}

static int zynq_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	dev_dbg(dev, "%s\n", __func__);
	INIT_WORK(&workqueue, handle_event);


	mb();
	remoteprocdev = pdev;
	ret = zynq_cpun_start(rproc->bootaddr, 1);

	return ret;
}

/* kick a firmware */
static void zynq_rproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct zynq_rproc_pdata *local = platform_get_drvdata(pdev);

	dev_dbg(dev, "KICK Firmware to start send messages vqid %d\n", vqid);

	/* Send swirq to firmware */
	if (!vqid)
		gic_raise_softirq(cpumask_of(1), local->vring0);
	else
		gic_raise_softirq(cpumask_of(1), local->vring1);
}

/* power off the remote processor */
static int zynq_rproc_stop(struct rproc *rproc)
{
	dev_dbg(rproc->dev.parent, "%s\n", __func__);

	/* FIXME missing reset option */
	return 0;
}

static struct rproc_ops zynq_rproc_ops = {
	.start		= zynq_rproc_start,
	.stop		= zynq_rproc_stop,
	.kick		= zynq_rproc_kick,
};

/* Just to detect bug if interrupt forwarding is broken */
static irqreturn_t zynq_remoteproc_interrupt(int irq, void *dev_id)
{
	struct device *dev = dev_id;

	dev_err(dev, "GIC IRQ %d is not forwarded correctly\n", irq);

	/*
	 *  MS: Calling this function doesn't need to be BUG
	 * especially for cases where firmware doesn't disable
	 * interrupts. In next probing can be som interrupts pending.
	 * The next scenario is for cases when you want to monitor
	 * non frequent interrupt through Linux kernel. Interrupt happen
	 * and it is forwarded to Linux which update own statistic
	 * in (/proc/interrupt) and forward it to firmware.
	 *
	 * gic_set_cpu(1, irq);	- setup cpu1 as destination cpu
	 * gic_raise_softirq(cpumask_of(1), irq); - forward irq to firmware
	 */

	gic_set_cpu(1, irq);
	return IRQ_HANDLED;
}

static void clear_irq(struct platform_device *pdev)
{
	struct list_head *pos, *q;
	struct irq_list *tmp;
	struct zynq_rproc_pdata *local = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Deleting the irq_list\n");
	list_for_each_safe(pos, q, &local->mylist.list) {
		tmp = list_entry(pos, struct irq_list, list);
		free_irq(tmp->irq, &pdev->dev);
		gic_set_cpu(0, tmp->irq);
		list_del(pos);
		kfree(tmp);
	}
}

/**
	*		Device file methods
	*/
// Set the configuration of this module
ssize_t cpl_linux_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	cpl_command_t *command;

	for (command = cpl_command_list; command->clen!=0; command++){
		if ((count > command->clen) && (strncmp(command->cstring, buf, command->clen)==0)){
			// Matched command
			return command->handler(buf, count);
		}
	}

	return -EINVAL;
}

// Read the configuration of this module
ssize_t cpl_linux_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count){
	char obuf[64], oc;
	unsigned char ip[4];

	memcpy(ip, &ip_dest, 4);
	oc = sprintf(obuf,"Target Port=%d IP=%u.%u.%u.%u MAC=%02X:%02X:%02X:%02X:%02X:%02X",
		port_dst, ip[0], ip[1], ip[2], ip[3] ,destmac[0],destmac[1],destmac[2],destmac[3],destmac[4],destmac[5] );

	memcpy(buf, obuf, oc);
	return oc;
}
static DEVICE_ATTR_RW(cpl_linux);

// Send data to baremetal - invalidates any currently pending messages
ssize_t baremetal_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
  	//gpio_set_value(936, 1);
	//gpio_set_value(937, 1);
	struct rproc *rproc = container_of(dev, struct rproc, dev);

  outer_inv_range(CPL_WEBDATA_TX_BASE, CPL_WEBDATA_TX_BASE+sizeof(webdata_t));
  outer_inv_range(CPL_WEBDATA_RX_BASE, CPL_WEBDATA_RX_BASE+sizeof(webdata_t));

	volatile webdata_t *webdata = (webdata_t*)(ddr_shared + CPL_WEBDATA_TX_OFF);

	if (webdata->pending){
		dev_dbg(dev, "CPL: Pending flag still set\n");
		return -EBUSY;
	}

	if (count > CPL_WEBDATA_ALLOC){
		dev_err(dev, "Max payload = %d bytes\n", CPL_WEBDATA_ALLOC);
		return -E2BIG;
	}

	if (count == 0){
		return -EINVAL;
	}

#if PRINT==1
	printk("Sending BM %d bytes\n", count);
#endif

	// Copy data and flag pending
	memcpy(webdata->data, buf, count);
	webdata->dlen = count;
	webdata->pending = 1;

#if PRINT==1
	printk("Linux -> BM write \r\n");
	printk("Address 0x%08p \r\n",webdata);
	printk("pending 0x%08lx \r\n",webdata->pending);
	printk("dlen 0x%08lx \r\n",webdata->dlen);
	printk("data[0] 0x%02x \r\n",webdata->data[0]);
	printk("Rx Address 0x%08p \r\n",webdata);
#endif

	// Notify CPU1
	zynq_rproc_kick(rproc, 1);
	//mdelay(50);
	return count;
}

// Read data sent by baremetal
ssize_t baremetal_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count){

	outer_inv_range(CPL_WEBDATA_TX_BASE, CPL_WEBDATA_TX_BASE+sizeof(webdata_t));
  outer_inv_range(CPL_WEBDATA_RX_BASE, CPL_WEBDATA_RX_BASE+sizeof(webdata_t));

	volatile webdata_t *webdata = (webdata_t*)(ddr_shared + CPL_WEBDATA_RX_OFF);
	volatile u32 available;

#if PRINT==1
	printk("read pointer \r\n");
	printk("Address 0x%08p \r\n",webdata);
	printk("pending 0x%08lx \r\n",webdata->pending);
	printk("dlen 0x%08lx \r\n",webdata->dlen);
	printk("data[0] 0x%02x \r\n",webdata->data[0]);
#endif

	if (!webdata->pending){
		return -ENOMSG;
	}

	if (webdata->dlen > count){
		return -ENOMEM;
	}

	available = webdata->dlen;
	memcpy(buf, webdata->data, available);
	webdata->pending = 0;
	return available;

}
static DEVICE_ATTR_RW(baremetal);

// Read data sent by baremetal
ssize_t baremetal_data_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count){

	volatile char* baredata = (char*)(ddr_shared + CPL_OFF_FRAME2);
	u32 available;

	printk("read ddr\r\n");

	available = CPL_FRAME_ALLOC;
	memcpy(buf, baredata, available);

	printk("read length %08x\r\n",available);

	return available;
}
static DEVICE_ATTR_RO(baremetal_data);

// Read web data sent by baremetal
ssize_t webimage_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count){

	volatile char* webImageSelect = (char*)(web_shared);
	volatile char* webImageData = (char*)(web_shared + 4);
	u32 available;
	u32 offset = 0;//(*webImageSelect) * CPL_LINE_FRAME_ALLOC

	printk("read ddr(web image data)...offset : %08x\r\n",offset);

	printk("read ddr(web image data)...count : %d\r\n",count);

	available = 5000;//CPL_LINE_PACKET_SIZE * 120

	//buf = (char*)malloc(5000);

	memcpy(buf, webImageData + offset, available);

	//mdelay(1000);

	printk("read length(web image data) %08x\r\n",available);

	printk("read data[0](web image data) %x\r\n",buf[0]);

	return available;//
}
static DEVICE_ATTR_RO(webimage);

// Check if any data is available
ssize_t pending_show(struct device* dev, struct device_attribute *attr, char *buf, size_t count){
	outer_inv_range(CPL_WEBDATA_RX_BASE, CPL_WEBDATA_RX_BASE+sizeof(webdata_t));
	outer_inv_range(CPL_WEBDATA_TX_BASE, CPL_WEBDATA_TX_BASE+sizeof(webdata_t));

	volatile webdata_t *webdata = (webdata_t*)(ddr_shared + CPL_WEBDATA_RX_OFF);
	char pending = webdata->pending;
	// memcpy(buf,&pending,1);
	return sprintf(buf, "%u", pending);
}
static DEVICE_ATTR_RO(pending);

static int zynq_remoteproc_probe(struct platform_device *pdev)
{
	const unsigned char *prop;
	struct resource *res; /* IO mem resources */
	int ret = 0;
	struct irq_list *tmp;
	int count = 0;
	struct zynq_rproc_pdata *local;

	ret = cpl_init(pdev);
	if (ret != 0){
		dev_err(&pdev->dev, "Error %d: initialising CPL module\n", ret);
		return -1;
	}

	ret = cpu_down(1);
	/* EBUSY means CPU is already released */
	if (ret && (ret != -EBUSY)) {
		dev_err(&pdev->dev, "Can't release cpu1\n");
		return -ENOMEM;
	}

	local = devm_kzalloc(&pdev->dev, sizeof(struct zynq_rproc_pdata),
			     GFP_KERNEL);
	if (!local)
		return -ENOMEM;

	platform_set_drvdata(pdev, local);

	/* Declare memory for firmware */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "invalid address\n");
		return -ENODEV;
	}

	local->mem_start = res->start;
	local->mem_end = res->end;

	/* Alloc phys addr from 0 to max_addr for firmware */
	ret = dma_declare_coherent_memory(&pdev->dev, local->mem_start,
		local->mem_start, local->mem_end - local->mem_start + 1,
		DMA_MEMORY_IO);
	if (!ret) {
		dev_err(&pdev->dev, "dma_declare_coherent_memory failed\n");
		ret = -ENOMEM;
		goto dma_fault;
	}

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		goto dma_mask_fault;
	}

	/* Init list for IRQs - it can be long list */
	INIT_LIST_HEAD(&local->mylist.list);

	/* Alloc IRQ based on DTS to be sure that no other driver will use it */
	while (1) {
		int irq;

		irq = platform_get_irq(pdev, count++);
		if (irq == -ENXIO || irq == -EINVAL)
			break;

		tmp = kzalloc(sizeof(struct irq_list), GFP_KERNEL);
		if (!tmp) {
			dev_err(&pdev->dev, "Unable to alloc irq list\n");
			ret = -ENOMEM;
			goto irq_fault;
		}

		tmp->irq = irq;

		dev_dbg(&pdev->dev, "%d: Alloc irq: %d\n", count, tmp->irq);

		/* Allocating shared IRQs will ensure that any module will
		 * use these IRQs */
		ret = request_irq(tmp->irq, zynq_remoteproc_interrupt, 0,
					dev_name(&pdev->dev), &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "IRQ %d already allocated\n",
								tmp->irq);
			goto irq_fault;
		}

		/*
		 * MS: Here is place for detecting problem with firmware
		 * which doesn't work correctly with interrupts
		 *
		 * MS: Comment if you want to count IRQs on Linux
		 */
		gic_set_cpu(1, tmp->irq);
		list_add(&(tmp->list), &(local->mylist.list));
	}

	/* Allocate free IPI number */
	/* Read vring0 ipi number */
	ret = of_property_read_u32(pdev->dev.of_node, "vring0", &local->vring0);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to read property");
		goto ipi_fault;
	}

	ret = set_ipi_handler(local->vring0, ipi_kick, "Firmware kick");
	if (ret) {
		dev_err(&pdev->dev, "IPI handler already registered\n");
		goto irq_fault;
	}

	/* Read vring1 ipi number */
	ret = of_property_read_u32(pdev->dev.of_node, "vring1", &local->vring1);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to read property");
		goto ipi_fault;
	}

	/* Module param firmware first */
	if (firmware)
		prop = firmware;
	else
		prop = of_get_property(pdev->dev.of_node, "firmware", NULL);

	if (prop) {
		dev_dbg(&pdev->dev, "Using firmware: %s\n", prop);
		local->rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
				&zynq_rproc_ops, prop, sizeof(struct rproc));
		if (!local->rproc) {
			dev_err(&pdev->dev, "rproc allocation failed\n");
			goto ipi_fault;
		}

		ret = rproc_add(local->rproc);
		if (ret) {
			dev_err(&pdev->dev, "rproc registration failed\n");
			goto rproc_fault;
		}

		device_create_file(&local->rproc->dev, &dev_attr_pending);
		device_create_file(&local->rproc->dev, &dev_attr_baremetal);
		device_create_file(&local->rproc->dev, &dev_attr_baremetal_data);
		device_create_file(&local->rproc->dev, &dev_attr_cpl_linux);
		device_create_file(&local->rproc->dev, &dev_attr_webimage);

		return ret;
	} else
		ret = -ENODEV;

rproc_fault:
	rproc_put(local->rproc);
ipi_fault:
	clear_ipi_handler(local->vring0);

irq_fault:
	clear_irq(pdev);

dma_mask_fault:
	dma_release_declared_memory(&pdev->dev);

dma_fault:
	/* Cpu can't be power on - for example in nosmp mode */
	ret |= cpu_up(1);
	if (ret)
		dev_err(&pdev->dev, "Can't power on cpu1 %d\n", ret);

	return ret;
}

static int zynq_remoteproc_remove(struct platform_device *pdev)
{
	struct zynq_rproc_pdata *local = platform_get_drvdata(pdev);
	u32 ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	dma_release_declared_memory(&pdev->dev);

	clear_ipi_handler(local->vring0);
	clear_irq(pdev);

	rproc_del(local->rproc);
	rproc_put(local->rproc);

	/* Cpu can't be power on - for example in nosmp mode */
	ret = cpu_up(1);
	if (ret)
		dev_err(&pdev->dev, "Can't power on cpu1 %d\n", ret);

	return 0;
}

/* Match table for OF platform binding */
static const struct of_device_id zynq_remoteproc_match[] = {
	{ .compatible = "xlnx,zynq_remoteproc", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, zynq_remoteproc_match);

static struct platform_driver zynq_remoteproc_driver = {
	.probe = zynq_remoteproc_probe,
	.remove = zynq_remoteproc_remove,
	.driver = {
		.name = "zynq_remoteproc",
		.of_match_table = zynq_remoteproc_match,
	},
};
module_platform_driver(zynq_remoteproc_driver);

module_param(firmware, charp, 0);
MODULE_PARM_DESC(firmware, "Override the firmware image name. Default value in DTS.");

//This file has been modified.
//MODULE_AUTHOR("Michal Simek <monstr@monstr.eu");
//MODULE_LICENSE("GPL v2");
//MODULE_DESCRIPTION("Zynq remote processor control driver");

MODULE_AUTHOR("AR'S");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Zynq remote processor control driver(modified)");


/**
	Initialise the CPL shared memory and default values.
	Currently it is spliced into a device driver
			(e.g. gpio-xilinx or zynq_remoteproc)

	Return: 0 on success, -1 on error
*/
int cpl_init(struct platform_device *pdev){

		struct resource r;
		struct in_device* in_dev;
		struct in_ifaddr* if_info;

		struct resource r_b;//For BRAM

		struct resource r_c;//DDR3 For webImage

		struct resource r_d;//SDRAM For sensors


		// Attempt to acquire shared memory region specified in device tree
		struct device_node *node = pdev->dev.of_node;
		struct device_node *sharedmem = of_parse_phandle(node, "memory-region", 0);

		struct device_node *sharedmem_bram = of_parse_phandle(node, "memory-region_b", 0);

		struct device_node *sharedmem_web = of_parse_phandle(node, "memory-region_c", 0);

		struct device_node *sharedmem_sensors = of_parse_phandle(node, "memory-region_d", 0);

		u32 ret;

		if (!sharedmem){
			printk("cpl_init: could not find 'memory-region' entry for this device\r\n");
			return -1;
		}

		if (of_address_to_resource(sharedmem, 0, &r) != 0){
			printk("cpl_init: could not resolve resource\r\n");
			return -2;
		}

		ddr_shared = memremap(r.start, resource_size(&r), MEMREMAP_WC);
		printk("ddr_shared Address memremap wc 0x%08p \r\n",ddr_shared);
		//ioremap_nocache(r.start, resource_size(&r))
		if (!ddr_shared){
			printk("cpl_init: could not map shared memory\r\n");
			return -3;
		}

		printk("cpl_init: mapped shared memory @%X\r\n", r.start);

		/*
		u32* workddr = (u32*)ddr_shared;
		int p=0;

		for(p=0;p<MEM_SHARED_SIZE/4;p++)
		{
			workddr[p] = 0;
		}
		*/

		if (!sharedmem_bram){
			printk("cpl_init: could not find 'memory-region_b' entry for this device\r\n");
			return -1;
		}

		if (of_address_to_resource(sharedmem_bram, 0, &r_b) != 0){
			printk("cpl_init: could not resolve resource\r\n");
			return -2;
		}

		bram_shared = memremap(r_b.start, resource_size(&r_b), MEMREMAP_WC);//
		printk("bram_shared Address memremap wc 0x%08p \r\n",bram_shared);
		//bram_shared = ioremap_nocache(r_b.start, resource_size(&r_b));//
		//printk("bram_shared Address ioremap 0x%08p \r\n",bram_shared);
		if (!bram_shared){
			printk("cpl_init: could not map shared bram memory\r\n");
			return -3;
		}

		printk("cpl_init: mapped shared bram memory @%X\r\n", r_b.start);



		if (!sharedmem_web){
			printk("cpl_init: could not find 'memory-region_c' entry for this device\r\n");
			return -1;
		}

		if (of_address_to_resource(sharedmem_web, 0, &r_c) != 0){
			printk("cpl_init: could not resolve resource\r\n");
			return -2;
		}

		web_shared = memremap(r_c.start, resource_size(&r_c), MEMREMAP_WC);
		//ioremap_nocache(r_c.start, resource_size(&r_c)), MEMREMAP_WC
		if (!web_shared){
			printk("cpl_init: could not map shared web memory\r\n");
			return -3;
		}

		printk("cpl_init: mapped shared web memory @%X\r\n", r_c.start);




		if (!sharedmem_sensors){
			printk("cpl_init: could not find 'memory-region_d' entry for this device\r\n");
			return -1;
		}

		if (of_address_to_resource(sharedmem_sensors, 0, &r_d) != 0){
			printk("cpl_init: could not resolve resource\r\n");
			return -2;
		}

		sensors_shared = memremap(r_d.start, resource_size(&r_d), MEMREMAP_WC);
		//ioremap_nocache(r_c.start, resource_size(&r_c)), MEMREMAP_WC
		if (!sensors_shared){
			printk("cpl_init: could not map shared sensors memory\r\n");
			return -3;
		}

		printk("cpl_init: mapped shared sensors memory @%X\r\n", r_d.start);





		outer_inv_range(MEM_SHARED_BASE, MEM_SHARED_BASE+MEM_SHARED_SIZE);
		outer_inv_range(READ_BRAM_ADDRESS, READ_BRAM_ADDRESS+READ_BRAM_SIZE);
		outer_inv_range(WEB_BRAM_ADDRESS, WEB_BRAM_ADDRESS+WEB_BRAM_SIZE);
		outer_inv_range(SENSORS_BRAM_ADDRESS, SENSORS_BRAM_ADDRESS+SENSORS_BRAM_SIZE);
		outer_inv_range(0x12032000,0x12032000+0x00001000);
		outer_inv_range(0x40004000,0x40005000);
		outer_inv_range(0x40100000,0x40350000);

		outer_inv_range(0x10000000,0x12000000);
		outer_inv_range(0x3A000000,0x3F810000);

		//outer_disable();

		//gpio_request(936, NULL);
		//gpio_export(936, true);
		//gpio_request(937, NULL);
		//gpio_export(937, true);

		//gpio_direction_output(936, 0);
		//gpio_direction_output(937, 0);

		//gpio_set_value(936, 0);
		//gpio_set_value(937, 0);

		// Try and acquire eth0 interface (it may not be available yet)
		if (!dev){
			dev = __dev_get_by_name(&init_net, "eth0");
			if (dev == NULL){
				printk("cpl_init: could not acquire interface \"eth0\", try again on packet send\n");
			} else {

					//eth_platform_get_mac_address(dev, srcmac);//hukkatsu
					memcpy(srcmac, dev->dev_addr, dev->addr_len);
					printk("CPL: MAC for eth0_0: %X\r\n", srcmac[0]);
					printk("CPL: MAC for eth0_1: %X\r\n", srcmac[1]);
					printk("CPL: MAC for eth0_2: %X\r\n", srcmac[2]);
					printk("CPL: MAC for eth0_3: %X\r\n", srcmac[3]);
					printk("CPL: MAC for eth0_4: %X\r\n", srcmac[4]);
					printk("CPL: MAC for eth0_5: %X\r\n", srcmac[5]);

						in_dev = (struct in_device*)dev->ip_ptr;
						if_info = in_dev->ifa_list;
						for (;if_info;if_info=if_info->ifa_next){
							if(!(strcmp(if_info->ifa_label, "eth0"))){
								printk("CPL: IP for eth0: %X\n", if_info->ifa_address);
								break;
							}
						}
			}
		}

		ret = of_property_read_u32(pdev->dev.of_node, "cpl_frame", &frame_ready_irq);
		if (ret < 0) {
			dev_err(&pdev->dev, "unable to read property \"cpl_frame\"");
			return -1;
		}

		ret = set_ipi_handler(frame_ready_irq, line_ready, "CPL Frame");//frame_ready
		if (ret) {
			dev_err(&pdev->dev, "IPI handler already registered\n");
			return -1;
		}

		// Disable the frame signal IRQ by default
		disable_irq(frame_ready_irq);
		return 0;

}
// EXPORT_SYMBOL_GPL(cpl_init);

int cpl_sendpacket(char* payload, int pno, int len){

		struct sk_buff *skb;
		struct udphdr *udph;
		struct iphdr  *iph;
		struct ethhdr *eth;

		struct in_device* in_dev;
		struct in_ifaddr* if_info;

		u32 my_ip_addr = 0;

		static int dev_get_attempt=0;
		static const int header_sz  = sizeof(struct udphdr) + sizeof(struct iphdr) + sizeof(struct ethhdr);

		// Acquire eth0 interface
		if (!dev){

			dev = __dev_get_by_name(&init_net, "eth0");

			if (dev == NULL){

				if (++dev_get_attempt == 100){
					printk("cpl_sendpacket: failed to acquire \"eth0\" after 100 packets\r\n");
				}
				return -1;
			}
		}

		// Continuous updates of local IP address
		in_dev = (struct in_device*)dev->ip_ptr;
		if_info = in_dev->ifa_list;
		for (;if_info;if_info=if_info->ifa_next){
			if(!(strcmp(if_info->ifa_label, "eth0"))){
				my_ip_addr = if_info->ifa_address;
				break;
			}
		}
		if (!my_ip_addr){
			// No IP on this interface
			return -2;
		}

		// Allocate memory for new packet -> released after the scheduler processes it from the queue
		// @note 3 bytes are added to the end of the packet for a "CPL" tag for the queue
		//    priority manager
		skb = alloc_skb(header_sz + len, GFP_KERNEL);//CPL_UDP_PACKET_SIZE
		if (!skb){
			//printk("Error creating SKB\n");
			return -1;
		}

		// Reserve space in SKB for header (~42 bytes)
		skb_reserve(skb, header_sz);

		// Attach payload
		// @note this is where the actual CPL data is read from whichever memory model is used to store it
		skb->data = (void*)skb_put(skb, len);//CPL_UDP_PACKET_SIZE
		// memcpy(skb->data, "CPL", 3);
		memcpy(skb->data, &pno, 2);
		memcpy(skb->data + 2, payload, len);


		#if 0
			if (pno = 119){
				printk("CPL: Packet #:%d\n", pno);
				for (i = 0; i < 50; i++){
					printk("%X ", *(payload + i + 2));
				}
				printk("\n\n");
			}
		#endif

		// Build UDP layer
		skb_push(skb, sizeof(struct udphdr));
		skb_reset_transport_header(skb);

		udph = udp_hdr(skb);
		udph->source = htons(port_src);
		udph->dest = htons(port_dst);
		udph->len = htons(sizeof(struct udphdr) + len);	// Including "CPL"CPL_UDP_PACKET_SIZE
		udph->check = 0;

		// Build IP layer
		skb_push(skb, sizeof(struct iphdr));
		skb_reset_network_header(skb);

		iph = ip_hdr(skb);
		iph->version = 4;
		iph->ihl = 5;
		iph->tos = 0x14;
		iph->tot_len = htons(skb->len);
		iph->id = 0;
		iph->frag_off = 0;
		iph->ttl = 64;
		iph->protocol = IPPROTO_UDP;
		iph->saddr = my_ip_addr;
		iph->daddr = ip_dest;
		iph->check = 0;

		// Build MAC layer
		skb_push(skb, sizeof(struct ethhdr));
		skb_reset_mac_header(skb);

		eth = eth_hdr(skb);
		memcpy(eth->h_dest, destmac, sizeof(destmac));
		memcpy(eth->h_source, srcmac, sizeof(srcmac));
		eth->h_proto = htons(ETH_P_IP);

		skb->dev = dev;
		skb->priority=6;
		memcpy(&skb->cb[45], "CPL", 3);

		// Send packet to network interface
		if (dev_queue_xmit(skb) < 0){
			printk("Error submitting packet\n");
		}

		return 0;
}

int cpl_sendpacket_b(char* payload, int pno, int len){

		struct sk_buff *skb;
		struct udphdr *udph;
		struct iphdr  *iph;
		struct ethhdr *eth;

		struct in_device* in_dev;
		struct in_ifaddr* if_info;

		u32 my_ip_addr = 0;

		static int dev_get_attempt=0;
		static const int header_sz  = sizeof(struct udphdr) + sizeof(struct iphdr) + sizeof(struct ethhdr);

		// Acquire eth0 interface
		if (!dev){

			dev = __dev_get_by_name(&init_net, "eth0");

			if (dev == NULL){

				if (++dev_get_attempt == 100){
					printk("cpl_sendpacket: failed to acquire \"eth0\" after 100 packets\r\n");
				}
				return -1;
			}
		}

		// Continuous updates of local IP address
		in_dev = (struct in_device*)dev->ip_ptr;
		if_info = in_dev->ifa_list;
		for (;if_info;if_info=if_info->ifa_next){
			if(!(strcmp(if_info->ifa_label, "eth0"))){
				my_ip_addr = if_info->ifa_address;
				break;
			}
		}
		if (!my_ip_addr){
			// No IP on this interface
			return -2;
		}

		// Allocate memory for new packet -> released after the scheduler processes it from the queue
		// @note 3 bytes are added to the end of the packet for a "CPL" tag for the queue
		//    priority manager
		skb = alloc_skb(header_sz + len, GFP_KERNEL);//CPL_UDP_PACKET_SIZE
		if (!skb){
			//printk("Error creating SKB\n");
			return -1;
		}

		// Reserve space in SKB for header (~42 bytes)
		skb_reserve(skb, header_sz);

		// Attach payload
		// @note this is where the actual CPL data is read from whichever memory model is used to store it
		skb->data = (void*)skb_put(skb, len);//CPL_UDP_PACKET_SIZE
		// memcpy(skb->data, "CPL", 3);
		memcpy(skb->data, &pno, 2);
		memcpy(skb->data + 2, payload, len-8);


		#if 0
			if (pno = 119){
				printk("CPL: Packet #:%d\n", pno);
				for (i = 0; i < 50; i++){
					printk("%X ", *(payload + i + 2));
				}
				printk("\n\n");
			}
		#endif

		// Build UDP layer
		skb_push(skb, sizeof(struct udphdr));
		skb_reset_transport_header(skb);

		udph = udp_hdr(skb);
		udph->source = htons(port_src);
		udph->dest = htons(port_dst);
		udph->len = htons(sizeof(struct udphdr) + len);	// Including "CPL"CPL_UDP_PACKET_SIZE
		udph->check = 0;

		// Build IP layer
		skb_push(skb, sizeof(struct iphdr));
		skb_reset_network_header(skb);

		iph = ip_hdr(skb);
		iph->version = 4;
		iph->ihl = 5;
		iph->tos = 0x14;
		iph->tot_len = htons(skb->len);
		iph->id = 0;
		iph->frag_off = 0;
		iph->ttl = 64;
		iph->protocol = IPPROTO_UDP;
		iph->saddr = my_ip_addr;
		iph->daddr = ip_dest;
		iph->check = 0;

		// Build MAC layer
		skb_push(skb, sizeof(struct ethhdr));
		skb_reset_mac_header(skb);

		eth = eth_hdr(skb);
		memcpy(eth->h_dest, destmac, sizeof(destmac));
		memcpy(eth->h_source, srcmac, sizeof(srcmac));
		eth->h_proto = htons(ETH_P_IP);

		skb->dev = dev;
		skb->priority=6;
		memcpy(&skb->cb[45], "CPL", 3);

		// Send packet to network interface
		if (dev_queue_xmit(skb) < 0){
			printk("Error submitting packet\n");
		}

		return 0;
}

int cpl_sendpacket_udp(char* payload,u32 len){

//		printk("cpl_sendpacket_udp\r\n");

		struct sk_buff *skb;
		struct udphdr *udph;
		struct iphdr  *iph;
		struct ethhdr *eth;

		struct in_device* in_dev;
		struct in_ifaddr* if_info;

		u32 my_ip_addr = 0;

		static int dev_get_attempt=0;
		static const int header_sz  = sizeof(struct udphdr) + sizeof(struct iphdr) + sizeof(struct ethhdr);

		// Acquire eth0 interface
		if (!dev){

			dev = __dev_get_by_name(&init_net, "eth0");

			if (dev == NULL){

				if (++dev_get_attempt == 100){
					printk("cpl_sendpacket: failed to acquire \"eth0\" after 100 packets\r\n");
				}
				return -1;
			}
		}

		// Continuous updates of local IP address
		in_dev = (struct in_device*)dev->ip_ptr;
		if_info = in_dev->ifa_list;
		for (;if_info;if_info=if_info->ifa_next){
			if(!(strcmp(if_info->ifa_label, "eth0"))){
				my_ip_addr = if_info->ifa_address;
				break;
			}
		}
		if (!my_ip_addr){
			// No IP on this interface
			return -2;
		}

		// Allocate memory for new packet -> released after the scheduler processes it from the queue
		// @note 3 bytes are added to the end of the packet for a "CPL" tag for the queue
		//    priority manager
		skb = alloc_skb(header_sz + len, GFP_KERNEL);
		if (!skb){
			//printk("Error creating SKB\n");
			return -1;
		}

		// Reserve space in SKB for header (~42 bytes)
		skb_reserve(skb, header_sz);

		// Attach payload
		// @note this is where the actual CPL data is read from whichever memory model is used to store it
		skb->data = (void*)skb_put(skb, len);
		// memcpy(skb->data, "CPL", 3);
		//memcpy(skb->data, &pno, 2);
		memcpy(skb->data, payload, len);

//		printk("len: 0x%08lx\r\n",len);
//		printk("payload[0]: 0x%02x\r\n",payload[0]);

		#if 0
			if (pno = 119){
				printk("CPL: Packet #:%d\n", pno);
				for (i = 0; i < 50; i++){
					printk("%X ", *(payload + i + 2));
				}
				printk("\n\n");
			}
		#endif

		// Build UDP layer
		skb_push(skb, sizeof(struct udphdr));
		skb_reset_transport_header(skb);

		udph = udp_hdr(skb);
		udph->source = htons(port_src);
		udph->dest = htons(port_dst);
		udph->len = htons(sizeof(struct udphdr) + len);	// Including "CPL"
		udph->check = 0;

		// Build IP layer
		skb_push(skb, sizeof(struct iphdr));
		skb_reset_network_header(skb);

		iph = ip_hdr(skb);
		iph->version = 4;
		iph->ihl = 5;
		iph->tos = 0x14;
		iph->tot_len = htons(skb->len);
		iph->id = 0;
		iph->frag_off = 0;
		iph->ttl = 64;
		iph->protocol = IPPROTO_UDP;
		iph->saddr = my_ip_addr;
		iph->daddr = ip_dest;
		iph->check = 0;

		// Build MAC layer
		skb_push(skb, sizeof(struct ethhdr));
		skb_reset_mac_header(skb);

		eth = eth_hdr(skb);
		memcpy(eth->h_dest, destmac, sizeof(destmac));
		memcpy(eth->h_source, srcmac, sizeof(srcmac));
		eth->h_proto = htons(ETH_P_IP);

		skb->dev = dev;
		skb->priority=6;
		memcpy(&skb->cb[45], "CPL", 3);

		// Send packet to network interface
		if (dev_queue_xmit(skb) < 0){
			printk("Error submitting packet\n");
		}

		return 0;
}


int cpl_setip(const char *buf, size_t count){

	unsigned char ip[4] = {0};

	sscanf(buf, "SETIP,%hhd.%hhd.%hhd.%hhd", &ip[0], &ip[1], &ip[2], &ip[3]);
	memcpy(&ip_dest, ip, 4);
	printk("New IP: %hhu.%hhu.%hhu.%hhu\n", ip[0], ip[1], ip[2], ip[3]);

	return count;
}

int cpl_setport(const char *buf, size_t count){

	unsigned int portno;
	sscanf(buf, "SETPORT,%d", &portno);

	port_dst = portno&0xFFFF;
	printk("Setting port to %u\n", port_dst);

	return count;
}

int cpl_setmac(const char* buf, size_t count){

	char c;
	u16 tmp[6];

	sscanf(buf, "SETMAC,%hhX:%hhX:%hhx:%hhX:%hhx:%hhX%*c",
		&tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5], &c);

	for (c=0; c < sizeof(destmac); c++){
		destmac[c] = tmp[c]&0xFF;
	}

	printk("Setting MAC to %02X:%02X:%02X:%02X:%02X:%02X\n",
		destmac[0], destmac[1], destmac[2], destmac[3], destmac[4], destmac[5]);

	return count;
}
