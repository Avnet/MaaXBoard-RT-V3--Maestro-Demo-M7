/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016, 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "usb_host_config.h"
#include "usb_host.h"
#include "usb_host_msd.h"
#include "host_msd_command.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_device_registers.h"
#include "main.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if MSD_THROUGHPUT_TEST_ENABLE
#include "fsl_device_registers.h"
#define THROUGHPUT_BUFFER_SIZE (64 * 1024) /* throughput test buffer */
#define MCU_CORE_CLOCK         (120000000) /* mcu core clock, user need to configure it. */
#warning "Please check the value of MCU_CORE_CLOCK to make sure it is the right CPU clock!"
#endif /* MSD_THROUGHPUT_TEST_ENABLE */

/*! @brief msd class handle array for fatfs */
extern usb_host_class_handle g_UsbFatfsClassHandle;

usb_host_msd_fatfs_instance_t g_MsdFatfsInstance; /* global msd fatfs instance */
static FATFS fatfs;
/* control transfer on-going state. It should set to 1 when start control transfer, it is set to 0 in the callback */
volatile uint8_t controlIng;
/* control transfer callback status */
volatile usb_status_t controlStatus;

#if MSD_FATFS_THROUGHPUT_TEST_ENABLE
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint32_t testThroughputBuffer[THROUGHPUT_BUFFER_SIZE / 4]; /* the buffer for throughput test */
uint32_t testSizeArray[] = {20 * 1024, 20 * 1024};                /* test time and test size (uint: K)*/
#else
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t testBuffer[(FF_MAX_SS > 256) ? FF_MAX_SS : 256]; /* normal test buffer */
#endif /* MSD_FATFS_THROUGHPUT_TEST_ENABLE */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief host msd ufi command callback.
 *
 * This function is used as callback function for ufi command.
 *
 * @param param      the host msd command instance pointer.
 * @param data       data buffer pointer.
 * @param dataLength data length.
 * @status           transfer result status.
 */
static void USB_HostMsdUfiCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status);

/*!
 * @brief host msd control transfer callback.
 *
 * This function is used as callback function for control transfer .
 *
 * @param param      the host msd command instance pointer.
 * @param data       data buffer pointer.
 * @param dataLength data length.
 * @status           transfer result status.
 */
static void USB_HostMsdControlCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status);

/*!
 * @brief host msd command test done.
 *
 * @param msdCommandInstance   the host command instance pointer.
 */
static void msd_command_test_done(usb_host_msd_command_instance_t *msdCommandInstance);

/*!
 * @brief host msd command test.
 *
 * This function implements msd command test.
 *
 * @param msdCommandInstance   the host command instance pointer.
 */
static void USB_HostMsdCommandTest(usb_host_msd_command_instance_t *msdCommandInstance);

/*
 * @brief usb disk mount and changed to drive directory. so it can be accessed without drive letter.
 *
 * */
static void USB_HostFatMount(usb_host_msd_fatfs_instance_t *msdFatfsInstance, app_handle_t *app_handle_ptr);

/*!
 * @brief msd fatfs test code execute done.
 */
static void USB_HostMsdFatfsTestDone(void);
/*!
 * @brief display file information.
 */
static void USB_HostMsdFatfsDisplayFileInfo(FILINFO *fileInfo);

/*!
 * @brief list files and sub-directory in one directory, the function don't check all sub-directories recursively.
 */
static FRESULT USB_HostMsdFatfsListDirectory(const TCHAR *path);
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern usb_host_handle g_HostHandle;                        /* global host handle */
usb_host_msd_command_instance_t g_MsdCommandInstance = {0}; /* global msd command instance */

/* command callback status */
volatile usb_status_t ufiStatus;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_TestUfiBuffer[512]; /*!< test buffer */

#if MSD_THROUGHPUT_TEST_ENABLE
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint32_t testThroughputBuffer[THROUGHPUT_BUFFER_SIZE / 4]; /* the buffer for throughput test */
uint32_t testSizeArray[] = {50 * 1024, 50 * 1024};                /* test time and test size (uint: K) */
#endif                                                            /* MSD_THROUGHPUT_TEST_ENABLE */

/*******************************************************************************
 * Code
 ******************************************************************************/
static void USB_HostMsdUfiCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_msd_command_instance_t *msdCommandInstance = (usb_host_msd_command_instance_t *)param;

    xSemaphoreGive(msdCommandInstance->commandSemaphore);
    ufiStatus = status;
}

static void USB_HostMsdControlCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_msd_fatfs_instance_t *msdFatfsInstance = (usb_host_msd_fatfs_instance_t *)param;

    if (msdFatfsInstance->runWaitState == kUSB_HostMsdRunWaitSetInterface) /* set interface finish */
    {
        msdFatfsInstance->runWaitState = kUSB_HostMsdRunIdle;
        msdFatfsInstance->runState     = kUSB_HostMsdRunMassStorageTest;
    }
    controlIng    = 0;
    controlStatus = status;
}

static void msd_command_test_done(usb_host_msd_command_instance_t *msdCommandInstance)
{
    vSemaphoreDelete(msdCommandInstance->commandSemaphore);
    msdCommandInstance->commandSemaphore = NULL;
    PRINTF("........................test done....................\r\n");
}

static void USB_HostMsdFatfsTestDone(void)
{
	PRINTF("............................USB mounted done......................\r\n");
}

static void USB_HostMsdFatfsDisplayFileInfo(FILINFO *fileInfo)
{
    char *fileName;
#if _USE_LFN
    fileName = (fileInfo->lfname[0] ? fileInfo->lfname : fileInfo->fname;
#else
    fileName = fileInfo->fname;
#endif /* _USE_LFN */
    /* note: if this file/directory don't have one attribute, '_' replace the attribute letter ('R' - readonly, 'H' - hide, 'S' - system) */
    PRINTF("    %s - %c%c%c - %s - %dBytes - %d-%d-%d %d:%d:%d\r\n", (fileInfo->fattrib & AM_DIR) ? "dir" : "fil",
             (fileInfo->fattrib & AM_RDO) ? 'R' : '_',
             (fileInfo->fattrib & AM_HID) ? 'H' : '_',
             (fileInfo->fattrib & AM_SYS) ? 'S' : '_',
             fileName,
             (fileInfo->fsize),
             (uint32_t)((fileInfo->fdate >> 9) + 1980) /* year */,
             (uint32_t)((fileInfo->fdate >> 5) & 0x000Fu) /* month */,
             (uint32_t)(fileInfo->fdate & 0x001Fu) /* day */,
             (uint32_t)((fileInfo->ftime >> 11) & 0x0000001Fu) /* hour */,
             (uint32_t)((fileInfo->ftime >> 5) & 0x0000003Fu) /* minute */,
             (uint32_t)(fileInfo->ftime & 0x0000001Fu) /* second */
             );
}

static FRESULT USB_HostMsdFatfsListDirectory(const TCHAR *path)
{
    FRESULT fatfsCode = FR_OK;
    FILINFO fileInfo;
    DIR dir;
    uint8_t outputLabel = 0;

#if _USE_LFN
    static uint8_t fileNameBuffer[_MAX_LFN];
    fileInfo.lfname = fileNameBuffer;
    fileInfo.lfsize = _MAX_LFN;
#endif /* _USE_LFN */

    fatfsCode = f_opendir(&dir, path);
    if (fatfsCode)
    {
        return fatfsCode;
    }
    while (1)
    {
        fatfsCode = f_readdir(&dir, &fileInfo);
        if ((fatfsCode) || (!fileInfo.fname[0]))
        {
            break;
        }
        outputLabel = 1;
        USB_HostMsdFatfsDisplayFileInfo(&fileInfo);
    }
    if (!outputLabel)
    {
        PRINTF("\r\n");
    }

    return fatfsCode;
}

static void USB_HostFatMount(usb_host_msd_fatfs_instance_t *msdFatfsInstance, app_handle_t *app_handle_ptr)
{
	FRESULT fatfsCode;
	uint8_t driverNumberBuffer[3];
	app_handle_t *app = app_handle_ptr;

	PRINTF("[USB]  fatfs mount as logical driver %d......", USBDISK);
	sprintf((char *)&driverNumberBuffer[0], "%c:", USBDISK + '0');
	fatfsCode = f_mount(&app->fileSystem, (char const *)&driverNumberBuffer[0], 0);
	if (fatfsCode)
	{
		PRINTF("[USB]   Mount volume failed.\r\n");
		return;
	}
	PRINTF("success\r\n");

	#if (FF_FS_RPATH >= 2)
	    fatfsCode = f_chdrive((char const *)&driverNumberBuffer[0]);
	    if (fatfsCode)
	    {
	    	PRINTF("[USB]  Change drive failed.\r\n");
	        return;
	    }
	#endif

	PRINTF("[USB]  USB Disk drive mounted\r\n");
	app->usbDiskInserted = true;
	xSemaphoreGive(app->usbDiskSem);
}

void USB_HostFatUnmount()
{
	uint8_t driverNumberBuffer[3];
	sprintf((char *)&driverNumberBuffer[0], "%c:", USBDISK + '0');
	PRINTF("Unmounting USB thumb drive\r\n");
	f_unmount((char const *)&driverNumberBuffer[0]);
}

static void USB_HostMsdFatfsTest(usb_host_msd_fatfs_instance_t *msdFatfsInstance)
{
    FRESULT fatfsCode;
    FATFS *fs;
    FIL file;
    FILINFO fileInfo;
    uint32_t freeClusterNumber;
    uint32_t index;
    uint32_t resultSize;
    char *testString;
    uint8_t driverNumberBuffer[3];

#if _USE_LFN
    static uint8_t fileNameBuffer[_MAX_LFN];
    fileInfo.lfname = fileNameBuffer;
    fileInfo.lfsize = _MAX_LFN;
#endif /* _USE_LFN */

    /* time delay */
    for (freeClusterNumber = 0; freeClusterNumber < 10000; ++freeClusterNumber)
    {
        __NOP();
    }

    PRINTF("............................fatfs test.....................\r\n");

    PRINTF("fatfs mount as logical driver %d......", USBDISK);
    sprintf((char *)&driverNumberBuffer[0], "%c:", USBDISK + '0');
    fatfsCode = f_mount(&fatfs, (char const *)&driverNumberBuffer[0], 0);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");

#if (FF_FS_RPATH >= 2)
    fatfsCode = f_chdrive((char const *)&driverNumberBuffer[0]);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
#endif

#if FF_USE_MKFS
    MKFS_PARM formatOptions;
    formatOptions.fmt = FM_SFD | FM_ANY;
    PRINTF("test f_mkfs......");
    fatfsCode = f_mkfs((char const *)&driverNumberBuffer[0], &formatOptions, testBuffer, FF_MAX_SS);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
#endif /* FF_USE_MKFS */

    PRINTF("test f_getfree:\r\n");
    fatfsCode = f_getfree((char const *)&driverNumberBuffer[0], (DWORD *)&freeClusterNumber, &fs);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    if (fs->fs_type == FS_FAT12)
    {
        PRINTF("    FAT type = FAT12\r\n");
    }
    else if (fs->fs_type == FS_FAT16)
    {
        PRINTF("    FAT type = FAT16\r\n");
    }
    else
    {
        PRINTF("    FAT type = FAT32\r\n");
    }
    PRINTF("    bytes per cluster = %d; number of clusters=%lu \r\n", fs->csize * 512, fs->n_fatent - 2);
    PRINTF("    The free size: %dKB, the total size:%dKB\r\n", (freeClusterNumber * (fs->csize) / 2),
             ((fs->n_fatent - 2) * (fs->csize) / 2));

    PRINTF("directory operation:\r\n");
    PRINTF("list root directory:\r\n");
    fatfsCode = USB_HostMsdFatfsListDirectory((char const *)&driverNumberBuffer[0]);
    if (fatfsCode)
    {
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("create directory \"dir_1\"......");
    fatfsCode = f_mkdir(_T("1:/dir_1"));
    if (fatfsCode)
    {
        if (fatfsCode == FR_EXIST)
        {
            PRINTF("directory exist\r\n");
        }
        else
        {
            PRINTF("error\r\n");
            USB_HostMsdFatfsTestDone();
            return;
        }
    }
    else
    {
        PRINTF("success\r\n");
    }
    PRINTF("create directory \"dir_2\"......");
    fatfsCode = f_mkdir(_T("1:/dir_2"));
    if (fatfsCode)
    {
        if (fatfsCode == FR_EXIST)
        {
            PRINTF("directory exist\r\n");
        }
        else
        {
            PRINTF("error\r\n");
            USB_HostMsdFatfsTestDone();
            return;
        }
    }
    else
    {
        PRINTF("success\r\n");
    }
    PRINTF("create sub directory \"dir_2/sub_1\"......");
    fatfsCode = f_mkdir(_T("1:/dir_1/sub_1"));
    if (fatfsCode)
    {
        if (fatfsCode == FR_EXIST)
        {
            PRINTF("directory exist\r\n");
        }
        else
        {
            PRINTF("error\r\n");
            USB_HostMsdFatfsTestDone();
            return;
        }
    }
    else
    {
        PRINTF("success\r\n");
    }
    PRINTF("list root directory:\r\n");
    fatfsCode = USB_HostMsdFatfsListDirectory(_T("1:"));
    if (fatfsCode)
    {
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("list directory \"dir_1\":\r\n");
    fatfsCode = USB_HostMsdFatfsListDirectory(_T("1:/dir_1"));
    if (fatfsCode)
    {
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("rename directory \"dir_1/sub_1\" to \"dir_1/sub_2\"......");
    fatfsCode = f_rename(_T("1:/dir_1/sub_1"), _T("1:/dir_1/sub_2"));
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("delete directory \"dir_1/sub_2\"......");
    fatfsCode = f_unlink(_T("1:/dir_1/sub_2"));
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");

#if (FF_FS_RPATH >= 2)
    PRINTF("get current directory......");
    fatfsCode = f_getcwd((TCHAR *)&testBuffer[0], 256);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("%s\r\n", testBuffer);
    PRINTF("change current directory to \"dir_1\"......");
    fatfsCode = f_chdir(_T("dir_1"));
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("list current directory:\r\n");
    fatfsCode = USB_HostMsdFatfsListDirectory(_T("."));
    if (fatfsCode)
    {
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("get current directory......");
    fatfsCode = f_getcwd((TCHAR *)&testBuffer[0], 256);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("%s\r\n", testBuffer);
#endif

    PRINTF("get directory \"dir_1\" information:\r\n");
    fatfsCode = f_stat(_T("1:/dir_1"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    USB_HostMsdFatfsDisplayFileInfo(&fileInfo);
    PRINTF("change \"dir_1\" timestamp to 2015.10.1, 12:30:0......");
    fileInfo.fdate = ((2015 - 1980) << 9 | 10 << 5 | 1); /* 2015.10.1 */
    fileInfo.ftime = (12 << 11 | 30 << 5);               /* 12:30:00 */
    fatfsCode      = f_utime(_T("1:/dir_1"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("get directory \"dir_1\" information:\r\n");
    fatfsCode = f_stat(_T("1:/dir_1"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    USB_HostMsdFatfsDisplayFileInfo(&fileInfo);

    PRINTF("file operation:\r\n");
    PRINTF("create file \"f_1.dat\"......");
    fatfsCode = f_open(&file, _T("1:/f_1.dat"), FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if (fatfsCode)
    {
        if (fatfsCode == FR_EXIST)
        {
            PRINTF("file exist\r\n");
        }
        else
        {
            PRINTF("error\r\n");
            USB_HostMsdFatfsTestDone();
            return;
        }
    }
    else
    {
        PRINTF("success\r\n");
    }
    PRINTF("test f_write......");
    for (index = 0; index < 58; ++index)
    {
        testBuffer[index] = 'A' + index;
    }
    testBuffer[58] = '\r';
    testBuffer[59] = '\n';
    fatfsCode      = f_write(&file, testBuffer, 60, (UINT *)&resultSize);
    if ((fatfsCode) || (resultSize != 60))
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    fatfsCode = f_sync(&file);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("test f_printf......");
    if (f_printf(&file, _T("%s\r\n"), "f_printf test") == EOF)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    fatfsCode = f_sync(&file);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("test f_puts......");
    if (f_puts(_T("f_put test\r\n"), &file) == EOF)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    fatfsCode = f_sync(&file);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("test f_putc......");
    testString = "f_putc test\r\n";
    while (*testString)
    {
        if (f_putc(*testString, &file) == EOF)
        {
            PRINTF("error\r\n");
            f_close(&file);
            USB_HostMsdFatfsTestDone();
            return;
        }
        testString++;
    }
    fatfsCode = f_sync(&file);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("test f_seek......");
    fatfsCode = f_lseek(&file, 0);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("test f_gets......");
    testString = f_gets((TCHAR *)&testBuffer[0], 10, &file);
    PRINTF("%s\r\n", testString);
    PRINTF("test f_read......");
    fatfsCode = f_read(&file, testBuffer, 10, (UINT *)&resultSize);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    testBuffer[resultSize] = 0;
    PRINTF("%s\r\n", testBuffer);
#if _USE_FORWARD && _FS_TINY
    PRINTF("test f_forward......");
    fatfsCode = f_forward(&file, USB_HostMsdFatfsForward, 10, &resultSize);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("\r\n");
#endif
    PRINTF("test f_truncate......");
    fatfsCode = f_truncate(&file);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        f_close(&file);
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("test f_close......");
    fatfsCode = f_close(&file);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("get file \"f_1.dat\" information:\r\n");
    fatfsCode = f_stat(_T("1:/f_1.dat"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    USB_HostMsdFatfsDisplayFileInfo(&fileInfo);
    PRINTF("change \"f_1.dat\" timestamp to 2015.10.1, 12:30:0......");
    fileInfo.fdate = ((uint32_t)(2015 - 1980) << 9 | 10 << 5 | 1); /* 2015.10.1 */
    fileInfo.ftime = (12 << 11 | 30 << 5);                         /* 12:30:00 */
    fatfsCode      = f_utime(_T("1:/f_1.dat"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("change \"f_1.dat\" to readonly......");
    fatfsCode = f_chmod(_T("1:/f_1.dat"), AM_RDO, AM_RDO);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("get file \"f_1.dat\" information:\r\n");
    fatfsCode = f_stat(_T("1:/f_1.dat"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    USB_HostMsdFatfsDisplayFileInfo(&fileInfo);
    PRINTF("remove \"f_1.dat\" readonly attribute......");
    fatfsCode = f_chmod(_T("1:/f_1.dat"), 0, AM_RDO);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");
    PRINTF("get file \"f_1.dat\" information:\r\n");
    fatfsCode = f_stat(_T("1:/f_1.dat"), &fileInfo);
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    USB_HostMsdFatfsDisplayFileInfo(&fileInfo);
    PRINTF("rename \"f_1.dat\" to \"f_2.dat\"......");
    fatfsCode = f_rename(_T("1:/f_1.dat"), _T("1:/f_2.dat"));
    if (fatfsCode)
    {
        if (fatfsCode == FR_EXIST)
        {
            PRINTF("file exist\r\n");
        }
        else
        {
            PRINTF("error\r\n");
            USB_HostMsdFatfsTestDone();
            return;
        }
    }
    else
    {
        PRINTF("success\r\n");
    }
    PRINTF("delete \"f_2.dat\"......");
    fatfsCode = f_unlink(_T("1:/f_2.dat"));
    if (fatfsCode)
    {
        PRINTF("error\r\n");
        USB_HostMsdFatfsTestDone();
        return;
    }
    PRINTF("success\r\n");

    USB_HostMsdFatfsTestDone();
}


static void USB_HostMsdCommandTest(usb_host_msd_command_instance_t *msdCommandInstance)
{
    usb_status_t status;
    uint32_t blockSize = 512;
    uint32_t address;

    if (msdCommandInstance->commandSemaphore == NULL)
    {
        msdCommandInstance->commandSemaphore = xSemaphoreCreateCounting(0x01U, 0x00U);
    }
    else
    {
        vSemaphoreDelete(msdCommandInstance->commandSemaphore);
        msdCommandInstance->commandSemaphore = xSemaphoreCreateCounting(0x01U, 0x00U);
    }
    if (NULL == msdCommandInstance->commandSemaphore)
    {
        PRINTF("create semaphore fail\r\n");
        return;
    }

    PRINTF("........................test start....................\r\n");

    PRINTF("get max logical units....");
    status = USB_HostMsdGetMaxLun(msdCommandInstance->classHandle, msdCommandInstance->testUfiBuffer,
                                  USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (ufiStatus == kStatus_USB_Success) /* print the command result */
    {
        PRINTF("success, logical units: %d\r\n", msdCommandInstance->testUfiBuffer[0]);
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

    if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    PRINTF("test unit ready....");
    status = USB_HostMsdTestUnitReady(msdCommandInstance->classHandle, 0, USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if ((ufiStatus == kStatus_USB_Success) || (ufiStatus == kStatus_USB_MSDStatusFail)) /* print the command result */
    {
        PRINTF("success, unit status: %s\r\n", ufiStatus == kStatus_USB_MSDStatusFail ? "not ready" : "ready");
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

    if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    PRINTF("request sense....");
    status = USB_HostMsdRequestSense(msdCommandInstance->classHandle, 0, msdCommandInstance->testUfiBuffer,
                                     sizeof(usb_host_ufi_sense_data_t), USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (ufiStatus == kStatus_USB_Success) /* print the command result */
    {
        PRINTF("success\r\n");
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

    if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    PRINTF("inquiry...");
    status = USB_HostMsdInquiry(msdCommandInstance->classHandle, 0, msdCommandInstance->testUfiBuffer,
                                sizeof(usb_host_ufi_inquiry_data_t), USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (ufiStatus == kStatus_USB_Success) /* print the command result */
    {
        PRINTF("success\r\n");
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

    if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    PRINTF("read capacity...");
    status = USB_HostMsdReadCapacity(msdCommandInstance->classHandle, 0, msdCommandInstance->testUfiBuffer,
                                     sizeof(usb_host_ufi_read_capacity_t), USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (ufiStatus == kStatus_USB_Success) /* print the command result */
    {
        address   = (uint32_t) & (msdCommandInstance->testUfiBuffer[0]);
        address   = (uint32_t)((usb_host_ufi_read_capacity_t *)(address))->blockLengthInBytes;
        blockSize = USB_LONG_FROM_BIG_ENDIAN_ADDRESS(((uint8_t *)address));
        address   = (uint32_t) & (msdCommandInstance->testUfiBuffer[0]);
        address   = (uint32_t)((usb_host_ufi_read_capacity_t *)(address))->lastLogicalBlockAddress;
        PRINTF("success, last logical block:%d block length:%d\r\n",
                 USB_LONG_FROM_BIG_ENDIAN_ADDRESS(((uint8_t *)address)), blockSize);
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

    if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (blockSize == 0)
    {
        blockSize = 512;
    }
    PRINTF("read(10)...");
    status = USB_HostMsdRead10(msdCommandInstance->classHandle, 0, 0, msdCommandInstance->testUfiBuffer, blockSize, 1,
                               USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (ufiStatus == kStatus_USB_Success) /* print the command result */
    {
        PRINTF("success\r\n");
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

    if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    PRINTF("write(10)...");
    status = USB_HostMsdWrite10(msdCommandInstance->classHandle, 0, 0, msdCommandInstance->testUfiBuffer, blockSize, 1,
                                USB_HostMsdUfiCallback, msdCommandInstance);
    if (status != kStatus_USB_Success)
    {
        PRINTF("error\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
    {
        msd_command_test_done(msdCommandInstance);
        return;
    }
    if (ufiStatus == kStatus_USB_Success) /* print the command result */
    {
        PRINTF("success\r\n");
    }
    else
    {
        PRINTF("fail\r\n");
        msd_command_test_done(msdCommandInstance);
        return;
    }

#if MSD_THROUGHPUT_TEST_ENABLE
    uint64_t totalTime;
    uint32_t testSize;
    uint32_t blockAddress;
    uint8_t testIndex;

    /* time delay (~100ms) */
    for (testSize = 0; testSize < 400000; ++testSize)
    {
        __NOP();
    }

    CoreDebug->DEMCR |= (1 << CoreDebug_DEMCR_TRCENA_Pos);

    for (testSize = 0; testSize < (THROUGHPUT_BUFFER_SIZE / 4); ++testSize)
    {
        testThroughputBuffer[testSize] = testSize;
    }

    PRINTF("throughput test:\r\n");
    for (testIndex = 0; testIndex < (sizeof(testSizeArray) / 4); ++testIndex)
    {
        totalTime    = 0;
        blockAddress = 0;
        testSize     = testSizeArray[testIndex] * 1024;
        while (testSize)
        {
            if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
            {
                msd_command_test_done(msdCommandInstance);
                return;
            }
            DWT->CYCCNT = 0;
            DWT->CTRL |= (1 << DWT_CTRL_CYCCNTENA_Pos);
            status =
                USB_HostMsdWrite10(msdCommandInstance->classHandle, 0, blockAddress,
                                   (uint8_t *)&testThroughputBuffer[0], THROUGHPUT_BUFFER_SIZE,
                                   (THROUGHPUT_BUFFER_SIZE / blockSize), USB_HostMsdUfiCallback, msdCommandInstance);
            if (status != kStatus_USB_Success)
            {
                PRINTF("    error\r\n");
                msd_command_test_done(msdCommandInstance);
                return;
            }
            if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
            {
                msd_command_test_done(msdCommandInstance);
                return;
            }
            totalTime += DWT->CYCCNT;
            DWT->CTRL &= ~(1U << DWT_CTRL_CYCCNTENA_Pos);
            if (ufiStatus != kStatus_USB_Success)
            {
                PRINTF("fail\r\n");
                msd_command_test_done(msdCommandInstance);
                return;
            }
            testSize -= THROUGHPUT_BUFFER_SIZE;
            blockAddress += (THROUGHPUT_BUFFER_SIZE / blockSize);
        }
        testSize = testSizeArray[testIndex];
        PRINTF("    write %dKB data the speed is %d KB/s\r\n", testSize,
                 (uint32_t)((uint64_t)testSize * (uint64_t)MCU_CORE_CLOCK / (uint64_t)totalTime));

        totalTime    = 0;
        blockAddress = 0;
        testSize     = testSizeArray[testIndex] * 1024;
        while (testSize)
        {
            if (msdCommandInstance->deviceState != kStatus_DEV_Attached)
            {
                msd_command_test_done(msdCommandInstance);
                return;
            }
            DWT->CYCCNT = 0;
            DWT->CTRL |= (1 << DWT_CTRL_CYCCNTENA_Pos);
            status =
                USB_HostMsdRead10(msdCommandInstance->classHandle, 0, blockAddress, (uint8_t *)&testThroughputBuffer[0],
                                  THROUGHPUT_BUFFER_SIZE, (THROUGHPUT_BUFFER_SIZE / blockSize), USB_HostMsdUfiCallback,
                                  msdCommandInstance);
            if (status != kStatus_USB_Success)
            {
                PRINTF("    error\r\n");
                msd_command_test_done(msdCommandInstance);
                return;
            }
            if (pdTRUE != xSemaphoreTake(msdCommandInstance->commandSemaphore, portMAX_DELAY)) /* wait the command */
            {
                msd_command_test_done(msdCommandInstance);
                return;
            }
            totalTime += DWT->CYCCNT;
            DWT->CTRL &= ~(1U << DWT_CTRL_CYCCNTENA_Pos);
            if (ufiStatus != kStatus_USB_Success)
            {
                PRINTF("fail\r\n");
                msd_command_test_done(msdCommandInstance);
                return;
            }
            testSize -= THROUGHPUT_BUFFER_SIZE;
            blockAddress += (THROUGHPUT_BUFFER_SIZE / blockSize);
        }
        testSize = testSizeArray[testIndex];
        PRINTF("    read %dKB data the speed is %d KB/s\r\n", testSize,
                 (uint32_t)((uint64_t)testSize * (uint64_t)MCU_CORE_CLOCK / (uint64_t)totalTime));
    }
#endif /* MSD_THROUGHPUT_TEST_ENABLE */

    msd_command_test_done(msdCommandInstance); /* all test are done */
}

void USB_HostMsdTask(void *arg)
{
    usb_status_t status;
    custom_app_handle_t *custom_ptr = (custom_app_handle_t *)arg;
    usb_host_msd_fatfs_instance_t *msdFatfsInstance = custom_ptr->usb_host_msd_fatfs_inst_ptr;
    app_handle_t *app_handle_ptr_t = custom_ptr->app_handle_ptr;

    if (msdFatfsInstance->deviceState != msdFatfsInstance->prevDeviceState)
    {
    	msdFatfsInstance->prevDeviceState = msdFatfsInstance->deviceState;
        switch (msdFatfsInstance->deviceState)
        {
            case kStatus_DEV_Idle:
                break;

            case kStatus_DEV_Attached: /* device is attached and enumeration is done */
                status = USB_HostMsdInit(msdFatfsInstance->deviceHandle,
                                         &msdFatfsInstance->classHandle); /* msd class initialization */
                g_UsbFatfsClassHandle = msdFatfsInstance->classHandle;
                if (status != kStatus_USB_Success)
                {
                    PRINTF("[USB]  host msd init fail\r\n");
                    return;
                }
                msdFatfsInstance->runState = kUSB_HostMsdRunSetInterface;
                break;

            case kStatus_DEV_Detached: /* device is detached */
            	msdFatfsInstance->deviceState = kStatus_DEV_Idle;
            	msdFatfsInstance->runState    = kUSB_HostMsdRunIdle;
                USB_HostMsdDeinit(msdFatfsInstance->deviceHandle,
                		msdFatfsInstance->classHandle); /* msd class de-initialization */
                msdFatfsInstance->classHandle = NULL;
                app_handle_ptr_t->usbDiskInserted = false;
                PRINTF("[USB]  mass storage device detached\r\n");
                break;

            default:
                break;
        }
    }

    /* run state */
    switch (msdFatfsInstance->runState)
    {
        case kUSB_HostMsdRunIdle:
            break;

        case kUSB_HostMsdRunSetInterface: /* set msd interface */
        	msdFatfsInstance->runState     = kUSB_HostMsdRunIdle;
        	msdFatfsInstance->runWaitState = kUSB_HostMsdRunWaitSetInterface;
            status = USB_HostMsdSetInterface(msdFatfsInstance->classHandle, msdFatfsInstance->interfaceHandle, 0,
                                             USB_HostMsdControlCallback, msdFatfsInstance);
            if (status != kStatus_USB_Success)
            {
                PRINTF("[USB]  set interface fail\r\n");
            }
            break;

        case kUSB_HostMsdRunMassStorageTest:            /* set interface succeed */
        	USB_HostFatMount(msdFatfsInstance, app_handle_ptr_t); /*mount the usb disk*/
        	msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
            break;

        default:
            break;
    }
}

usb_status_t USB_HostMsdEvent(usb_device_handle deviceHandle,
                              usb_host_configuration_handle configurationHandle,
                              uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    usb_host_configuration_t *configuration;
    uint8_t interfaceIndex;
    usb_host_interface_t *interface;
    uint32_t infoValue;
    uint8_t id;

    switch (eventCode)
    {
        case kUSB_HostEventAttach:
            /* judge whether is configurationHandle supported */
            configuration = (usb_host_configuration_t *)configurationHandle;
            for (interfaceIndex = 0; interfaceIndex < configuration->interfaceCount; ++interfaceIndex)
            {
                interface = &configuration->interfaceList[interfaceIndex];
                id        = interface->interfaceDesc->bInterfaceClass;
                if (id != USB_HOST_MSD_CLASS_CODE)
                {
                    continue;
                }
                id = interface->interfaceDesc->bInterfaceSubClass;
                if ((id != USB_HOST_MSD_SUBCLASS_CODE_UFI) && (id != USB_HOST_MSD_SUBCLASS_CODE_SCSI))
                {
                    continue;
                }
                id = interface->interfaceDesc->bInterfaceProtocol;
                if (id != USB_HOST_MSD_PROTOCOL_BULK)
                {
                    continue;
                }
                else
                {
                    if (g_MsdFatfsInstance.deviceState == kStatus_DEV_Idle)
                    {
                        /* the interface is supported by the application */
                        g_MsdFatfsInstance.deviceHandle    = deviceHandle;
                        g_MsdFatfsInstance.interfaceHandle = interface;
                        g_MsdFatfsInstance.configHandle    = configurationHandle;
                        return kStatus_USB_Success;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            status = kStatus_USB_NotSupported;
            break;

        case kUSB_HostEventNotSupported:
            break;

        case kUSB_HostEventEnumerationDone:
            if (g_MsdFatfsInstance.configHandle == configurationHandle)
            {
                if ((g_MsdFatfsInstance.deviceHandle != NULL) && (g_MsdFatfsInstance.interfaceHandle != NULL))
                {
                    /* the device enumeration is done */
                    if (g_MsdFatfsInstance.deviceState == kStatus_DEV_Idle)
                    {
                        g_MsdFatfsInstance.deviceState = kStatus_DEV_Attached;

                        USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDevicePID, &infoValue);
                        PRINTF("[USB]  mass storage device attached:pid=0x%x", infoValue);
                        USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceVID, &infoValue);
                        PRINTF("[USB]  vid=0x%x ", infoValue);
                        USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceAddress, &infoValue);
                        PRINTF("[USB]  address=%d\r\n", infoValue);
                    }
                    else
                    {
                    	PRINTF("[USB]  not idle msd instance\r\n");
                        status = kStatus_USB_Error;
                    }
                }
            }
            break;

        case kUSB_HostEventDetach:
            if (g_MsdFatfsInstance.configHandle == configurationHandle)
            {
                /* the device is detached */
                g_UsbFatfsClassHandle           = NULL;
                g_MsdFatfsInstance.configHandle = NULL;
                if (g_MsdFatfsInstance.deviceState != kStatus_DEV_Idle)
                {
                    g_MsdFatfsInstance.deviceState = kStatus_DEV_Detached;
                }
            }
            break;

        default:
            break;
    }
    return status;
}
