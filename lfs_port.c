#include "lfs_port.h"
//#include "cy_project.h"
//#include "cy_device_headers.h"
#if (CONFIG_FREERTOS_LOCK == 1)
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#endif

#if (CONFIG_FREERTOS_LOCK == 1)
static SemaphoreHandle_t workflash_mutex = NULL;//workflash device mutex
#endif
#if (CONFIG_WORKFLASH_FS == 1)

/**
 * lfs与底层flash读数据接口
 * @param  c
 * @param  block  块编号
 * @param  off    块内偏移地址
 * @param  buffer 用于存储读取到的数据
 * @param  size   要读取的字节数
 * @return
 */
static int lfs_deskio_read(const struct lfs_config *c, 
            lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    volatile uint8_t *pdata = (uint8_t *)buffer;
    volatile uint8_t *taddr = NULL;
    uint32_t intrState;

    if(off > SECTOR_SIZE)
        return LFS_ERR_INVAL;
#if (CONFIG_FREERTOS_LOCK == 1)
    while(xSemaphoreTake(workflash_mutex, CONFIG_LOCK_OSTICK) != pdTRUE);
#endif
    while(size > 0) {
        intrState = Cy_SysLib_EnterCriticalSection();
        taddr = (volatile uint8_t *)(SECTOR_BASE_ADDR + block * SECTOR_SIZE + off);
        *pdata = *taddr; 
        Cy_SysLib_ExitCriticalSection(intrState);
        pdata++;
        off++;
        size--;
    }
#if (CONFIG_FREERTOS_LOCK == 1)
    xSemaphoreGive(workflash_mutex);
#endif

	return LFS_ERR_OK;
}

/**
 * lfs与底层flash写数据接口
 * @param  c
 * @param  block  块编号
 * @param  off    块内偏移地址
 * @param  buffer 待写入的数据
 * @param  size   待写入数据的大小
 * @return
 */
static int lfs_deskio_prog(const struct lfs_config *c, 
            lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t dstAddr;

    if (size % 4) {
      return LFS_ERR_INVAL;
    } 

    dstAddr = SECTOR_BASE_ADDR + block * SECTOR_SIZE + off;
#if (CONFIG_FREERTOS_LOCK == 1)
    while(xSemaphoreTake(workflash_mutex, CONFIG_LOCK_OSTICK) != pdTRUE);
#endif
    for (uint32_t i = 0U; i < size / 4; i++) {
        Cy_FlashWriteWork(dstAddr + i * 4, ((const uint32_t *)buffer) + i, CY_FLASH_DRIVER_BLOCKING);
    }
#if (CONFIG_FREERTOS_LOCK == 1)
    xSemaphoreGive(workflash_mutex);
#endif
    return LFS_ERR_OK;
}

/**
 * lfs与底层flash擦除接口
 * @param  c
 * @param  block 块编号
 * @return
 */
static int lfs_deskio_erase(const struct lfs_config *c, lfs_block_t block)
{

#if (CONFIG_FREERTOS_LOCK == 1)
    while(xSemaphoreTake(workflash_mutex, CONFIG_LOCK_OSTICK) != pdTRUE);
#endif

    Cy_FlashSectorErase(SECTOR_BASE_ADDR + block * SECTOR_SIZE, CY_FLASH_DRIVER_BLOCKING);
#if (CONFIG_FREERTOS_LOCK == 1)
    xSemaphoreGive(workflash_mutex);
#endif

    return LFS_ERR_OK;
}

static int lfs_deskio_sync(const struct lfs_config *c)
{
	return LFS_ERR_OK;
}

const static struct lfs_config lfs_cfg_diskio = {
	// block device operations
	.read  = lfs_deskio_read,
	.prog  = lfs_deskio_prog,
	.erase = lfs_deskio_erase,
	.sync  = lfs_deskio_sync,

    // block device configuration
	.read_size = 16,
	.prog_size = 16,
	.block_size = SECTOR_SIZE,
	.block_count = SECTOR_COUNT,
	.cache_size = 16,
	.lookahead_size = 16,
	.block_cycles = 500,
};

// lfs句柄
static lfs_t lfs_workflash;
lfs_t *workflash_lfs = NULL;

static void lfs_workflash_init(void)
{
    // Initialization 
    Cy_FlashInit(false /*blocking*/);
    FLASHC->unFLASH_CTL.stcField.u1WORK_ERR_SILENT = 1; 
    FLASHC->unFLASH_CTL.stcField.u1MAIN_ERR_SILENT = 1;
        
	// mount the filesystem
	int err = lfs_mount(&lfs_workflash, &lfs_cfg_diskio);

	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (err)
	{
		lfs_format(&lfs_workflash, &lfs_cfg_diskio);
		lfs_mount(&lfs_workflash, &lfs_cfg_diskio);
	}
    workflash_lfs = &lfs_workflash;

}

#endif

#if (CONFIG_RAM_FS == 1)

static uint8_t ramfs[RAM_SECTOR_SIZE * RAM_SECTOR_COUNT];
#define RAM_BASE_ADDR       ((size_t)ramfs)

static int lfs_ramfs_read(const struct lfs_config *c, 
            lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    volatile uint8_t *pdata = (uint8_t *)buffer;
    volatile uint8_t *taddr = NULL;
    
    if(off > RAM_SECTOR_SIZE)
        return LFS_ERR_INVAL;

    while(size > 0) {
        taddr = (volatile uint8_t *)(RAM_BASE_ADDR + block * RAM_SECTOR_SIZE + off);
        *pdata = *taddr;
        pdata++;
        off++;
        size--;
    }

	return LFS_ERR_OK;
}

static int lfs_ramfs_prog(const struct lfs_config *c, 
            lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    size_t dstAddr;

    dstAddr = RAM_BASE_ADDR + block * RAM_SECTOR_SIZE + off;
    
    for (uint32_t i = 0U; i < size; i++)
      memcpy((void *)(dstAddr + i), ((uint8_t *)buffer) + i, 1);

    return LFS_ERR_OK;
}

static int lfs_ramfs_erase(const struct lfs_config *c, lfs_block_t block)
{

    return LFS_ERR_OK;
}

static int lfs_ramfs_sync(const struct lfs_config *c)
{
	return LFS_ERR_OK;
}

const static struct lfs_config lfs_cfg_ramio = {
	// block device operations
	.read  = lfs_ramfs_read,
	.prog  = lfs_ramfs_prog,
	.erase = lfs_ramfs_erase,
	.sync  = lfs_ramfs_sync,

    // block device configuration
	.read_size = 16,
	.prog_size = 16,
	.block_size = RAM_SECTOR_SIZE,
	.block_count = RAM_SECTOR_COUNT,
	.cache_size = 16,
	.lookahead_size = 16,
	.block_cycles = 500,
};
// lfs句柄
static lfs_t lfs_ram;
lfs_t *ram_lfs = NULL;

static void lfs_ram_init(void)
{   
	// mount the filesystem
	int err = lfs_mount(&lfs_ram, &lfs_cfg_ramio);

	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (err)
	{
		lfs_format(&lfs_ram, &lfs_cfg_ramio);
		lfs_mount(&lfs_ram, &lfs_cfg_ramio);
	}
    ram_lfs = &lfs_ram;
}

#endif


#ifdef LFS_TEST
int32_t lfs_testcase(void)
{
    uint32_t count = 11, readback = 0;
    int res = 0;
    
#if (CONFIG_RAM_FS == 1)
    lfs_file_t ram_file;
	// read current count
	lfs_file_open(ram_lfs, &ram_file, "count", LFS_O_RDWR | LFS_O_CREAT);
	lfs_file_write(ram_lfs, &ram_file, &count, sizeof(count));
	// remember the storage is not updated until the file is closed successfully
	lfs_file_close(ram_lfs, &ram_file);
        
        //re-open the file to read back
        lfs_file_open(ram_lfs, &ram_file, "count", LFS_O_RDONLY);
        lfs_file_read(ram_lfs, &ram_file, &readback, sizeof(readback));
        lfs_file_close(ram_lfs, &ram_file);
        res += !(count == readback);
#endif

#if (CONFIG_WORKFLASH_FS == 1)
	// read current count
	uint32_t boot_count = 0, boot_back = 0;
    lfs_file_t flash_file;
	lfs_file_open(&lfs_workflash, &flash_file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
	lfs_file_read(&lfs_workflash, &flash_file, &boot_count, sizeof(boot_count));

	// update boot count
	boot_count += 1;
	lfs_file_rewind(&lfs_workflash, &flash_file);
	lfs_file_write(&lfs_workflash, &flash_file, &boot_count, sizeof(boot_count));

	// remember the storage is not updated until the file is closed successfully
	lfs_file_close(&lfs_workflash, &flash_file);
        
        lfs_file_open(&lfs_workflash, &flash_file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
	lfs_file_read(&lfs_workflash, &flash_file, &boot_back, sizeof(boot_back));
        lfs_file_close(&lfs_workflash, &flash_file);
        
        res += !(boot_count == boot_back);
#endif
        return res;
}
#endif

void lfs_init(void)
{

#if (CONFIG_FREERTOS_LOCK == 1)
    if (NULL == workflash_mutex) {
      workflash_mutex = xSemaphoreCreateMutex();
      xSemaphoreGive(workflash_mutex);
    }
#endif

#if (CONFIG_RAM_FS == 1)
    lfs_ram_init();
#endif
#if (CONFIG_WORKFLASH_FS == 1)
    lfs_workflash_init();
#endif

}

void lfs_deinit(void)
{
#if (CONFIG_RAM_FS == 1)
    lfs_unmount(&lfs_ram);
#endif
#if (CONFIG_WORKFLASH_FS == 1)
	lfs_unmount(&lfs_workflash);
#endif
#if (CONFIG_FREERTOS_LOCK == 1)
    if (NULL != workflash_mutex) {
      vSemaphoreDelete(workflash_mutex);
    }
#endif
}