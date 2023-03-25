#ifndef LFS_PORT_H
#define LFS_PORT_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "lfs_util.h"
#include "lfs.h"

#define CONFIG_RAM_FS           1
#define CONFIG_WORKFLASH_FS     0
#define LFS_TEST                1
#define CONFIG_FREERTOS_LOCK    0
#define CONFIG_LOCK_OSTICK      10


void lfs_init(void);
void lfs_deinit(void);

#if (CONFIG_RAM_FS == 1)
#define RAM_SECTOR_SIZE            128U//CY_WORK_LES_SIZE_IN_BYTE
#define RAM_SECTOR_COUNT           128U

extern lfs_t *ram_lfs;

#endif

#if (CONFIG_WORKFLASH_FS == 1)

/* on the single bank mode 
 *  work flash size: 128K
 *  large sectors 2KB x 48(sector)      base address: 0x1400 0000
 *  small sectors 128B x 256(sector)    base address: 0x1401 8000
 * */
#define SECTOR_BASE_ADDR       CY_WFLASH_LG_SBM_TOP 
#define SECTOR_SIZE            CY_WORK_LES_SIZE_IN_BYTE
#define SECTOR_COUNT           48U

extern lfs_t *workflash_lfs;

#endif


#if (LFS_TEST == 1)
int32_t lfs_testcase(void);
#endif


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* #ifndef LFS_PORT_H */