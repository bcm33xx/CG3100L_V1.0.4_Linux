This repo contains the Linux source code for Netgear CG3100L.

Most facts here also apply to CG3100.

### [`shared/broadcom/include/bcm963xx/bcm3380`](shared/broadcom/include/bcm963xx/bcm3380)
There seems to be a complete register definition for the BCM3380 SoC. Interestingly, almost none of these definitions is referenced in the code base.

# Kernel configs
1. `CONFIG_BCM93380` is defined.
2. `CONFIG_BRCM_IKOS` is not defined.

# Memory layout
[3380_cpu.h](shared/opensource/include/bcm963xx/3380_cpu.h) and [bcm_hwdefs.h](shared/opensource/include/bcm963xx/bcm_hwdefs.h) provide important information about the flash layout:

```c
#define PHYS_DRAM_BASE          0x00000000      /* Dynamic RAM Base */
#define PHYS_FLASH_BASE         0x1FC00000      /* Flash Memory     */

/*****************************************************************************/
/* Note that the addresses above are physical addresses and that programs    */
/* have to use converted addresses defined below:                            */
/*****************************************************************************/
#define DRAM_BASE           (0x80000000 | PHYS_DRAM_BASE)   /* cached DRAM */
#define DRAM_BASE_NOCACHE   (0xA0000000 | PHYS_DRAM_BASE)   /* uncached DRAM */

/* Binary images are always built for a standard MIPS boot address */
#define IMAGE_BASE          (0xA0000000 | PHYS_FLASH_BASE)

// Rikka0w0: This was set to 0 on purpose???
#define FLASH_BASE          0
//#define FLASH_BASE          0xB8000000


#define BOOT_OFFSET         (FLASH_BASE - IMAGE_BASE)
```

We can conclude that, upon power on, the SoC runs instructions from the beginning of the flash. The base address for bootloader is 0xBFC00000.

# Co-processors
Defined in [3380_cpu.h](shared/opensource/include/bcm963xx/3380_cpu.h). The Linux kernel [mipsregs.h](https://github.com/torvalds/linux/blob/master/arch/mips/include/asm/mipsregs.h) provides some macro to simplify the access to those co-processors registers.

| Offset | Description                                 | Linux header macro     |
|--------|---------------------------------------------|----------------------- |
| 0      | Coprocessor 0 Broadcom Config Register Bits | read_c0_brcm_config    |
| 1      | Coprocessor 0 CMT Interrupt Register        | read_c0_brcm_cmt_intr  |
| 2      | Coprocessor 0 CMT Control Register          | read_c0_brcm_cmt_ctrl  |
| 3      | Coprocessor 0 CMT Local Register            | read_c0_brcm_cmt_local |
