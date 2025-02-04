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

# IO Processors (IOP)
These are some undocumented co-processors that actually exist. The ethernet relies on FPM and MSP, maybe also FAP. 

1. FPM stands for "free pool manager"
2. MSP stands for "Message Processing Processor"
3. FAP stands for "Forwarding Assist Processor", possibly used for NAT acceleration.

Some of the base addresses can be found in [3380_map.h#L193](https://github.com/bcm33xx/CG3100L_V1.0.4_Linux/blob/main/shared/broadcom/include/bcm963xx/3380_map.h#L193):
```c
    //  IOP's Start Here
    ////////////////////

    // Upstream Token Processor
    #define UTP_BLOCK       0xb4000000   // UTP registers
    #define UTP_BLOCK_SMISB 0xb8000000   // UTP registers accessed from the SMISB Mips Bus

    // Downstream Token Processor
    #define DTP_BLOCK       0xb4200000   // DTP registers
    #define DTP_BLOCK_SMISB 0xb8200000   // DTP registers accessed from the SMISB Mips Bus

    // Forwarding Assist Processor
    #define FAP_BLOCK       0xb4400000   // FAP registers
    #define FAP_BLOCK_SMISB 0xb8400000   // FAP registers accessed from the SMISB Mips Bus

    // MPEG Encapsulation Processor
    #define MEP_BLOCK       0xb4600000   // MEP registers
    #define MEP_BLOCK_SMISB 0xb8600000   // MEP registers accessed from the SMISB Mips Bus

    // Message Processing Processor
    #define MSP_BLOCK       0xb5800000   // MSP registers
    #define MSP_BLOCK_SMISB 0xb8800000   // MSP registers accessed from the SMISB Mips Bus

    //  IOP's End Here
    ////////////////////
```

So `printf("Error: LAN RX status = %x, token = %08lx\n", MEMORY[0xB8801240] & 0x7FFF, MEMORY[0xB8801240]);` should be accessing 0x15801240.
