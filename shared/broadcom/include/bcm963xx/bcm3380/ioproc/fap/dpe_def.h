#ifndef DPE_DEF_H
#define DPE_DEF_H


//#define MAP_IO_REG_TO_FPGA_ADDR(x) (x)
//enum dma_op_t  { INSERT_CMD, REPLACE_CMD, DELETE_CMD, END_CMD };
enum dpe_cmd_t { DPE_DMA_OUT, DPE_DMA_IN, DPE_FFE_EXE, DPE_DMA_IN_FFE_EXE };

//#define HOST_IOPROC_BASE  0
//#define MAP_IO_REG_TO_HOST_ADDR(x)  ((((uint32)&x) & 0x03ffffff) | HOST_IOPROC_BASE)


#define DPE_REG_START                   0x0000
#define DPE_REG_END                     0x0064
#define MPEG_REG_START                  0x0200
#define MPEG_REG_END                    0x031c
#define DPE_NUG_MEM_START               0x2000        // 32x256       => 1k byte
#define DPE_NUG_MEM_END                 0x23FC
#define DPE_PKT_MEM_START               0x3000        // 32x256 2x    => 2k byte
#define DPE_PKT_MEM_END                 0x37FC
#define DPE_FFE_MEM_START               0x4000        // 48 (64) x 2048 => 16kbyte
#define DPE_FFE_MEM_END                 0x7FFC

#define HOST_DPE_REG_BASE       (HOST_DPE_BASE + DPE_REG_START) 
#define HOST_MPEG_REG_BASE      (HOST_DPE_BASE + MPEG_REG_START) 
#define HOST_DPE_NUG_MEM_BASE   (HOST_DPE_BASE + DPE_NUG_MEM_START)
#define HOST_DPE_PKT_MEM_BASE   (HOST_DPE_BASE + DPE_PKT_MEM_START)
#define HOST_DPE_FFE_MEM_BASE   (HOST_DPE_BASE + DPE_FFE_MEM_START)

#define MIPS_DPE_REG_BASE       (MIPS_DPE_BASE + DPE_REG_START) 
#define MIPS_MPEG_REG_BASE      (MIPS_DPE_BASE + MPEG_REG_START) 
#define MIPS_DPE_NUG_MEM_BASE   (MIPS_DPE_BASE + DPE_NUG_MEM_START)
#define MIPS_DPE_PKT_MEM_BASE   (MIPS_DPE_BASE + DPE_PKT_MEM_START)
#define MIPS_DPE_FFE_MEM_BASE   (MIPS_DPE_BASE + DPE_FFE_MEM_START)

#define DpeReg         ((volatile DPERegs_S *)MIPS_DPE_REG_BASE)
#define MpegReg        ((volatile MPEGRegs_S *)MIPS_MPEG_REG_BASE)

#define MPEG_FRM_LEN        188
#define MPEG_STREAM_TOTAL   8
#define MPEG_FLOW_TOTAL     8
#define MPEG_MAX_FLOW_PKT   7
#define MPEG_BUFF_NUM       (MPEG_MAX_FLOW_PKT*2)
#define MPEG_FLOW_TIMEOUT   400
#define MPEG_SYNC_BYTE      0x47
#define MPEG_TX_SYNC_BIT    0x200
#define MPEG_TX_VALID_BIT   0x100

#define WORD_LENGTH         4
#define DPE_CMD_FIFO_DEPTH  8
#define DPE_PKT_MEM_SLOT_SIZE 256
#define DPE_NUG_MEM_SLOT_SIZE 64
//#define SHARED_MEM_PKT_BASE (SHARED_MEM_BASE + 0x1000)
#define SHARED_MEM_PKT_BASE FAP_PSM_BASE

#define SHAREMEM_ST_ADDR (((unsigned int)SharedMemPtr) + 0x1000)
#define IOP_CMD_ST_ADDR (((SHAREMEM_ST_ADDR) & 0xfffffffc) - 0x200)


typedef struct dpe_fifo_s
{
  uint rd_ptr;
  uint wr_ptr;
  uint depth;
  uint ovr;
  uint udr;
  uint full;
  uint not_empty;
} dpe_fifo_s;

typedef struct dpe_mpeg_stream_cfg_s
{
  uint active;
  uint pkt_slot_num;
  uint nug_slot_num;
  uint pkt_len;
  uint ffe_pc;
} dpe_mpeg_stream_cfg_s;

typedef struct dpe_mpeg_flow_cfg_s
{
  uint min_pkt;
  uint flow_en;
} dpe_mpeg_flow_cfg_s;

typedef struct dpe_mpeg_queue_cfg_s
{
  uint base_ptr;
  uint last_ptr;
  uint buff_num;
} dpe_mpeg_queue_cfg_s;

typedef struct dpe_mpeg_frm_s
{
  byte ts_pkt[MPEG_FRM_LEN];
  uint16 pid;
  uint   pcr;
  uint   ts_num;
} dpe_mpeg_frm_s; 

typedef struct mpeg_sts_s
{
  uint flow_num;
  uint buf_full;
  uint ts_num;
  uint pcr;
  uint pkt_len;
  uint timestamp;
  uint head_ptr;
  uint tail_ptr;
  uint head_wrap;
  uint tail_wrap;
} mpeg_sts_s;

typedef struct mpeg_sts_dqm_msg_s
{
  uint flow_num;
  uint buf_full;
  uint ts_num;
  uint pcr;
  uint pkt_len;
  uint timestamp;
  uint mainmem_addr;
  uint dma_length;
  uint length_n;
} mpeg_sts_dqm_msg_s;

typedef struct dpe_cmd_s
{
  uint idx;
  uint cmd;
  uint pkt_type;
  uint pkt_mem_slot;
  uint nug_mem_slot;
  uint buff_len;
  uint ffe_pc;
  uint mpeg_cmd;
  uint dma_addr;
  uint ffe_return_val0;
  uint ffe_return_val1;
} dpe_cmd_s;

typedef struct iop_dpe_cmd_s
{
  uint idx;
  uint iop_src_addr;
  uint iop_length;
  uint iop_cmnd_addr;
  uint pkt_type;
  uint dpe_cmd;
  uint pkt_mem_slot;
  uint nug_mem_slot;
  uint ffe_pc;
  uint ffe_return_val0;
  uint ffe_return_val1;
} iop_dpe_cmd_s;

typedef struct dpe_sts_s
{
  uint fifo_depth;
  uint pkt_mem_slot;
  uint nug_mem_slot;
  uint ffe_return_val0;
  uint ffe_return_val1;
} dpe_sts_s;

typedef struct iop_dma_rslt_s
{
  uint src_addr;
  uint dest_addr;
  uint dma_len;
  uint32 exp_data[128];
} iop_dma_rslt_s;

typedef struct iop_dma_cmd_s
{
  uint dma_ch;
  uint src_addr;
  uint dst_addr;
  uint cmd_addr;
  uint dma_len;
  uint cont;
  uint is_token;
  uint dst_addr_mode;
  uint exec_cmd_list;
  uint wait;
  uint len_n_val;
} iop_dma_cmd_s;

typedef struct iop_dma_sts_s
{
  uint dma_ch;
  uint src_addr;
  uint dst_addr;
  uint16 hcs0;
  uint16 hcs1;
  uint dma_len;
  uint cont;
  uint rslt_len_stat;
} iop_dma_sts_s;


typedef struct DPECMDFIFORegs_S
{
  uint32        cmd_word0;   // 00
                #define DPE_CMD_MASK                     0xC0000000
                #define DPE_CMD_SHIFT                    30
                #define DPE_PKT_MEM_SLOT_MASK            0x38000000
                #define DPE_PKT_MEM_SLOT_SHIFT           27
                #define DPE_NUG_MEM_SLOT_MASK            0x07000000
                #define DPE_NUG_MEM_SLOT_SHIFT           24
                #define DPE_BUF_LEN_MASK                 0x00FF8000
                #define DPE_BUF_LEN_SHIFT                15
                #define DPE_FFE_PC_MASK                  0x00007FFF
                #define DPE_FFE_PC_SHIFT                 0
  uint32        cmd_word1;   // 04
                #define DPE_MPEG_CMD_MASK                0x80000000
                #define DPE_MPEG_CMD_SHIFT               31
                #define DPE_DMA_ADDR_MASK                0x0000FFFF
                #define DPE_DMA_ADDR_SHIFT               0
} DPECMDFIFORegs_S;

typedef struct DPESTSFIFORegs_S
{
  uint32        sts_word0;   // 08
                #define DPE_STS_FIFO_NOT_EMPTY_MASK      0x80000000
                #define DPE_STS_FIFO_NOT_EMPTY_SHIFT     31
                #define DPE_STS_FIFO_DEPTH_MASK          0x7C000000
                #define DPE_STS_FIFO_DEPTH_SHIFT         26
                #define DPE_STS_PKT_MEM_SLOT_MASK        0x03800000
                #define DPE_STS_PKT_MEM_SLOT_SHIFT       23
                #define DPE_STS_NUG_MEM_SLOT_MASK        0x00700000
                #define DPE_STS_NUG_MEM_SLOT_SHIFT       20
                #define FFE_RETURN_VAL0_MASK             0x0000FFFF
                #define FFE_RETURN_VAL0_SHIFT            0
                #define DPE_MPEG_BUF_FULL_MASK           0x00080000
                #define DPE_MPEG_BUF_FULL_SHIFT          19
                #define DPE_MPEG_TS_NUM_MASK             0x00070000
                #define DPE_MPEG_TS_NUM_SHIFT            16
                #define DPE_MPEG_STS_PKTLEN_MASK         0x0fff
                #define DPE_MPEG_STS_PKTLEN_SHIFT        0
                #define DPE_MPEG_STS_PCR_MASK            0x8000
                #define DPE_MPEG_STS_PCR_SHIFT           15
                #define DPE_MPEG_STS_FLOWNUM_MASK        0x7000
                #define DPE_MPEG_STS_FLOWNUM_SHIFT       12
  uint32        sts_word1;   // 0c
                #define FFE_RETURN_VAL1_MASK             0xFFFFFFFF
                #define FFE_RETURN_VAL1_SHIFT            0
} DPESTSFIFORegs_S;

typedef struct DPECFGRegs_S
{
  uint32        config;   // 10
                #define DPE_FIFO_RESET_MASK              0x00000008
                #define DPE_FIFO_RESET_SHIFT             3
                #define MPEG_MODE_MASK                   0x00000004
                #define MPEG_MODE_SHIFT                  2
                #define ENABLE_FFE_CMD_MASK              0x00000002
                #define ENABLE_FFE_CMD_SHIFT             1
                #define ENABLE_FFE_MASK                  0x00000001
                #define ENABLE_FFE_SHIFT                 0
} DPECFGRegs_S;

typedef struct DPEFIFOSTATERegs_S
{
  uint32        cmd_fifo1;   // 14
  uint32        cmd_fifo2;   // 18
  uint32        sts_fifo1;   // 1c
  uint32        sts_fifo2;   // 20
  uint32        dma_fifo;    // 24
  uint32        ctrl_fifo;   // 28
                #define DPE_FIFO_FULL_MASK               0x80000000
                #define DPE_FIFO_FULL_SHIFT              31
                #define DPE_FIFO_NOT_EMPTY_MASK          0x40000000
                #define DPE_FIFO_NOT_EMPTY_SHIFT         30
                #define DPE_FIFO_UDR_MASK                0x20000000
                #define DPE_FIFO_UDR_SHIFT               29
                #define DPE_FIFO_OVR_MASK                0x10000000
                #define DPE_FIFO_OVR_SHIFT               28
                #define DPE_FIFO_WPTR_MASK               0x001F0000
                #define DPE_FIFO_WPTR_SHIFT              16
                #define DPE_FIFO_RPTR_MASK               0x00001F00
                #define DPE_FIFO_RPTR_SHIFT              8
                #define DPE_FIFO_DEPTH_MASK              0x0000001F
                #define DPE_FIFO_DEPTH_SHIFT             0
} DPEFIFOSTATERegs_S;

typedef struct DPEFIFOSTSRegs_S
{
  uint32        status;   // 2c
                #define DPE_CMD_FIFO1_FULL_MASK          0x80000000
                #define DPE_CMD_FIFO1_FULL_SHIFT         31
                #define DPE_CMD_FIFO1_NOT_EMPTY_MASK     0x40000000
                #define DPE_CMD_FIFO1_NOT_EMPTY_SHIFT    30
                #define DPE_CMD_FIFO1_UDR_MASK           0x20000000
                #define DPE_CMD_FIFO1_UDR_SHIFT          29
                #define DPE_CMD_FIFO1_OVR_MASK           0x10000000
                #define DPE_CMD_FIFO1_OVR_SHIFT          28
                #define DPE_CMD_FIFO2_FULL_MASK          0x08000000
                #define DPE_CMD_FIFO2_FULL_SHIFT         27
                #define DPE_CMD_FIFO2_NOT_EMPTY_MASK     0x04000000
                #define DPE_CMD_FIFO2_NOT_EMPTY_SHIFT    26
                #define DPE_CMD_FIFO2_UDR_MASK           0x02000000
                #define DPE_CMD_FIFO2_UDR_SHIFT          25
                #define DPE_CMD_FIFO2_OVR_MASK           0x01000000
                #define DPE_CMD_FIFO2_OVR_SHIFT          24
                #define DPE_STS_FIFO1_FULL_MASK          0x00800000
                #define DPE_STS_FIFO1_FULL_SHIFT         23
                #define DPE_STS_FIFO1_NOT_EMPTY_MASK     0x00400000
                #define DPE_STS_FIFO1_NOT_EMPTY_SHIFT    22
                #define DPE_STS_FIFO1_UDR_MASK           0x00200000
                #define DPE_STS_FIFO1_UDR_SHIFT          21
                #define DPE_STS_FIFO1_OVR_MASK           0x00100000
                #define DPE_STS_FIFO1_OVR_SHIFT          20
                #define DPE_STS_FIFO2_FULL_MASK          0x00080000
                #define DPE_STS_FIFO2_FULL_SHIFT         19
                #define DPE_STS_FIFO2_NOT_EMPTY_MASK     0x00040000
                #define DPE_STS_FIFO2_NOT_EMPTY_SHIFT    18
                #define DPE_STS_FIFO2_UDR_MASK           0x00020000
                #define DPE_STS_FIFO2_UDR_SHIFT          17
                #define DPE_STS_FIFO2_OVR_MASK           0x00010000
                #define DPE_STS_FIFO2_OVR_SHIFT          16
                #define DPE_DMA_FIFO_FULL_MASK           0x00008000
                #define DPE_DMA_FIFO_FULL_SHIFT          15
                #define DPE_DMA_FIFO_NOT_EMPTY_MASK      0x00004000
                #define DPE_DMA_FIFO_NOT_EMPTY_SHIFT     14
                #define DPE_DMA_FIFO_UDR_MASK            0x00002000
                #define DPE_DMA_FIFO_UDR_SHIFT           13
                #define DPE_DMA_FIFO_OVR_MASK            0x00001000
                #define DPE_DMA_FIFO_OVR_SHIFT           12
                #define DPE_CTRL_FIFO_FULL_MASK          0x00000800
                #define DPE_CTRL_FIFO_FULL_SHIFT         11
                #define DPE_CTRL_FIFO_NOT_EMPTY_MASK     0x00000400
                #define DPE_CTRL_FIFO_NOT_EMPTY_SHIFT    10
                #define DPE_CTRL_FIFO_UDR_MASK           0x00000200
                #define DPE_CTRL_FIFO_UDR_SHIFT          9
                #define DPE_CTRL_FIFO_OVR_MASK           0x00000100
                #define DPE_CTRL_FIFO_OVR_SHIFT          8
} DPEFIFOSTSRegs_S;

typedef struct DPEFIFOINTMASKRegs_S
{
  uint32        mask;   // 30
} DPEFIFOINTMASKRegs_S;

typedef struct DPEMPEGFIFOSTATERegs_S
{
  uint32        mpeg_fifo;   // 34
} DPEMPEGFIFOSTATERegs_S; 

typedef struct DPEMPEGSTSRegs_S
{
  uint32        status;   // 38
                #define DPE_MPEG_FLOW_BUF_FULL_MASK     0x00ff0000
                #define DPE_MPEG_FLOW_BUF_FULL_SHIFT    16
                #define DPE_MPEG_STREAM_PKT_OVR_MASK    0x0000ff00
                #define DPE_MPEG_STREAM_PKT_OVR_SHIFT   8
                #define DPE_MPEG_STREAM_WRD_OVR_MASK    0x000000ff
                #define DPE_MPEG_STREAM_WRD_OVR_SHIFT   0
} DPEMPEGSTSRegs_S;

typedef struct DPEMPEGINTMASKRegs_S
{
  uint32        mask;   // 3c
} DPEMPEGINTMASKRegs_S;

typedef struct DPEFFESTSRegs_S
{
  uint32        status;   // 40
                #define DPE_FFE_UNSUPP_CRC_TYPE_MASK   0x00000008
                #define DPE_FFE_UNSUPP_CRC_TYPE_SHIFT  3
                #define DPE_FFE_UNSUPP_CRC_CODE_MASK   0x00000004
                #define DPE_FFE_UNSUPP_CRC_CODE_SHIFT  2
                #define DPE_FFE_HALT_ENCNR_MASK        0x00000002
                #define DPE_FFE_HALT_ENCNR_SHIFT       1
                #define DPE_FFE_INVALID_OPCODE_MASK    0x00000001
                #define DPE_FFE_INVALID_OPCODE_SHIFT   0
} DPEFFESTSRegs_S;

typedef struct DPEFFEINTMASKRegs_S
{
  uint32        mask;   // 44
} DPEFFEINTMASKRegs_S;

typedef struct DPEINTRegs_S
{
  uint32        interrupt;   // 48
                #define DPE_FFE_INT_MASK     0x00000004
                #define DPE_FFE_INT_SHIFT    2
                #define DPE_MPEG_INT_MASK    0x00000002
                #define DPE_MPEG_INT_SHIFT   1
                #define DPE_BASIC_INT_MASK   0x00000001
                #define DPE_BASIC_INT_SHIFT  0
} DPEINTRegs_S;

typedef struct DPEDBGCODERegs_S
{
  uint32        debug_state_code;   // 4c
                #define DPE_ZERO_LEN_ERR_MASK     0x10000000
                #define DPE_ZERO_LEN_ERR_SHIFT    31
                #define DPE_MPEG_PKT_ARB_ST_MASK  0x00030000
                #define DPE_MPEG_PKT_ARB_ST_SHIFT 16
                #define DPE_DMA_ENGINE_ST_MASK    0x00007000
                #define DPE_DMA_ENGINE_ST_SHIFT   12
                #define DPE_DMA_CTRL_ST_MASK      0x00000700
                #define DPE_DMA_CTRL_ST_SHIFT     8
                #define DPE_FIFO_HELPER_FETCH_ST_MASK  0x00000030
                #define DPE_FIFO_HELPER_FETCH_ST_SHIFT 4
                #define DPE_FIFO_HELPER_FWD_ST_MASK    0x00000003
                #define DPE_FIFO_HELPER_FWD_ST_SHIFT   0
} DPEDBGCODERegs_S;

typedef struct DPEFFEDEBUGRegs_S
{
  uint32        ffe_debug1;   // 50
  uint32        ffe_debug2;   // 54
                #define FFE_NP_MASK     0x3fc00000
                #define FFE_NP_SHIFT    22
                #define FFE_DP_MASK     0x003ff800
                #define FFE_DP_SHIFT    11
                #define FFE_IP_MASK     0x000007ff
                #define FFE_IP_SHIFT    0
                #define FFE_ST_MASK     0x00000100
                #define FFE_ST_SHIFT    8
                #define FFE_SP_MASK     0x000000ff
                #define FFE_SP_SHIFT    0
} DPEFFEDEBUGRegs_S;

typedef struct DPEFFECTRLRegs_S
{
  uint32        ffe_ctrl1;   // 58
  uint32        ffe_ctrl2;   // 5c
                #define FFE_INT_DISABLE_MASK     0x00000001
                #define FFE_INT_DISABLE_SHIFT    0
                #define FFE_FORCE_IMEM_CS_MASK   0x00000004
                #define FFE_FORCE_IMEM_CS_SHIFT  2
                #define FFE_FORCE_DMEM_CS_MASK   0x00000002
                #define FFE_FORCE_DMEM_CS_SHIFT  1
                #define FFE_FORCE_MMEM_CS_MASK   0x00000001
                #define FFE_FORCE_MMEM_CS_SHIFT  0
} DPEFFECTRLRegs_S;

typedef struct DPEDIAGOUTRegs_S
{
  uint32        diag_out;   // 60
                #define DPE_DIAG_OUT_HIGH_MASK     0xffff0000
                #define DPE_DIAG_OUT_HIGH_SHIFT    16
                #define DPE_DIAG_OUT_LOW_MASK      0x0000ffff
                #define DPE_DIAG_OUT_LOW_SHIFT     0
} DPEDIAGOUTRegs_S;

typedef struct MPEGSTREAMCFGRegs_S
{
  uint32        config;
                #define MPEG_ACTIVE_MASK           0x80000000
                #define MPEG_ACTIVE_SHIFT          31
                #define MPEG_PKT_SLOT_NUM_MASK     0x30000000
                #define MPEG_PKT_SLOT_NUM_SHIFT    28
                #define MPEG_NUG_SLOT_NUM_MASK     0x06000000
                #define MPEG_NUG_SLOT_NUM_SHIFT    25
                #define MPEG_NUG_DONT_TOGG_MASK    0x01000000
                #define MPEG_NUG_DONT_TOGG_SHIFT   24
                #define MPEG_PKT_LEN_MASK          0x00ff0000
                #define MPEG_PKT_LEN_SHIFT         16
                #define MPEG_FFE_PC_MASK           0x0000ffff
                #define MPEG_FFE_PC_SHIFT          0
} MPEGSTREAMCFGRegs_S;

typedef struct MPEGSTREAMSTSRegs_S
{
  uint32        status;
                #define MPEG_DMA_PEND_MASK        0x00008000
                #define MPEG_DMA_PEND_SHIFT       15
                #define MPEG_FFE_PEND_MASK        0x00004000
                #define MPEG_FFE_PEND_SHIFT       14
                #define MPEG_PKT_RDY_MASK         0x00002000
                #define MPEG_PKT_RDY_SHIFT        13
                #define MPEG_WRD_RDY_MASK         0x00001000
                #define MPEG_WRD_RDY_SHIFT        12
                #define MPEG_PKT_OVR_MASK         0x00000400
                #define MPEG_PKT_OVR_SHIFT        10
                #define MPEG_WRD_OVR_MASK         0x40000200
                #define MPEG_WRD_OVR_SHIFT        9
                #define MPEG_ODD_EVEN_SLOT_MASK   0x00000100
                #define MPEG_ODD_EVEN_SLOT_SHIFT  8
                #define MPEG_BYTE_CNT_MASK        0x000000ff
                #define MPEG_BYTE_CNT_SHIFT       0
} MPEGSTREAMSTSRegs_S;

typedef struct MPEGFLOWCFGRegs_S
{
  uint32        config;
                #define MPEG_FLOW_ENABLE_MASK     0x80000000
                #define MPEG_FLOW_ENABLE_SHIFT    31
                #define MPEG_MIN_PKT_MASK         0x0000000f
                #define MPEG_MIN_PKT_SHIFT        0
} MPEGFLOWCFGRegs_S;

typedef struct MPEGFLOWSTSRegs_S
{
  uint32        status;
                #define MPEG_BUF_FULL_MASK        0x80000000
                #define MPEG_BUF_FULL_SHIFT       31
                #define MPEG_DROP_CNT_MASK        0x0000ff00
                #define MPEG_DROP_CNT_SHIFT       8
                #define MPEG_PKT_CNT_MASK         0x0000000f
                #define MPEG_PKT_CNT_SHIFT        0
} MPEGFLOWSTSRegs_S;

typedef struct MPEGQBASERegs_S
{
  uint32        pointer;
                #define MPEG_Q_PTR_MASK           0x0000fffc
                #define MPEG_Q_PTR_SHIFT          2
} MPEGQBASERegs_S;

typedef struct MPEGQLASTRegs_S
{
  uint32        pointer;
} MPEGQLASTRegs_S;

typedef struct MPEGQTAILRegs_S
{
  uint32        pointer;
                #define MPEG_Q_WRAP_BIT_MASK      0x80000000
                #define MPEG_Q_WRAP_BIT_SHIFT     31
} MPEGQTAILRegs_S;

typedef struct MPEGQHEADRegs_S
{
  uint32        pointer;
} MPEGQHEADRegs_S;



typedef struct DPERegs_S
{
  DPECMDFIFORegs_S       dpe_cmd_fifo;
  DPESTSFIFORegs_S       dpe_sts_fifo;
  DPECFGRegs_S           dpe_cfg;
  DPEFIFOSTATERegs_S     dpe_fifo_state;
  DPEFIFOSTSRegs_S       dpe_fifo_status;
  DPEFIFOINTMASKRegs_S   dpe_fifo_int_mask;
  DPEMPEGFIFOSTATERegs_S dpe_mpeg_fifo_state;
  DPEMPEGSTSRegs_S       dpe_mpeg_status;
  DPEMPEGINTMASKRegs_S   dpe_mpeg_int_mask;
  DPEFFESTSRegs_S        dpe_ffe_status;
  DPEFFEINTMASKRegs_S    dpe_ffe_int_mask;
  DPEINTRegs_S           dpe_int;
  DPEDBGCODERegs_S       dpe_debug_decode;
  DPEFFEDEBUGRegs_S      dpe_ffe_debug;
  DPEFFECTRLRegs_S       dpe_ffe_ctrl;
  DPEDIAGOUTRegs_S       dpe_diag_out;
} DPERegs_S;

typedef struct MPEGRegs_S
{
  MPEGSTREAMCFGRegs_S   stream_config[8];       // 0x00
  MPEGSTREAMSTSRegs_S   stream_status[8];       // 0x20
  MPEGFLOWCFGRegs_S     flow_config[8];         // 0x40
  MPEGFLOWSTSRegs_S     flow_status[8];         // 0x60
  MPEGQBASERegs_S       queue_base[8];          // 0x80
  MPEGQLASTRegs_S       queue_last[8];          // 0xa0
  MPEGQTAILRegs_S       queue_tail[8];          // 0xc0
  MPEGQHEADRegs_S       queue_head[8];          // 0xe0
} MPEGRegs_S;

#endif //#ifndef DPE_DEF_H
