// RISC-V Constants and Definitions
// ALU Operations
`define ALU_NONE              4'd0
`define ALU_SHIFTL            4'd1
`define ALU_SHIFTR            4'd2
`define ALU_SHIFTR_ARITH      4'd3
`define ALU_ADD               4'd4
`define ALU_SUB               4'd6
`define ALU_AND               4'd7
`define ALU_OR                4'd8
`define ALU_XOR               4'd9
`define ALU_LESS_THAN         4'd10
`define ALU_LESS_THAN_SIGNED  4'd11

// Instruction Opcodes & Masks
// Integer Register-Immediate Instructions
`define INST_ANDI       32'h00007013
`define INST_ANDI_MASK  32'h0000707f
`define INST_ADDI       32'h00000013
`define INST_ADDI_MASK  32'h0000707f
`define INST_SLTI       32'h00002013
`define INST_SLTI_MASK  32'h0000707f
`define INST_SLTIU      32'h00003013
`define INST_SLTIU_MASK 32'h0000707f
`define INST_ORI        32'h00006013
`define INST_ORI_MASK   32'h0000707f
`define INST_XORI       32'h00004013
`define INST_XORI_MASK  32'h0000707f
`define INST_SLLI       32'h00001013
`define INST_SLLI_MASK  32'hfc00707f
`define INST_SRLI       32'h00005013
`define INST_SRLI_MASK  32'hfc00707f
`define INST_SRAI       32'h40005013
`define INST_SRAI_MASK  32'hfc00707f
`define INST_LUI        32'h00000037
`define INST_LUI_MASK   32'h0000007f
`define INST_AUIPC      32'h00000017
`define INST_AUIPC_MASK 32'h0000007f

// Integer Register-Register Instructions
`define INST_ADD        32'h00000033
`define INST_ADD_MASK   32'hfe00707f
`define INST_SUB        32'h40000033
`define INST_SUB_MASK   32'hfe00707f
`define INST_SLT        32'h00002033
`define INST_SLT_MASK   32'hfe00707f
`define INST_SLTU       32'h00003033
`define INST_SLTU_MASK  32'hfe00707f
`define INST_XOR        32'h00004033
`define INST_XOR_MASK   32'hfe00707f
`define INST_OR         32'h00006033
`define INST_OR_MASK    32'hfe00707f
`define INST_AND        32'h00007033
`define INST_AND_MASK   32'hfe00707f
`define INST_SLL        32'h00001033
`define INST_SLL_MASK   32'hfe00707f
`define INST_SRL        32'h00005033
`define INST_SRL_MASK   32'hfe00707f
`define INST_SRA        32'h40005033
`define INST_SRA_MASK   32'hfe00707f

// Jumps and Branches
`define INST_JAL        32'h0000006f
`define INST_JAL_MASK   32'h0000007f
`define INST_JALR       32'h00000067
`define INST_JALR_MASK  32'h0000707f
`define INST_BEQ        32'h00000063
`define INST_BEQ_MASK   32'h0000707f
`define INST_BNE        32'h00001063
`define INST_BNE_MASK   32'h0000707f
`define INST_BLT        32'h00004063
`define INST_BLT_MASK   32'h0000707f
`define INST_BGE        32'h00005063
`define INST_BGE_MASK   32'h0000707f
`define INST_BLTU       32'h00006063
`define INST_BLTU_MASK  32'h0000707f
`define INST_BGEU       32'h00007063
`define INST_BGEU_MASK  32'h0000707f

// Load/Store
`define INST_LB         32'h00000003
`define INST_LB_MASK    32'h0000707f
`define INST_LH         32'h00001003
`define INST_LH_MASK    32'h0000707f
`define INST_LW         32'h00002003
`define INST_LW_MASK    32'h0000707f
`define INST_LBU        32'h00004003
`define INST_LBU_MASK   32'h0000707f
`define INST_LHU        32'h00005003
`define INST_LHU_MASK   32'h0000707f
`define INST_LWU        32'h00006003
`define INST_LWU_MASK   32'h0000707f
`define INST_SB         32'h00000023
`define INST_SB_MASK    32'h0000707f
`define INST_SH         32'h00001023
`define INST_SH_MASK    32'h0000707f
`define INST_SW         32'h00002023
`define INST_SW_MASK    32'h0000707f

// System / Traps
`define INST_ECALL      32'h00000073
`define INST_ECALL_MASK 32'hffffffff
`define INST_EBREAK     32'h00100073
`define INST_EBREAK_MASK 32'hffffffff
`define INST_ERET       32'h20000073
`define INST_ERET_MASK  32'hcfffffff

// CSR Access
`define INST_CSRRW      32'h00001073
`define INST_CSRRW_MASK 32'h0000707f
`define INST_CSRRS      32'h00002073
`define INST_CSRRS_MASK 32'h0000707f
`define INST_CSRRC      32'h00003073
`define INST_CSRRC_MASK 32'h0000707f
`define INST_CSRRWI     32'h00005073
`define INST_CSRRWI_MASK 32'h0000707f
`define INST_CSRRSI     32'h00006073
`define INST_CSRRSI_MASK 32'h0000707f
`define INST_CSRRCI     32'h00007073
`define INST_CSRRCI_MASK 32'h0000707f

// Multiply / Divide Extension (M)
`define INST_MUL        32'h02000033
`define INST_MUL_MASK   32'hfe00707f
`define INST_MULH       32'h02001033
`define INST_MULH_MASK  32'hfe00707f
`define INST_MULHSU     32'h02002033
`define INST_MULHSU_MASK 32'hfe00707f
`define INST_MULHU      32'h02003033
`define INST_MULHU_MASK 32'hfe00707f
`define INST_DIV        32'h02004033
`define INST_DIV_MASK   32'hfe00707f
`define INST_DIVU       32'h02005033
`define INST_DIVU_MASK  32'hfe00707f
`define INST_REM        32'h02006033
`define INST_REM_MASK   32'hfe00707f
`define INST_REMU       32'h02007033
`define INST_REMU_MASK  32'hfe00707f

// Fences
`define INST_WFI        32'h10500073
`define INST_WFI_MASK   32'hffff8fff
`define INST_FENCE      32'h0000000f
`define INST_FENCE_MASK 32'h0000707f
`define INST_SFENCE     32'h12000073
`define INST_SFENCE_MASK 32'hfe007fff
`define INST_IFENCE     32'h0000100f
`define INST_IFENCE_MASK 32'h0000707f

// Privilege Modes
`define PRIV_USER       2'd0
`define PRIV_SUPER      2'd1
`define PRIV_MACHINE    2'd3

// Interrupt Numbers (IRQ)
`define IRQ_S_SOFT      1
`define IRQ_M_SOFT      3
`define IRQ_S_TIMER     5
`define IRQ_M_TIMER     7
`define IRQ_S_EXT       9
`define IRQ_M_EXT       11

`define IRQ_MIN         `IRQ_S_SOFT
`define IRQ_MAX         (`IRQ_M_EXT + 1)
`define IRQ_MASK        ((1<<`IRQ_M_EXT)|(1<<`IRQ_S_EXT)|(1<<`IRQ_M_TIMER)|(1<<`IRQ_S_TIMER)|(1<<`IRQ_M_SOFT)|(1<<`IRQ_S_SOFT))

// M-Mode CSRs
`define CSR_MSTATUS     12'h300
`define CSR_MSTATUS_MASK 32'hFFFFFFFF
`define CSR_MISA        12'h301
`define CSR_MISA_MASK   32'hFFFFFFFF
`define CSR_MEDELEG     12'h302
`define CSR_MEDELEG_MASK 32'h0000FFFF
`define CSR_MIDELEG     12'h303
`define CSR_MIDELEG_MASK 32'h0000FFFF
`define CSR_MIE         12'h304
`define CSR_MIE_MASK    `IRQ_MASK
`define CSR_MTVEC       12'h305
`define CSR_MTVEC_MASK  32'hFFFFFFFF
`define CSR_MSCRATCH    12'h340
`define CSR_MSCRATCH_MASK 32'hFFFFFFFF
`define CSR_MEPC        12'h341
`define CSR_MEPC_MASK   32'hFFFFFFFF
`define CSR_MCAUSE      12'h342
`define CSR_MCAUSE_MASK 32'h8000000F
`define CSR_MTVAL       12'h343
`define CSR_MTVAL_MASK  32'hFFFFFFFF
`define CSR_MIP         12'h344
`define CSR_MIP_MASK    `IRQ_MASK
`define CSR_MCYCLE      12'hc00
`define CSR_MCYCLE_MASK 32'hFFFFFFFF
`define CSR_MTIME       12'hc01
`define CSR_MTIME_MASK  32'hFFFFFFFF
`define CSR_MTIMEH      12'hc81
`define CSR_MTIMEH_MASK 32'hFFFFFFFF
`define CSR_MHARTID     12'hF14
`define CSR_MHARTID_MASK 32'hFFFFFFFF
`define CSR_MTIMECMP    12'h7c0
`define CSR_MTIMECMP_MASK 32'hFFFFFFFF

// S-Mode CSRs
`define CSR_SSTATUS     12'h100
`define CSR_SSTATUS_MASK `SR_SMODE_MASK
`define CSR_SIE         12'h104
`define CSR_SIE_MASK    ((1<<`IRQ_S_EXT)|(1<<`IRQ_S_TIMER)|(1<<`IRQ_S_SOFT))
`define CSR_STVEC       12'h105
`define CSR_STVEC_MASK  32'hFFFFFFFF
`define CSR_SSCRATCH    12'h140
`define CSR_SSCRATCH_MASK 32'hFFFFFFFF
`define CSR_SEPC        12'h141
`define CSR_SEPC_MASK   32'hFFFFFFFF
`define CSR_SCAUSE      12'h142
`define CSR_SCAUSE_MASK 32'h8000000F
`define CSR_STVAL       12'h143
`define CSR_STVAL_MASK  32'hFFFFFFFF
`define CSR_SIP         12'h144
`define CSR_SIP_MASK    ((1<<`IRQ_S_EXT)|(1<<`IRQ_S_TIMER)|(1<<`IRQ_S_SOFT))
`define CSR_SATP        12'h180
`define CSR_SATP_MASK   32'hFFFFFFFF

// Cache Control (custom)
`define CSR_DFLUSH          12'h3a0 
`define CSR_DFLUSH_MASK     32'hFFFFFFFF
`define CSR_DWRITEBACK      12'h3a1 
`define CSR_DWRITEBACK_MASK 32'hFFFFFFFF
`define CSR_DINVALIDATE     12'h3a2 
`define CSR_DINVALIDATE_MASK 32'hFFFFFFFF

// Simulator Control
`define CSR_DSCRATCH       12'h7b2
`define CSR_SIM_CTRL       12'h8b2
`define CSR_SIM_CTRL_MASK  32'hFFFFFFFF
`define CSR_SIM_CTRL_EXIT  (0 << 24)
`define CSR_SIM_CTRL_PUTC  (1 << 24)

// MISA bits
`define MISA_RV32     32'h40000000
`define MISA_RVI      32'h00000100
`define MISA_RVE      32'h00000010
`define MISA_RVM      32'h00001000
`define MISA_RVA      32'h00000001
`define MISA_RVF      32'h00000020
`define MISA_RVD      32'h00000008
`define MISA_RVC      32'h00000004
`define MISA_RVS      32'h00040000
`define MISA_RVU      32'h00100000

// Status Register Fields
`define SR_UIE         (1 << 0)
`define SR_SIE         (1 << 1)
`define SR_MIE         (1 << 3)
`define SR_UPIE        (1 << 4)
`define SR_SPIE        (1 << 5)
`define SR_MPIE        (1 << 7)
`define SR_SPP         (1 << 8)
`define SR_UIE_R       0
`define SR_SIE_R       1
`define SR_MIE_R       3
`define SR_UPIE_R      4
`define SR_SPIE_R      5
`define SR_MPIE_R      7
`define SR_SPP_R       8
`define SR_MPP_R       12:11
`define SR_MPP_U       `PRIV_USER
`define SR_MPP_S       `PRIV_SUPER
`define SR_MPP_M       `PRIV_MACHINE
`define SR_SUM_R       18
`define SR_SUM         (1 << `SR_SUM_R)
`define SR_MPRV_R      17
`define SR_MPRV        (1 << `SR_MPRV_R)
`define SR_MXR_R       19
`define SR_MXR         (1 << `SR_MXR_R)
`define SR_SMODE_MASK  (`SR_UIE | `SR_SIE | `SR_UPIE | `SR_SPIE | `SR_SPP | `SR_SUM)

`define SR_IP_MSIP_R      `IRQ_M_SOFT
`define SR_IP_MTIP_R      `IRQ_M_TIMER
`define SR_IP_MEIP_R      `IRQ_M_EXT
`define SR_IP_SSIP_R      `IRQ_S_SOFT
`define SR_IP_STIP_R      `IRQ_S_TIMER
`define SR_IP_SEIP_R      `IRQ_S_EXT

// MMU / Paging
`define SATP_PPN_R        19:0
`define SATP_ASID_R       30:22
`define SATP_MODE_R       31

`define MMU_LEVELS        2
`define MMU_PTIDXBITS     10
`define MMU_PTESIZE       4
`define MMU_PGSHIFT       (`MMU_PTIDXBITS + 2)
`define MMU_PGSIZE        (1 << `MMU_PGSHIFT)
`define MMU_VPN_BITS      (`MMU_PTIDXBITS * `MMU_LEVELS)
`define MMU_PPN_BITS      (32 - `MMU_PGSHIFT)
`define MMU_VA_BITS       (`MMU_VPN_BITS + `MMU_PGSHIFT)
`define PAGE_PRESENT      0
`define PAGE_READ         1
`define PAGE_WRITE        2
`define PAGE_EXEC         3
`define PAGE_USER         4
`define PAGE_GLOBAL       5
`define PAGE_ACCESSED     6
`define PAGE_DIRTY        7
`define PAGE_SOFT         9:8
`define PAGE_FLAGS        10'h3FF
`define PAGE_PFN_SHIFT    10
`define PAGE_SIZE         4096

// Exception Codes
`define EXCEPTION_W                    6
`define EXCEPTION_MISALIGNED_FETCH     6'h10
`define EXCEPTION_FAULT_FETCH          6'h11
`define EXCEPTION_ILLEGAL_INSTRUCTION  6'h12
`define EXCEPTION_BREAKPOINT           6'h13
`define EXCEPTION_MISALIGNED_LOAD      6'h14
`define EXCEPTION_FAULT_LOAD           6'h15
`define EXCEPTION_MISALIGNED_STORE     6'h16
`define EXCEPTION_FAULT_STORE          6'h17
`define EXCEPTION_ECALL                6'h18
`define EXCEPTION_ECALL_U              6'h18
`define EXCEPTION_ECALL_S              6'h19
`define EXCEPTION_ECALL_H              6'h1a
`define EXCEPTION_ECALL_M              6'h1b
`define EXCEPTION_PAGE_FAULT_INST      6'h1c
`define EXCEPTION_PAGE_FAULT_LOAD      6'h1d
`define EXCEPTION_PAGE_FAULT_STORE     6'h1f
`define EXCEPTION_EXCEPTION            6'h10
`define EXCEPTION_INTERRUPT            6'h20
`define EXCEPTION_ERET_U               6'h30
`define EXCEPTION_ERET_S               6'h31
`define EXCEPTION_ERET_H               6'h32
`define EXCEPTION_ERET_M               6'h33
`define EXCEPTION_FENCE                6'h34
`define EXCEPTION_TYPE_MASK            6'h30
`define EXCEPTION_SUBTYPE_R            3:0

`define MCAUSE_INT                     31
`define MCAUSE_INTERRUPT               (1 << `MCAUSE_INT)

// Debug / Regs
`define RISCV_REGNO_FIRST   13'd0
`define RISCV_REGNO_GPR0    13'd0
`define RISCV_REGNO_GPR31   13'd31
`define RISCV_REGNO_PC      13'd32
`define RISCV_REGNO_CSR0    13'd65
`define RISCV_REGNO_CSR4095 (`RISCV_REGNO_CSR0 + 13'd4095)
`define RISCV_REGNO_PRIV    13'd4161