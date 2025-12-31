`include "riscv_defs.v"

module riscv_decoder
(
    // Inputs
    input  wire        valid,
    input  wire        fault_in,
    input  wire        en_muldiv,
    input  wire [31:0] instr,

    // Outputs
    output wire        is_invalid,
    output wire        is_exec,
    output wire        is_lsu,
    output wire        is_branch,
    output wire        is_mul,
    output wire        is_div,
    output wire        is_csr,
    output wire        writes_rd
);

    //---------------------------------------------------------
    // Instruction Decoding
    //---------------------------------------------------------

    // Check for invalid instructions
    wire valid_inst = 
        ((instr & `INST_ANDI_MASK) == `INST_ANDI)     ||
        ((instr & `INST_ADDI_MASK) == `INST_ADDI)     ||
        ((instr & `INST_SLTI_MASK) == `INST_SLTI)     ||
        ((instr & `INST_SLTIU_MASK) == `INST_SLTIU)   ||
        ((instr & `INST_ORI_MASK) == `INST_ORI)       ||
        ((instr & `INST_XORI_MASK) == `INST_XORI)     ||
        ((instr & `INST_SLLI_MASK) == `INST_SLLI)     ||
        ((instr & `INST_SRLI_MASK) == `INST_SRLI)     ||
        ((instr & `INST_SRAI_MASK) == `INST_SRAI)     ||
        ((instr & `INST_LUI_MASK) == `INST_LUI)       ||
        ((instr & `INST_AUIPC_MASK) == `INST_AUIPC)   ||
        ((instr & `INST_ADD_MASK) == `INST_ADD)       ||
        ((instr & `INST_SUB_MASK) == `INST_SUB)       ||
        ((instr & `INST_SLT_MASK) == `INST_SLT)       ||
        ((instr & `INST_SLTU_MASK) == `INST_SLTU)     ||
        ((instr & `INST_XOR_MASK) == `INST_XOR)       ||
        ((instr & `INST_OR_MASK) == `INST_OR)         ||
        ((instr & `INST_AND_MASK) == `INST_AND)       ||
        ((instr & `INST_SLL_MASK) == `INST_SLL)       ||
        ((instr & `INST_SRL_MASK) == `INST_SRL)       ||
        ((instr & `INST_SRA_MASK) == `INST_SRA)       ||
        ((instr & `INST_JAL_MASK) == `INST_JAL)       ||
        ((instr & `INST_JALR_MASK) == `INST_JALR)     ||
        ((instr & `INST_BEQ_MASK) == `INST_BEQ)       ||
        ((instr & `INST_BNE_MASK) == `INST_BNE)       ||
        ((instr & `INST_BLT_MASK) == `INST_BLT)       ||
        ((instr & `INST_BGE_MASK) == `INST_BGE)       ||
        ((instr & `INST_BLTU_MASK) == `INST_BLTU)     ||
        ((instr & `INST_BGEU_MASK) == `INST_BGEU)     ||
        ((instr & `INST_LB_MASK) == `INST_LB)         ||
        ((instr & `INST_LH_MASK) == `INST_LH)         ||
        ((instr & `INST_LW_MASK) == `INST_LW)         ||
        ((instr & `INST_LBU_MASK) == `INST_LBU)       ||
        ((instr & `INST_LHU_MASK) == `INST_LHU)       ||
        ((instr & `INST_LWU_MASK) == `INST_LWU)       ||
        ((instr & `INST_SB_MASK) == `INST_SB)         ||
        ((instr & `INST_SH_MASK) == `INST_SH)         ||
        ((instr & `INST_SW_MASK) == `INST_SW)         ||
        ((instr & `INST_ECALL_MASK) == `INST_ECALL)   ||
        ((instr & `INST_EBREAK_MASK) == `INST_EBREAK) ||
        ((instr & `INST_ERET_MASK) == `INST_ERET)     ||
        ((instr & `INST_CSRRW_MASK) == `INST_CSRRW)   ||
        ((instr & `INST_CSRRS_MASK) == `INST_CSRRS)   ||
        ((instr & `INST_CSRRC_MASK) == `INST_CSRRC)   ||
        ((instr & `INST_CSRRWI_MASK) == `INST_CSRRWI) ||
        ((instr & `INST_CSRRSI_MASK) == `INST_CSRRSI) ||
        ((instr & `INST_CSRRCI_MASK) == `INST_CSRRCI) ||
        ((instr & `INST_WFI_MASK) == `INST_WFI)       ||
        ((instr & `INST_FENCE_MASK) == `INST_FENCE)   ||
        ((instr & `INST_IFENCE_MASK) == `INST_IFENCE) ||
        ((instr & `INST_SFENCE_MASK) == `INST_SFENCE) ||
        (en_muldiv && (
            ((instr & `INST_MUL_MASK) == `INST_MUL)       ||
            ((instr & `INST_MULH_MASK) == `INST_MULH)     ||
            ((instr & `INST_MULHSU_MASK) == `INST_MULHSU) ||
            ((instr & `INST_MULHU_MASK) == `INST_MULHU)   ||
            ((instr & `INST_DIV_MASK) == `INST_DIV)       ||
            ((instr & `INST_DIVU_MASK) == `INST_DIVU)     ||
            ((instr & `INST_REM_MASK) == `INST_REM)       ||
            ((instr & `INST_REMU_MASK) == `INST_REMU)
        ));

    assign is_invalid = valid && ~valid_inst;

    //---------------------------------------------------------
    // Output Logic
    //---------------------------------------------------------

    // Does this instruction write to a destination register (RD)?
    assign writes_rd = 
        ((instr & `INST_JALR_MASK) == `INST_JALR)     ||
        ((instr & `INST_JAL_MASK) == `INST_JAL)       ||
        ((instr & `INST_LUI_MASK) == `INST_LUI)       ||
        ((instr & `INST_AUIPC_MASK) == `INST_AUIPC)   ||
        ((instr & `INST_ADDI_MASK) == `INST_ADDI)     ||
        ((instr & `INST_SLLI_MASK) == `INST_SLLI)     ||
        ((instr & `INST_SLTI_MASK) == `INST_SLTI)     ||
        ((instr & `INST_SLTIU_MASK) == `INST_SLTIU)   ||
        ((instr & `INST_XORI_MASK) == `INST_XORI)     ||
        ((instr & `INST_SRLI_MASK) == `INST_SRLI)     ||
        ((instr & `INST_SRAI_MASK) == `INST_SRAI)     ||
        ((instr & `INST_ORI_MASK) == `INST_ORI)       ||
        ((instr & `INST_ANDI_MASK) == `INST_ANDI)     ||
        ((instr & `INST_ADD_MASK) == `INST_ADD)       ||
        ((instr & `INST_SUB_MASK) == `INST_SUB)       ||
        ((instr & `INST_SLL_MASK) == `INST_SLL)       ||
        ((instr & `INST_SLT_MASK) == `INST_SLT)       ||
        ((instr & `INST_SLTU_MASK) == `INST_SLTU)     ||
        ((instr & `INST_XOR_MASK) == `INST_XOR)       ||
        ((instr & `INST_SRL_MASK) == `INST_SRL)       ||
        ((instr & `INST_SRA_MASK) == `INST_SRA)       ||
        ((instr & `INST_OR_MASK) == `INST_OR)         ||
        ((instr & `INST_AND_MASK) == `INST_AND)       ||
        ((instr & `INST_LB_MASK) == `INST_LB)         ||
        ((instr & `INST_LH_MASK) == `INST_LH)         ||
        ((instr & `INST_LW_MASK) == `INST_LW)         ||
        ((instr & `INST_LBU_MASK) == `INST_LBU)       ||
        ((instr & `INST_LHU_MASK) == `INST_LHU)       ||
        ((instr & `INST_LWU_MASK) == `INST_LWU)       ||
        ((instr & `INST_MUL_MASK) == `INST_MUL)       ||
        ((instr & `INST_MULH_MASK) == `INST_MULH)     ||
        ((instr & `INST_MULHSU_MASK) == `INST_MULHSU) ||
        ((instr & `INST_MULHU_MASK) == `INST_MULHU)   ||
        ((instr & `INST_DIV_MASK) == `INST_DIV)       ||
        ((instr & `INST_DIVU_MASK) == `INST_DIVU)     ||
        ((instr & `INST_REM_MASK) == `INST_REM)       ||
        ((instr & `INST_REMU_MASK) == `INST_REMU)     ||
        ((instr & `INST_CSRRW_MASK) == `INST_CSRRW)   ||
        ((instr & `INST_CSRRS_MASK) == `INST_CSRRS)   ||
        ((instr & `INST_CSRRC_MASK) == `INST_CSRRC)   ||
        ((instr & `INST_CSRRWI_MASK) == `INST_CSRRWI) ||
        ((instr & `INST_CSRRSI_MASK) == `INST_CSRRSI) ||
        ((instr & `INST_CSRRCI_MASK) == `INST_CSRRCI);

    // Execution Unit
    assign is_exec = 
        ((instr & `INST_ANDI_MASK) == `INST_ANDI)   ||
        ((instr & `INST_ADDI_MASK) == `INST_ADDI)   ||
        ((instr & `INST_SLTI_MASK) == `INST_SLTI)   ||
        ((instr & `INST_SLTIU_MASK) == `INST_SLTIU) ||
        ((instr & `INST_ORI_MASK) == `INST_ORI)     ||
        ((instr & `INST_XORI_MASK) == `INST_XORI)   ||
        ((instr & `INST_SLLI_MASK) == `INST_SLLI)   ||
        ((instr & `INST_SRLI_MASK) == `INST_SRLI)   ||
        ((instr & `INST_SRAI_MASK) == `INST_SRAI)   ||
        ((instr & `INST_LUI_MASK) == `INST_LUI)     ||
        ((instr & `INST_AUIPC_MASK) == `INST_AUIPC) ||
        ((instr & `INST_ADD_MASK) == `INST_ADD)     ||
        ((instr & `INST_SUB_MASK) == `INST_SUB)     ||
        ((instr & `INST_SLT_MASK) == `INST_SLT)     ||
        ((instr & `INST_SLTU_MASK) == `INST_SLTU)   ||
        ((instr & `INST_XOR_MASK) == `INST_XOR)     ||
        ((instr & `INST_OR_MASK) == `INST_OR)       ||
        ((instr & `INST_AND_MASK) == `INST_AND)     ||
        ((instr & `INST_SLL_MASK) == `INST_SLL)     ||
        ((instr & `INST_SRL_MASK) == `INST_SRL)     ||
        ((instr & `INST_SRA_MASK) == `INST_SRA);

    // Load/Store Unit
    assign is_lsu = 
        ((instr & `INST_LB_MASK) == `INST_LB)   ||
        ((instr & `INST_LH_MASK) == `INST_LH)   ||
        ((instr & `INST_LW_MASK) == `INST_LW)   ||
        ((instr & `INST_LBU_MASK) == `INST_LBU) ||
        ((instr & `INST_LHU_MASK) == `INST_LHU) ||
        ((instr & `INST_LWU_MASK) == `INST_LWU) ||
        ((instr & `INST_SB_MASK) == `INST_SB)   ||
        ((instr & `INST_SH_MASK) == `INST_SH)   ||
        ((instr & `INST_SW_MASK) == `INST_SW);

    // Branch Unit
    assign is_branch = 
        ((instr & `INST_JAL_MASK) == `INST_JAL)     ||
        ((instr & `INST_JALR_MASK) == `INST_JALR)   ||
        ((instr & `INST_BEQ_MASK) == `INST_BEQ)     ||
        ((instr & `INST_BNE_MASK) == `INST_BNE)     ||
        ((instr & `INST_BLT_MASK) == `INST_BLT)     ||
        ((instr & `INST_BGE_MASK) == `INST_BGE)     ||
        ((instr & `INST_BLTU_MASK) == `INST_BLTU)   ||
        ((instr & `INST_BGEU_MASK) == `INST_BGEU);

    // Multiplier Unit
    assign is_mul = en_muldiv && (
        ((instr & `INST_MUL_MASK) == `INST_MUL)       ||
        ((instr & `INST_MULH_MASK) == `INST_MULH)     ||
        ((instr & `INST_MULHSU_MASK) == `INST_MULHSU) ||
        ((instr & `INST_MULHU_MASK) == `INST_MULHU));

    // Divider Unit
    assign is_div = en_muldiv && (
        ((instr & `INST_DIV_MASK) == `INST_DIV)   ||
        ((instr & `INST_DIVU_MASK) == `INST_DIVU) ||
        ((instr & `INST_REM_MASK) == `INST_REM)   ||
        ((instr & `INST_REMU_MASK) == `INST_REMU));

    // CSR / System
    assign is_csr = 
        ((instr & `INST_ECALL_MASK) == `INST_ECALL)   ||
        ((instr & `INST_EBREAK_MASK) == `INST_EBREAK) ||
        ((instr & `INST_ERET_MASK) == `INST_ERET)     ||
        ((instr & `INST_CSRRW_MASK) == `INST_CSRRW)   ||
        ((instr & `INST_CSRRS_MASK) == `INST_CSRRS)   ||
        ((instr & `INST_CSRRC_MASK) == `INST_CSRRC)   ||
        ((instr & `INST_CSRRWI_MASK) == `INST_CSRRWI) ||
        ((instr & `INST_CSRRSI_MASK) == `INST_CSRRSI) ||
        ((instr & `INST_CSRRCI_MASK) == `INST_CSRRCI) ||
        ((instr & `INST_WFI_MASK) == `INST_WFI)       ||
        ((instr & `INST_FENCE_MASK) == `INST_FENCE)   ||
        ((instr & `INST_IFENCE_MASK) == `INST_IFENCE) ||
        ((instr & `INST_SFENCE_MASK) == `INST_SFENCE) ||
        is_invalid || 
        fault_in;

endmodule