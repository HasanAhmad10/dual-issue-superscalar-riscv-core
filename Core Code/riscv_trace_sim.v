`include "riscv_defs.v"

module riscv_trace_sim
(
    input  wire        valid,
    input  wire [31:0] pc,
    input  wire [31:0] instr
);

`ifdef verilator
    //---------------------------------------------------------
    // Helper: Register Name String
    //---------------------------------------------------------
    function [79:0] get_reg_name;
        input [4:0] r;
        begin
            case (r)
                5'd0:  get_reg_name = "zero";
                5'd1:  get_reg_name = "ra";
                5'd2:  get_reg_name = "sp";
                5'd3:  get_reg_name = "gp";
                5'd4:  get_reg_name = "tp";
                5'd5:  get_reg_name = "t0";
                5'd6:  get_reg_name = "t1";
                5'd7:  get_reg_name = "t2";
                5'd8:  get_reg_name = "s0";
                5'd9:  get_reg_name = "s1";
                5'd10: get_reg_name = "a0";
                5'd11: get_reg_name = "a1";
                5'd12: get_reg_name = "a2";
                5'd13: get_reg_name = "a3";
                5'd14: get_reg_name = "a4";
                5'd15: get_reg_name = "a5";
                5'd16: get_reg_name = "a6";
                5'd17: get_reg_name = "a7";
                5'd18: get_reg_name = "s2";
                5'd19: get_reg_name = "s3";
                5'd20: get_reg_name = "s4";
                5'd21: get_reg_name = "s5";
                5'd22: get_reg_name = "s6";
                5'd23: get_reg_name = "s7";
                5'd24: get_reg_name = "s8";
                5'd25: get_reg_name = "s9";
                5'd26: get_reg_name = "s10";
                5'd27: get_reg_name = "s11";
                5'd28: get_reg_name = "t3";
                5'd29: get_reg_name = "t4";
                5'd30: get_reg_name = "t5";
                5'd31: get_reg_name = "t6";
                default: get_reg_name = "unknown";
            endcase
        end
    endfunction

    //---------------------------------------------------------
    // Debug String Generation
    //---------------------------------------------------------
    reg [79:0] str_mnem;
    reg [79:0] str_ra;
    reg [79:0] str_rb;
    reg [79:0] str_rd;
    reg [31:0] val_imm;
    reg [31:0] val_pc;

    wire [4:0] idx_ra = instr[19:15];
    wire [4:0] idx_rb = instr[24:20];
    wire [4:0] idx_rd = instr[11:7];

    // Decoding Helpers
    wire [31:0] imm_i = {{20{instr[31]}}, instr[31:20]};
    wire [31:0] imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    wire [31:0] imm_u = {instr[31:12], 12'b0};
    wire [31:0] imm_j = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:25], instr[24:21], 1'b0};
    wire [4:0]  shamt = instr[24:20];

    always @(*) begin
        str_mnem = "-";
        str_ra   = "-";
        str_rb   = "-";
        str_rd   = "-";
        val_pc   = 32'bx;
        val_imm  = 32'bx;

        if (valid) begin
            val_pc = pc;
            str_ra = get_reg_name(idx_ra);
            str_rb = get_reg_name(idx_rb);
            str_rd = get_reg_name(idx_rd);

            // Decode Mnemonic
            if      ((instr & `INST_ADD_MASK) == `INST_ADD) str_mnem = "add";
            else if ((instr & `INST_SUB_MASK) == `INST_SUB) str_mnem = "sub";
            else if ((instr & `INST_XOR_MASK) == `INST_XOR) str_mnem = "xor";
            else if ((instr & `INST_OR_MASK)  == `INST_OR)  str_mnem = "or";
            else if ((instr & `INST_AND_MASK) == `INST_AND) str_mnem = "and";
            else if ((instr & `INST_SLL_MASK) == `INST_SLL) str_mnem = "sll";
            else if ((instr & `INST_SRL_MASK) == `INST_SRL) str_mnem = "srl";
            else if ((instr & `INST_SRA_MASK) == `INST_SRA) str_mnem = "sra";
            else if ((instr & `INST_SLT_MASK) == `INST_SLT) str_mnem = "slt";
            else if ((instr & `INST_SLTU_MASK)== `INST_SLTU)str_mnem = "sltu";
            
            else if ((instr & `INST_ADDI_MASK)== `INST_ADDI)begin str_mnem = "addi"; val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_XORI_MASK)== `INST_XORI)begin str_mnem = "xori"; val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_ORI_MASK) == `INST_ORI) begin str_mnem = "ori";  val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_ANDI_MASK)== `INST_ANDI)begin str_mnem = "andi"; val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_SLTI_MASK)== `INST_SLTI)begin str_mnem = "slti"; val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_SLTIU_MASK)==`INST_SLTIU)begin str_mnem="sltiu";val_imm = imm_i; str_rb = "-"; end
            
            else if ((instr & `INST_SLLI_MASK)== `INST_SLLI)begin str_mnem = "slli"; val_imm = {27'b0, shamt}; str_rb = "-"; end
            else if ((instr & `INST_SRLI_MASK)== `INST_SRLI)begin str_mnem = "srli"; val_imm = {27'b0, shamt}; str_rb = "-"; end
            else if ((instr & `INST_SRAI_MASK)== `INST_SRAI)begin str_mnem = "srai"; val_imm = {27'b0, shamt}; str_rb = "-"; end

            else if ((instr & `INST_LUI_MASK) == `INST_LUI) begin str_mnem = "lui";  val_imm = imm_u; str_ra = "-"; str_rb = "-"; end
            else if ((instr & `INST_AUIPC_MASK)== `INST_AUIPC)begin str_mnem = "auipc";val_imm= imm_u; str_ra = "pc"; str_rb = "-"; end

            else if ((instr & `INST_JAL_MASK) == `INST_JAL) begin 
                str_mnem = (idx_rd == 1) ? "call" : "jal"; 
                val_imm  = pc + imm_j; 
                str_ra = "-"; str_rb = "-"; 
            end
            else if ((instr & `INST_JALR_MASK)== `INST_JALR)begin 
                if (idx_ra == 1 && imm_i == 0) str_mnem = "ret";
                else str_mnem = "jalr";
                val_imm = imm_i; str_rb = "-"; 
            end

            else if ((instr & `INST_BEQ_MASK) == `INST_BEQ) begin str_mnem = "beq"; str_rd = "-"; end
            else if ((instr & `INST_BNE_MASK) == `INST_BNE) begin str_mnem = "bne"; str_rd = "-"; end
            else if ((instr & `INST_BLT_MASK) == `INST_BLT) begin str_mnem = "blt"; str_rd = "-"; end
            else if ((instr & `INST_BGE_MASK) == `INST_BGE) begin str_mnem = "bge"; str_rd = "-"; end
            else if ((instr & `INST_BLTU_MASK)== `INST_BLTU)begin str_mnem = "bltu";str_rd = "-"; end
            else if ((instr & `INST_BGEU_MASK)== `INST_BGEU)begin str_mnem = "bgeu";str_rd = "-"; end

            else if ((instr & `INST_LB_MASK)  == `INST_LB)  begin str_mnem = "lb";  val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_LH_MASK)  == `INST_LH)  begin str_mnem = "lh";  val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_LW_MASK)  == `INST_LW)  begin str_mnem = "lw";  val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_LBU_MASK) == `INST_LBU) begin str_mnem = "lbu"; val_imm = imm_i; str_rb = "-"; end
            else if ((instr & `INST_LHU_MASK) == `INST_LHU) begin str_mnem = "lhu"; val_imm = imm_i; str_rb = "-"; end
            
            else if ((instr & `INST_SB_MASK)  == `INST_SB)  begin str_mnem = "sb";  val_imm = imm_s; str_rd = "-"; end
            else if ((instr & `INST_SH_MASK)  == `INST_SH)  begin str_mnem = "sh";  val_imm = imm_s; str_rd = "-"; end
            else if ((instr & `INST_SW_MASK)  == `INST_SW)  begin str_mnem = "sw";  val_imm = imm_s; str_rd = "-"; end
            
            // System
            else if ((instr & `INST_ECALL_MASK) == `INST_ECALL) str_mnem = "ecall";
            else if ((instr & `INST_EBREAK_MASK)== `INST_EBREAK)str_mnem = "ebreak";
            else if ((instr & `INST_ERET_MASK)  == `INST_ERET)  str_mnem = "mret";
            else if ((instr & `INST_CSRRW_MASK) == `INST_CSRRW) begin str_mnem = "csrrw"; str_rb = "-"; end
            else if ((instr & `INST_CSRRS_MASK) == `INST_CSRRS) begin str_mnem = "csrrs"; str_rb = "-"; end
            else if ((instr & `INST_CSRRC_MASK) == `INST_CSRRC) begin str_mnem = "csrrc"; str_rb = "-"; end
        end
    end
`endif

endmodule