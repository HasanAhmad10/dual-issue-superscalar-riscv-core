module riscv_exec
(
    input  wire        clk,
    input  wire        rst_n,

    // Instruction Inputs
    input  wire        valid,
    input  wire [31:0] instr,
    input  wire [31:0] pc,
    input  wire        invalid,
    input  wire [4:0]  rd,
    input  wire [4:0]  ra,
    input  wire [4:0]  rb,
    input  wire [31:0] rdata1,
    input  wire [31:0] rdata2,
    input  wire        hold,

    // Branch Outputs
    output wire        br_req,
    output wire        br_taken,
    output wire        br_untaken,
    output wire [31:0] br_source,
    output wire        br_is_call,
    output wire        br_is_ret,
    output wire        br_is_jmp,
    output wire [31:0] br_pc,
    
    // Delayed Branch (for feedback)
    output wire        br_d_req,
    output wire [31:0] br_d_pc,
    output wire [1:0]  br_d_priv,

    // Result Output
    output wire [31:0] result
);

`include "riscv_defs.v"

    //---------------------------------------------------------
    // Immediate Decoding
    //---------------------------------------------------------
    wire [31:0] imm_u = {instr[31:12], 12'b0};
    wire [31:0] imm_i = {{20{instr[31]}}, instr[31:20]};
    wire [31:0] imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] imm_j = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:25], instr[24:21], 1'b0};
    wire [4:0]  shamt = instr[24:20];

    //---------------------------------------------------------
    // ALU Control
    //---------------------------------------------------------
    reg [3:0]  alu_op;
    reg [31:0] alu_a;
    reg [31:0] alu_b;

    always @(*) begin
        alu_op = `ALU_NONE;
        alu_a  = 32'b0;
        alu_b  = 32'b0;

        // R-Type
        if ((instr & `INST_ADD_MASK) == `INST_ADD) begin alu_op = `ALU_ADD; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_SUB_MASK) == `INST_SUB) begin alu_op = `ALU_SUB; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_AND_MASK) == `INST_AND) begin alu_op = `ALU_AND; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_OR_MASK)  == `INST_OR)  begin alu_op = `ALU_OR;  alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_XOR_MASK) == `INST_XOR) begin alu_op = `ALU_XOR; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_SLL_MASK) == `INST_SLL) begin alu_op = `ALU_SHIFTL; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_SRL_MASK) == `INST_SRL) begin alu_op = `ALU_SHIFTR; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_SRA_MASK) == `INST_SRA) begin alu_op = `ALU_SHIFTR_ARITH; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_SLT_MASK) == `INST_SLT) begin alu_op = `ALU_LESS_THAN_SIGNED; alu_a = rdata1; alu_b = rdata2; end
        else if ((instr & `INST_SLTU_MASK)== `INST_SLTU)begin alu_op = `ALU_LESS_THAN; alu_a = rdata1; alu_b = rdata2; end
        
        // I-Type (Arithmetic/Logic)
        else if ((instr & `INST_ADDI_MASK) == `INST_ADDI) begin alu_op = `ALU_ADD; alu_a = rdata1; alu_b = imm_i; end
        else if ((instr & `INST_ANDI_MASK) == `INST_ANDI) begin alu_op = `ALU_AND; alu_a = rdata1; alu_b = imm_i; end
        else if ((instr & `INST_ORI_MASK)  == `INST_ORI)  begin alu_op = `ALU_OR;  alu_a = rdata1; alu_b = imm_i; end
        else if ((instr & `INST_XORI_MASK) == `INST_XORI) begin alu_op = `ALU_XOR; alu_a = rdata1; alu_b = imm_i; end
        else if ((instr & `INST_SLTI_MASK) == `INST_SLTI) begin alu_op = `ALU_LESS_THAN_SIGNED; alu_a = rdata1; alu_b = imm_i; end
        else if ((instr & `INST_SLTIU_MASK)== `INST_SLTIU)begin alu_op = `ALU_LESS_THAN; alu_a = rdata1; alu_b = imm_i; end
        
        // Shifts (Immediate)
        else if ((instr & `INST_SLLI_MASK) == `INST_SLLI) begin alu_op = `ALU_SHIFTL; alu_a = rdata1; alu_b = {27'b0, shamt}; end
        else if ((instr & `INST_SRLI_MASK) == `INST_SRLI) begin alu_op = `ALU_SHIFTR; alu_a = rdata1; alu_b = {27'b0, shamt}; end
        else if ((instr & `INST_SRAI_MASK) == `INST_SRAI) begin alu_op = `ALU_SHIFTR_ARITH; alu_a = rdata1; alu_b = {27'b0, shamt}; end
        
        // Upper Immediates
        else if ((instr & `INST_LUI_MASK)   == `INST_LUI)   begin alu_op = `ALU_NONE; alu_a = imm_u; end // Pass through
        else if ((instr & `INST_AUIPC_MASK) == `INST_AUIPC) begin alu_op = `ALU_ADD; alu_a = pc; alu_b = imm_u; end
        
        // Jumps (Link calculation)
        else if (((instr & `INST_JAL_MASK) == `INST_JAL) || ((instr & `INST_JALR_MASK) == `INST_JALR)) 
        begin 
            alu_op = `ALU_ADD; alu_a = pc; alu_b = 32'd4; 
        end
    end

    wire [31:0] alu_res;
    
    riscv_alu u_alu (
        .op     (alu_op),
        .in_a   (alu_a),
        .in_b   (alu_b),
        .res_out(alu_res)
    );

    // Output Register
    reg [31:0] res_q;
    always @(posedge clk or posedge rst_n)
        if (rst_n) res_q <= 32'b0;
        else if (!hold) res_q <= alu_res;

    assign result = res_q;

    //---------------------------------------------------------
    // Branch Logic
    //---------------------------------------------------------
    reg branch_hit;
    reg [31:0] branch_tgt;
    reg is_call, is_ret, is_jmp;

    // Comparison Functions
    function signed_less(input [31:0] a, input [31:0] b);
        signed_less = ($signed(a) < $signed(b));
    endfunction

    always @(*) begin
        branch_hit = 1'b0;
        branch_tgt = pc + imm_b; // Default branch target
        is_call = 1'b0;
        is_ret  = 1'b0;
        is_jmp  = 1'b0;

        if ((instr & `INST_JAL_MASK) == `INST_JAL) begin
            branch_hit = 1'b1;
            branch_tgt = pc + imm_j;
            is_jmp     = 1'b1;
            if (rd == 5'd1) is_call = 1'b1;
        end
        else if ((instr & `INST_JALR_MASK) == `INST_JALR) begin
            branch_hit = 1'b1;
            branch_tgt = rdata1 + imm_i;
            branch_tgt[0] = 1'b0;
            if (ra == 5'd1 && imm_i[11:0] == 12'b0) is_ret = 1'b1;
            else if (rd == 5'd1) is_call = 1'b1;
            else is_jmp = 1'b1;
        end
        else if ((instr & `INST_BEQ_MASK) == `INST_BEQ)  branch_hit = (rdata1 == rdata2);
        else if ((instr & `INST_BNE_MASK) == `INST_BNE)  branch_hit = (rdata1 != rdata2);
        else if ((instr & `INST_BLT_MASK) == `INST_BLT)  branch_hit = signed_less(rdata1, rdata2);
        else if ((instr & `INST_BGE_MASK) == `INST_BGE)  branch_hit = !signed_less(rdata1, rdata2);
        else if ((instr & `INST_BLTU_MASK)== `INST_BLTU) branch_hit = (rdata1 < rdata2);
        else if ((instr & `INST_BGEU_MASK)== `INST_BGEU) branch_hit = (rdata1 >= rdata2);
    end

    // Branch State Registers
    reg taken_q, ntaken_q;
    reg [31:0] pc_x_q; // Executing PC
    reg [31:0] pc_m_q; // Memory PC (source)
    reg call_q, ret_q, jmp_q;

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            taken_q <= 0; ntaken_q <= 0; pc_x_q <= 0; pc_m_q <= 0;
            call_q <= 0; ret_q <= 0; jmp_q <= 0;
        end else if (valid) begin
            taken_q  <= branch_hit;
            ntaken_q <= !branch_hit; // Only useful if it WAS a branch instruction
            // If taken, next PC is target, else PC+4
            pc_x_q   <= branch_hit ? branch_tgt : pc + 4;
            pc_m_q   <= pc;
            
            call_q   <= branch_hit & is_call;
            ret_q    <= branch_hit & is_ret;
            jmp_q    <= branch_hit & is_jmp;
        end
    end

    // Mask non-branch instructions from triggering outputs
    // (Ideally logic above should filter 'valid' based on is_branch)
    // Assuming 'valid' here implies an exec/branch op is present.
    
    // Wire outputs
    wire is_any_branch = ((instr & 7'h7f) == 7'b1100011) || is_jmp; // Branch opcode or JAL/JALR

    assign br_req       = (taken_q | ntaken_q) & valid; 
    assign br_taken     = taken_q;
    assign br_untaken   = ntaken_q;
    assign br_source    = pc_m_q;
    assign br_pc        = pc_x_q;
    
    assign br_is_call   = call_q;
    assign br_is_ret    = ret_q;
    assign br_is_jmp    = jmp_q;

    assign br_d_req     = branch_hit & valid;
    assign br_d_pc      = branch_tgt;
    assign br_d_priv    = 2'b0; // Unused in simple core

endmodule