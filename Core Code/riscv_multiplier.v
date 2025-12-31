module riscv_multiplier
(
    input  wire        clk,
    input  wire        rst_n,
    
    input  wire        op_valid,
    input  wire [31:0] op_code,
    input  wire [31:0] op_pc,
    input  wire        op_invalid,
    input  wire [4:0]  op_rd,
    input  wire [4:0]  op_ra,
    input  wire [4:0]  op_rb,
    input  wire [31:0] op_a,
    input  wire [31:0] op_b,
    input  wire        stall,

    output wire [31:0] res_out
);

`include "riscv_defs.v"

localparam LATENCY = 2; // Pipeline stages

reg [31:0] stage2_res;
reg [31:0] stage3_res;

reg [32:0] a_reg;
reg [32:0] b_reg;
reg        hi_sel;

// Instruction decode
wire is_mul     = ((op_code & `INST_MUL_MASK) == `INST_MUL);
wire is_mulh    = ((op_code & `INST_MULH_MASK) == `INST_MULH);
wire is_mulhsu  = ((op_code & `INST_MULHSU_MASK) == `INST_MULHSU);
wire is_mulhu   = ((op_code & `INST_MULHU_MASK) == `INST_MULHU);
wire is_valid   = is_mul | is_mulh | is_mulhsu | is_mulhu;

reg [32:0] op_a_sext;
reg [32:0] op_b_sext;

always @(*) begin
    // Sign extension logic
    if (is_mulhsu || is_mulh)
        op_a_sext = {op_a[31], op_a};
    else 
        op_a_sext = {1'b0, op_a}; // Unsigned

    if (is_mulh)
        op_b_sext = {op_b[31], op_b};
    else
        op_b_sext = {1'b0, op_b}; // Unsigned
end

// Pipeline Stage 1
always @(posedge clk or posedge rst_n)
if (rst_n) begin
    a_reg  <= 33'b0;
    b_reg  <= 33'b0;
    hi_sel <= 1'b0;
end else if (!stall && op_valid && is_valid) begin
    a_reg  <= op_a_sext;
    b_reg  <= op_b_sext;
    hi_sel <= !is_mul; // If not basic MUL, we want the high bits
end else if (!stall) begin
    a_reg  <= 33'b0;
    b_reg  <= 33'b0;
    hi_sel <= 1'b0;
end

// Multiplication (DSP inference expected)
wire [65:0] raw_prod = $signed(a_reg) * $signed(b_reg);

// Result Selection
reg [31:0] next_res;
always @(*) begin
    next_res = hi_sel ? raw_prod[63:32] : raw_prod[31:0];
end

// Pipeline Stage 2
always @(posedge clk or posedge rst_n)
if (rst_n)
    stage2_res <= 32'b0;
else if (!stall)
    stage2_res <= next_res;

// Pipeline Stage 3 (Optional)
always @(posedge clk or posedge rst_n)
if (rst_n)
    stage3_res <= 32'b0;
else if (!stall)
    stage3_res <= stage2_res;

assign res_out = (LATENCY == 3) ? stage3_res : stage2_res;

endmodule