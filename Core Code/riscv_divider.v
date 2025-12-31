module riscv_divider
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

    output wire        res_valid,
    output wire [31:0] res_out
);

`include "riscv_defs.v"

reg        done;
reg [31:0] result;

// Decode
wire is_div  = ((op_code & `INST_DIV_MASK) == `INST_DIV);
wire is_divu = ((op_code & `INST_DIVU_MASK) == `INST_DIVU);
wire is_rem  = ((op_code & `INST_REM_MASK) == `INST_REM);
wire is_remu = ((op_code & `INST_REMU_MASK) == `INST_REMU);

wire any_div = is_div | is_divu | is_rem | is_remu;
wire is_signed = is_div | is_rem;
wire is_division_op = is_div | is_divu; // vs remainder

// State
reg [31:0] dividend;
reg [62:0] divisor;
reg [31:0] quo;
reg [31:0] mask;
reg        is_active;
reg        op_is_div; // 1 for div, 0 for rem
reg        neg_res;   // Invert result at end?

wire start    = op_valid & any_div;
wire finished = (mask == 0) && is_active;

always @(posedge clk or posedge rst_n)
if (rst_n) begin
    is_active <= 0;
    dividend  <= 0;
    divisor   <= 0;
    neg_res   <= 0;
    quo       <= 0;
    mask      <= 0;
    op_is_div <= 0;
end else if (start) begin
    is_active <= 1;
    op_is_div <= is_division_op;
    
    // Absolute values for dividend
    if (is_signed && op_a[31]) dividend <= -op_a;
    else dividend <= op_a;

    // Absolute values for divisor
    if (is_signed && op_b[31]) divisor <= {-op_b, 31'b0};
    else divisor <= {op_b, 31'b0};

    // Determine sign of result
    // DIV: - / + = -
    // REM: Sign follows dividend
    neg_res <= (is_div && (op_a[31] != op_b[31]) && |op_b) || (is_rem && op_a[31]);

    quo  <= 0;
    mask <= 32'h80000000;
end else if (finished) begin
    is_active <= 0;
end else if (is_active) begin
    if (divisor <= {31'b0, dividend}) begin
        dividend <= dividend - divisor[31:0];
        quo      <= quo | mask;
    end
    divisor <= divisor >> 1;
    mask    <= mask >> 1;
end

reg [31:0] final_calc;
always @(*) begin
    if (op_is_div) 
        final_calc = neg_res ? -quo : quo;
    else 
        final_calc = neg_res ? -dividend : dividend;
end

always @(posedge clk or posedge rst_n)
if (rst_n) done <= 0;
else done <= finished;

always @(posedge clk or posedge rst_n)
if (rst_n) result <= 0;
else if (finished) result <= final_calc;

assign res_valid = done;
assign res_out   = result;

endmodule