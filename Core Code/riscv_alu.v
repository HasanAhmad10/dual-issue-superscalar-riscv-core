module riscv_alu
(
    input  [3:0]  op,
    input  [31:0] in_a,
    input  [31:0] in_b,
    output [31:0] res_out
);

`include "riscv_defs.v"

reg [31:0] result;
reg [31:16] fill_mask;

// Shift intermediate stages
reg [31:0] sr1, sr2, sr4, sr8;
reg [31:0] sl1, sl2, sl4, sl8;

wire [31:0] diff = in_a - in_b;

always @ (*)
begin
    // Defaults
    fill_mask = 16'b0;
    sr1 = 32'b0; sr2 = 32'b0; sr4 = 32'b0; sr8 = 32'b0;
    sl1 = 32'b0; sl2 = 32'b0; sl4 = 32'b0; sl8 = 32'b0;

    case (op)
       // Shift Left Logic
       `ALU_SHIFTL :
       begin
            sl1 = (in_b[0]) ? {in_a[30:0], 1'b0} : in_a;
            sl2 = (in_b[1]) ? {sl1[29:0], 2'b00} : sl1;
            sl4 = (in_b[2]) ? {sl2[27:0], 4'b0}  : sl2;
            sl8 = (in_b[3]) ? {sl4[23:0], 8'b0}  : sl4;
            
            if (in_b[4])
                result = {sl8[15:0], 16'b0};
            else
                result = sl8;
       end

       // Shift Right (Logical & Arithmetic)
       `ALU_SHIFTR, `ALU_SHIFTR_ARITH:
       begin
            // Arithmetic shift: fill with sign bit if MSB is set
            if (in_a[31] && op == `ALU_SHIFTR_ARITH)
                fill_mask = 16'hFFFF;
            else
                fill_mask = 16'h0000;

            sr1 = (in_b[0]) ? {fill_mask[31], in_a[31:1]}    : in_a;
            sr2 = (in_b[1]) ? {fill_mask[31:30], sr1[31:2]}  : sr1;
            sr4 = (in_b[2]) ? {fill_mask[31:28], sr2[31:4]}  : sr2;
            sr8 = (in_b[3]) ? {fill_mask[31:24], sr4[31:8]}  : sr4;

            if (in_b[4])
                result = {fill_mask[31:16], sr8[31:16]};
            else
                result = sr8;
       end       

       // Arithmetic Ops
       `ALU_ADD : result = in_a + in_b;
       `ALU_SUB : result = diff;

       // Boolean Logic
       `ALU_AND : result = in_a & in_b;
       `ALU_OR  : result = in_a | in_b;
       `ALU_XOR : result = in_a ^ in_b;

       // Comparisons
       `ALU_LESS_THAN : result = (in_a < in_b) ? 32'd1 : 32'd0;

       `ALU_LESS_THAN_SIGNED : 
       begin
            if (in_a[31] != in_b[31])
                result = in_a[31] ? 32'd1 : 32'd0;
            else
                result = diff[31] ? 32'd1 : 32'd0;            
       end       

       default  : result = in_a;
    endcase
end

assign res_out = result;

endmodule