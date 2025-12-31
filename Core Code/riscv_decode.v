module riscv_decode
#(
    parameter SUPPORT_MULDIV = 1,
    parameter EXTRA_DECODE_STAGE = 0
)
(
    input  wire        clk,
    input  wire        rst_n,
    
    // Fetch Interface
    input  wire        valid_in,
    input  wire [31:0] instr_in,
    input  wire [31:0] pc_in,
    input  wire        fault_fetch,
    input  wire        fault_page,
    input  wire        accept_out,
    input  wire        squash,

    // Decoded Outputs
    output wire        accept_in,
    output reg         valid_out,
    output reg  [31:0] instr_out,
    output reg  [31:0] pc_out,
    output reg         fault_fetch_out,
    output reg         fault_page_out,
    
    // Control Flags
    output wire        is_exec,
    output wire        is_lsu,
    output wire        is_branch,
    output wire        is_mul,
    output wire        is_div,
    output wire        is_csr,
    output wire        is_rd_valid,
    output wire        is_invalid
);

    wire [31:0] safe_instr = (fault_page | fault_fetch) ? 32'b0 : instr_in;

    generate
    if (EXTRA_DECODE_STAGE) begin : PIPELINED_DECODE
        reg [66:0] buffer; // Packed struct equivalent

        always @(posedge clk or posedge rst_n) begin
            if (rst_n) buffer <= 67'b0;
            else if (squash) buffer <= 67'b0;
            else if (accept_out || !valid_out)
                buffer <= {valid_in, fault_page, fault_fetch, safe_instr, pc_in};
        end

        always @(*) begin
            {valid_out, fault_page_out, fault_fetch_out, instr_out, pc_out} = buffer;
        end
        
        assign accept_in = accept_out;

    end else begin : DIRECT_DECODE
        
        always @(*) begin
            valid_out       = valid_in;
            pc_out          = pc_in;
            instr_out       = safe_instr;
            fault_page_out  = fault_page;
            fault_fetch_out = fault_fetch;
        end

        assign accept_in = accept_out;
    end
    endgenerate

    // Instantiate the raw decoder
    riscv_decoder u_dec (
        .valid      (valid_out),
        .fault_in   (fault_page_out | fault_fetch_out),
        .en_muldiv  (SUPPORT_MULDIV),
        .instr      (instr_out),
        
        .is_invalid (is_invalid),
        .is_exec    (is_exec),
        .is_lsu     (is_lsu),
        .is_branch  (is_branch),
        .is_mul     (is_mul),
        .is_div     (is_div),
        .is_csr     (is_csr),
        .writes_rd  (is_rd_valid)
    );

endmodule