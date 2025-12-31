module riscv_fetch
#(
    parameter SUPPORT_MMU = 1
)
(
    input  wire        clk,
    input  wire        rst_n, // Active high reset in original, kept high
    
    // Core Interface
    input  wire        accept_in,
    output wire        valid_out,
    output wire [31:0] instr_out,
    output wire [31:0] pc_out,
    output wire        fault_fetch_out,
    output wire        fault_page_out,

    // Instruction Cache / Memory Interface
    input  wire        mem_accept,
    input  wire        mem_valid,
    input  wire        mem_error,
    input  wire [31:0] mem_inst,
    input  wire        mem_page_fault,
    output wire        mem_rd,
    output wire        mem_flush,
    output wire        mem_invalidate,
    output wire [31:0] mem_pc,
    output wire [1:0]  mem_priv,

    // Control
    input  wire        invalidate_req,
    input  wire        branch_req,
    input  wire [31:0] branch_target,
    input  wire [1:0]  branch_priv,
    output wire        decode_squash
);

`include "riscv_defs.v"

//-------------------------------------------------------------
// State Registers
//-------------------------------------------------------------
reg        is_active;
wire       mem_busy;
wire       stall = !accept_in || mem_busy || !mem_accept;

//-------------------------------------------------------------
// Buffered Branch State
//-------------------------------------------------------------
reg        br_queued;
reg [31:0] br_target_q;
reg [1:0]  br_priv_q;

always @ (posedge clk or posedge rst_n)
if (rst_n) begin
    br_queued   <= 1'b0;
    br_target_q <= 32'b0;
    br_priv_q   <= `PRIV_MACHINE;
end else if (branch_req) begin
    br_queued   <= 1'b1;
    br_target_q <= branch_target;
    br_priv_q   <= branch_priv;
end else if (mem_rd && mem_accept) begin
    br_queued   <= 1'b0;
    br_target_q <= 32'b0;
end

wire        do_branch   = br_queued;
wire [31:0] next_pc_br  = br_target_q;
wire [1:0]  next_priv   = br_priv_q;

assign decode_squash    = branch_req;

//-------------------------------------------------------------
// Active Flag
//-------------------------------------------------------------
always @ (posedge clk or posedge rst_n)
if (rst_n)
    is_active <= 1'b0;
else if (do_branch && ~stall)
    is_active <= 1'b1;

//-------------------------------------------------------------
// Request Tracking
//-------------------------------------------------------------
reg req_pending;
reg inv_pending;

always @ (posedge clk or posedge rst_n)
if (rst_n)
    req_pending <= 1'b0;
else if (mem_rd && mem_accept)
    req_pending <= 1'b1;
else if (mem_valid)
    req_pending <= 1'b0;

always @ (posedge clk or posedge rst_n)
if (rst_n)
    inv_pending <= 1'b0;
else if (mem_invalidate && !mem_accept)
    inv_pending <= 1'b1;
else
    inv_pending <= 1'b0;

//-------------------------------------------------------------
// PC Management
//-------------------------------------------------------------
reg [31:0] pc_f;
reg [31:0] pc_d; // Last fetched
reg [1:0]  priv_f;
reg        drop_resp;

wire [31:0] next_pc_w;

always @ (posedge clk or posedge rst_n)
if (rst_n)
    pc_f <= 32'b0;
else if (do_branch && ~stall)
    pc_f <= next_pc_br;
else if (!stall)
    pc_f <= {next_pc_w[31:2], 2'b0} + 32'd4;

always @ (posedge clk or posedge rst_n)
if (rst_n)
    priv_f <= `PRIV_MACHINE;
else if (do_branch && ~stall)
    priv_f <= next_priv;

always @ (posedge clk or posedge rst_n)
if (rst_n)
    drop_resp <= 1'b0;
else if (do_branch && ~stall)
    drop_resp <= 1'b1;
else if (!stall)
    drop_resp <= 1'b0;

assign next_pc_w = pc_f;
wire fetch_resp_drop = do_branch | drop_resp;

always @ (posedge clk or posedge rst_n)
if (rst_n)
    pc_d <= 32'b0;
else if (mem_rd && mem_accept)
    pc_d <= next_pc_w;

//-------------------------------------------------------------
// Outputs
//-------------------------------------------------------------
assign mem_rd         = is_active & accept_in & !mem_busy;
assign mem_pc         = {next_pc_w[31:2], 2'b0};
assign mem_priv       = priv_f;
assign mem_flush      = invalidate_req | inv_pending;
assign mem_invalidate = 1'b0;

assign mem_busy       = req_pending && !mem_valid;

//-------------------------------------------------------------
// Skid Buffer (Backpressure handling)
//-------------------------------------------------------------
reg [65:0] skid_buf;
reg        skid_valid;

always @ (posedge clk or posedge rst_n)
if (rst_n) begin
    skid_buf   <= 66'b0;
    skid_valid <= 1'b0;
end else if (valid_out && !accept_in) begin
    skid_valid <= 1'b1;
    skid_buf   <= {fault_page_out, fault_fetch_out, pc_out, instr_out};
end else begin
    skid_valid <= 1'b0;
    skid_buf   <= 66'b0;
end

assign valid_out       = (mem_valid || skid_valid) & !fetch_resp_drop;
assign pc_out          = skid_valid ? skid_buf[63:32] : {pc_d[31:2], 2'b0};
assign instr_out       = skid_valid ? skid_buf[31:0]  : mem_inst;
assign fault_fetch_out = skid_valid ? skid_buf[64]    : mem_error;
assign fault_page_out  = skid_valid ? skid_buf[65]    : mem_page_fault;

endmodule