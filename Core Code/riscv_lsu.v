`include "riscv_defs.v"

module riscv_lsu
#(
    parameter MEM_CACHE_ADDR_MIN = 32'h80000000,
    parameter MEM_CACHE_ADDR_MAX = 32'h8fffffff
)
(
    input  wire        clk,
    input  wire        rst_n,

    // Core Pipeline Interface
    input  wire        valid,
    input  wire [31:0] instr,
    input  wire [31:0] pc,
    input  wire        invalid,
    input  wire [4:0]  rd_idx,
    input  wire [4:0]  ra_idx,
    input  wire [4:0]  rb_idx,
    input  wire [31:0] ra_val,
    input  wire [31:0] rb_val,

    // Writeback Interface
    output wire        wb_valid,
    output wire [31:0] wb_val,
    output wire [5:0]  wb_exception,
    output wire        stall,

    // Memory Interface
    input  wire [31:0] mem_rdata,
    input  wire        mem_accept,
    input  wire        mem_ack,
    input  wire        mem_error,
    input  wire [10:0] mem_resp_tag,
    input  wire        mem_load_fault,
    input  wire        mem_store_fault,

    output wire [31:0] mem_addr,
    output wire [31:0] mem_wdata,
    output wire        mem_rd,
    output wire [3:0]  mem_wr,
    output wire        mem_cacheable,
    output wire [10:0] mem_req_tag,
    output wire        mem_invalidate,
    output wire        mem_writeback,
    output wire        mem_flush
);

    //---------------------------------------------------------
    // Registers
    //---------------------------------------------------------
    reg [31:0] addr_q;
    reg [31:0] wdata_q;
    reg        rd_q;
    reg [3:0]  wr_q;
    reg        cacheable_q;
    reg        invalidate_q;
    reg        writeback_q;
    reg        flush_q;
    reg        unaligned_e1_q;
    reg        unaligned_e2_q;
    reg        is_load_q;
    reg        is_byte_q;
    reg        is_half_q;
    reg        is_signed_q;

    //---------------------------------------------------------
    // Decode Opcode
    //---------------------------------------------------------
    wire is_lb  = ((instr & `INST_LB_MASK) == `INST_LB);
    wire is_lbu = ((instr & `INST_LBU_MASK) == `INST_LBU);
    wire is_lh  = ((instr & `INST_LH_MASK) == `INST_LH);
    wire is_lhu = ((instr & `INST_LHU_MASK) == `INST_LHU);
    wire is_lw  = ((instr & `INST_LW_MASK) == `INST_LW);
    wire is_lwu = ((instr & `INST_LWU_MASK) == `INST_LWU);
    
    wire is_sb  = ((instr & `INST_SB_MASK) == `INST_SB);
    wire is_sh  = ((instr & `INST_SH_MASK) == `INST_SH);
    wire is_sw  = ((instr & `INST_SW_MASK) == `INST_SW);

    wire op_load    = is_lb | is_lh | is_lw | is_lbu | is_lhu | is_lwu;
    wire op_store   = is_sb | is_sh | is_sw;
    wire op_signed  = is_lb | is_lh | is_lw;

    // Cache Ops (CSR based)
    wire op_csr_rw  = ((instr & `INST_CSRRW_MASK) == `INST_CSRRW);
    wire op_c_flush = op_csr_rw && (instr[31:20] == `CSR_DFLUSH);
    wire op_c_wb    = op_csr_rw && (instr[31:20] == `CSR_DWRITEBACK);
    wire op_c_inv   = op_csr_rw && (instr[31:20] == `CSR_DINVALIDATE);

    //---------------------------------------------------------
    // Address Calculation & Alignment
    //---------------------------------------------------------
    reg [31:0] addr_c;
    reg [31:0] wdata_c;
    reg        rd_c;
    reg [3:0]  wr_c;
    reg        unaligned_c;

    always @(*) begin
        addr_c      = 32'b0;
        wdata_c     = 32'b0;
        rd_c        = 1'b0;
        wr_c        = 4'b0;
        unaligned_c = 1'b0;

        // Calculate Address
        if (valid && op_csr_rw)
            addr_c = ra_val;
        else if (valid && op_load)
            addr_c = ra_val + {{20{instr[31]}}, instr[31:20]}; // I-Type
        else
            addr_c = ra_val + {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-Type

        // Check Alignment
        if (valid) begin
            if (is_sw || is_lw || is_lwu) 
                unaligned_c = |addr_c[1:0];
            else if (is_sh || is_lh || is_lhu)
                unaligned_c = addr_c[0];
        end

        // Read Enable
        rd_c = valid && op_load && !unaligned_c;

        // Write Data Steering
        if (valid && is_sw && !unaligned_c) begin
            wdata_c = rb_val;
            wr_c    = 4'b1111;
        end else if (valid && is_sh && !unaligned_c) begin
            if (addr_c[1]) begin
                wdata_c = {rb_val[15:0], 16'b0};
                wr_c    = 4'b1100;
            end else begin
                wdata_c = {16'b0, rb_val[15:0]};
                wr_c    = 4'b0011;
            end
        end else if (valid && is_sb) begin
            case (addr_c[1:0])
                2'b00: begin wdata_c = {24'b0, rb_val[7:0]};        wr_c = 4'b0001; end
                2'b01: begin wdata_c = {16'b0, rb_val[7:0], 8'b0};  wr_c = 4'b0010; end
                2'b10: begin wdata_c = {8'b0, rb_val[7:0], 16'b0};  wr_c = 4'b0100; end
                2'b11: begin wdata_c = {rb_val[7:0], 24'b0};        wr_c = 4'b1000; end
            endcase
        end
    end

    //---------------------------------------------------------
    // Response Tracking
    //---------------------------------------------------------
    reg pending_resp;
    wire req_active   = (mem_rd || |mem_wr || mem_writeback || mem_invalidate || mem_flush) && mem_accept;
    wire resp_ok      = mem_ack && !mem_error;
    wire resp_err     = mem_ack && mem_error;
    
    // If we have a pending request but no ack yet, we are stalled
    wire delay_resp   = pending_resp && !resp_ok;

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) pending_resp <= 1'b0;
        else if (req_active) pending_resp <= 1'b1;
        else if (resp_ok || resp_err) pending_resp <= 1'b0;
    end

    // Dummy ACK propagation for unaligned accesses (which don't go to memory)
    always @(posedge clk or posedge rst_n) begin
        if (rst_n) unaligned_e2_q <= 1'b0;
        else unaligned_e2_q <= unaligned_e1_q && !delay_resp;
    end

    //---------------------------------------------------------
    // Pipeline State
    //---------------------------------------------------------
    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            addr_q <= 0; wdata_q <= 0; rd_q <= 0; wr_q <= 0;
            cacheable_q <= 0; invalidate_q <= 0; writeback_q <= 0; flush_q <= 0;
            unaligned_e1_q <= 0; is_load_q <= 0; is_byte_q <= 0; is_half_q <= 0; is_signed_q <= 0;
        end else if (resp_err || unaligned_e2_q) begin
            // Squash on fault
            addr_q <= 0; wdata_q <= 0; rd_q <= 0; wr_q <= 0;
            cacheable_q <= 0; invalidate_q <= 0; writeback_q <= 0; flush_q <= 0;
            unaligned_e1_q <= 0; is_load_q <= 0; is_byte_q <= 0; is_half_q <= 0; is_signed_q <= 0;
        end else if ((rd_q || |wr_q || unaligned_e1_q) && delay_resp) begin
            // Hold state
        end else if (!((mem_writeback || mem_invalidate || mem_flush || mem_rd || |mem_wr) && !mem_accept)) begin
            // Advance
            addr_q          <= addr_c;
            wdata_q         <= wdata_c;
            rd_q            <= rd_c;
            wr_q            <= wr_c;
            invalidate_q    <= valid & op_c_inv;
            writeback_q     <= valid & op_c_wb;
            flush_q         <= valid & op_c_flush;
            unaligned_e1_q  <= unaligned_c;
            is_load_q       <= valid & op_load;
            is_byte_q       <= is_lb | is_lbu | is_sb;
            is_half_q       <= is_lh | is_lhu | is_sh;
            is_signed_q     <= op_signed;
            
            if (op_c_inv || op_c_wb || op_c_flush) 
                cacheable_q <= 1;
            else 
                cacheable_q <= (addr_c >= MEM_CACHE_ADDR_MIN && addr_c <= MEM_CACHE_ADDR_MAX);
        end
    end

    //---------------------------------------------------------
    // Outputs to Memory
    //---------------------------------------------------------
    assign mem_addr       = {addr_q[31:2], 2'b0};
    assign mem_wdata      = wdata_q;
    assign mem_rd         = rd_q && !delay_resp;
    assign mem_wr         = delay_resp ? 4'b0 : wr_q;
    assign mem_cacheable  = cacheable_q;
    assign mem_req_tag    = 11'b0;
    assign mem_invalidate = invalidate_q;
    assign mem_writeback  = writeback_q;
    assign mem_flush      = flush_q;

    // Stall Logic
    assign stall = ((mem_writeback || mem_invalidate || mem_flush || mem_rd || |mem_wr) && !mem_accept) ||
                   delay_resp || unaligned_e1_q;

    //---------------------------------------------------------
    // FIFO for tracking request attributes
    //---------------------------------------------------------
    wire fifo_pop = mem_ack || unaligned_e2_q;
    wire fifo_push = ((mem_rd || |mem_wr || mem_writeback || mem_invalidate || mem_flush) && mem_accept) || (unaligned_e1_q && !delay_resp);
    
    wire        resp_is_load;
    wire [31:0] resp_addr_full;
    wire        resp_byte, resp_half, resp_signed;

    riscv_lsu_fifo #( .WIDTH(36), .DEPTH(2) ) u_fifo (
        .clk        (clk),
        .rst_n      (rst_n),
        .push       (fifo_push),
        .data_in    ({addr_q, is_signed_q, is_half_q, is_byte_q, is_load_q}),
        .pop        (fifo_pop),
        .data_out   ({resp_addr_full, resp_signed, resp_half, resp_byte, resp_is_load}),
        .accept     (),
        .valid      ()
    );

    //---------------------------------------------------------
    // Load Data Realignment & Writeback
    //---------------------------------------------------------
    reg [31:0] wb_data_calc;
    wire [1:0] addr_lsb = resp_addr_full[1:0];

    always @(*) begin
        wb_data_calc = 32'b0;
        
        // On fault or unaligned, return the bad address
        if ((mem_ack && mem_error) || unaligned_e2_q) begin
            wb_data_calc = resp_addr_full;
        end 
        else if (mem_ack && resp_is_load) begin
            if (resp_byte) begin
                case (addr_lsb)
                    2'b00: wb_data_calc = {24'b0, mem_rdata[7:0]};
                    2'b01: wb_data_calc = {24'b0, mem_rdata[15:8]};
                    2'b10: wb_data_calc = {24'b0, mem_rdata[23:16]};
                    2'b11: wb_data_calc = {24'b0, mem_rdata[31:24]};
                endcase
                if (resp_signed && wb_data_calc[7]) wb_data_calc[31:8] = 24'hFFFFFF;
            end 
            else if (resp_half) begin
                if (addr_lsb[1]) wb_data_calc = {16'b0, mem_rdata[31:16]};
                else             wb_data_calc = {16'b0, mem_rdata[15:0]};
                
                if (resp_signed && wb_data_calc[15]) wb_data_calc[31:16] = 16'hFFFF;
            end 
            else begin
                wb_data_calc = mem_rdata;
            end
        end
    end

    assign wb_valid = mem_ack | unaligned_e2_q;
    assign wb_val   = wb_data_calc;

    // Exception Generation
    wire err_load_align  = unaligned_e2_q & resp_is_load;
    wire err_store_align = unaligned_e2_q & !resp_is_load;
    wire err_load_bus    = mem_error && resp_is_load;
    wire err_store_bus   = mem_error && !resp_is_load;
    
    assign wb_exception = 
        err_load_align  ? `EXCEPTION_MISALIGNED_LOAD :
        err_store_align ? `EXCEPTION_MISALIGNED_STORE :
        (mem_error && mem_load_fault) ? `EXCEPTION_PAGE_FAULT_LOAD :
        (mem_error && mem_store_fault)? `EXCEPTION_PAGE_FAULT_STORE :
        err_load_bus    ? `EXCEPTION_FAULT_LOAD :
        err_store_bus   ? `EXCEPTION_FAULT_STORE :
        `EXCEPTION_W'b0;

endmodule

//=============================================================================
// Helper Module: FIFO
//=============================================================================
module riscv_lsu_fifo
#(
    parameter WIDTH = 36,
    parameter DEPTH = 2
)
(
    input  wire             clk,
    input  wire             rst_n,
    
    input  wire             push,
    input  wire [WIDTH-1:0] data_in,
    output wire             accept,
    
    input  wire             pop,
    output wire [WIDTH-1:0] data_out,
    output wire             valid
);

    localparam ADDR_W = $clog2(DEPTH);
    localparam COUNT_W = ADDR_W + 1;

    reg [WIDTH-1:0]   ram [0:DEPTH-1];
    reg [ADDR_W-1:0]  wr_ptr;
    reg [ADDR_W-1:0]  rd_ptr;
    reg [COUNT_W-1:0] count;
    
    integer i;

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
            for (i=0; i<DEPTH; i=i+1) ram[i] <= 0;
        end else begin
            // Push
            if (push && accept) begin
                ram[wr_ptr] <= data_in;
                wr_ptr      <= (wr_ptr == DEPTH-1) ? 0 : wr_ptr + 1;
            end
            
            // Pop
            if (pop && valid) begin
                rd_ptr      <= (rd_ptr == DEPTH-1) ? 0 : rd_ptr + 1;
            end
            
            // Count tracking
            if ((push && accept) && !(pop && valid))
                count <= count + 1;
            else if (!(push && accept) && (pop && valid))
                count <= count - 1;
        end
    end

    assign valid    = (count != 0);
    assign accept   = (count != DEPTH);
    assign data_out = ram[rd_ptr];

endmodule