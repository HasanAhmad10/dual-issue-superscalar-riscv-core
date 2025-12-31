module riscv_pipe_ctrl
#(
    parameter SUPPORT_LOAD_BYPASS = 1,
    parameter SUPPORT_MUL_BYPASS  = 1
)
(
    input  wire        clk,
    input  wire        rst_n,

    // Issue Interface
    input  wire        iss_valid,
    input  wire        iss_accept,
    input  wire        iss_stall,
    input  wire        iss_lsu,
    input  wire        iss_csr,
    input  wire        iss_div,
    input  wire        iss_mul,
    input  wire        iss_branch,
    input  wire        iss_rd_valid,
    input  wire [4:0]  iss_rd,
    input  wire [5:0]  iss_exc,
    input  wire        take_intr,
    input  wire        iss_br_taken,
    input  wire [31:0] iss_br_tgt,
    input  wire [31:0] iss_pc,
    input  wire [31:0] iss_instr,
    input  wire [31:0] iss_ra_val,
    input  wire [31:0] iss_rb_val,

    // Execution Stage 1 (ALU) Inputs
    input  wire [31:0] alu_res_e1,
    input  wire [31:0] csr_val_e1,
    input  wire        csr_we_e1,
    input  wire [31:0] csr_wdata_e1,
    input  wire [5:0]  csr_exc_e1,

    // Execution Stage 1 Outputs
    output wire        is_load_e1,
    output wire        is_store_e1,
    output wire        is_mul_e1,
    output wire        is_branch_e1,
    output wire [4:0]  rd_e1,
    output wire [31:0] pc_e1,
    output wire [31:0] opcode_e1,
    output wire [31:0] ra_val_e1,
    output wire [31:0] rb_val_e1,

    // Execution Stage 2 (Mem/Mul) Inputs
    input  wire        mem_complete,
    input  wire [31:0] mem_res_e2,
    input  wire [5:0]  mem_exc_e2,
    input  wire [31:0] mul_res_e2,

    // Execution Stage 2 Outputs
    output wire        is_load_e2,
    output wire        is_mul_e2,
    output wire [4:0]  rd_e2,
    output wire [31:0] res_e2,

    // Out-of-band Results
    input  wire        div_complete,
    input  wire [31:0] div_res,

    // Writeback / Commit Outputs
    output wire        valid_wb,
    output wire        csr_wb,
    output wire [4:0]  rd_wb,
    output wire [31:0] res_wb,
    output wire [31:0] pc_wb,
    output wire [31:0] opcode_wb,
    output wire [31:0] ra_val_wb,
    output wire [31:0] rb_val_wb,
    output wire [5:0]  exc_wb,
    output wire        csr_we_wb,
    output wire [11:0] csr_addr_wb,
    output wire [31:0] csr_wdata_wb,
    output wire [31:0] exc_addr_wb,

    // Flow Control
    output wire        stall_pipe,
    output wire        squash_out,
    input  wire        squash_in,
    input  wire        squash_wb
);

`include "riscv_defs.v"

    // Bit definitions for control vector
    localparam P_ALU      = 0;
    localparam P_LOAD     = 1;
    localparam P_STORE    = 2;
    localparam P_CSR      = 3;
    localparam P_DIV      = 4;
    localparam P_MUL      = 5;
    localparam P_BRANCH   = 6;
    localparam P_RD_VALID = 7;
    localparam P_INTR     = 8;
    localparam P_COMPLETE = 9;
    localparam P_WIDTH    = 10;

    //---------------------------------------------------------
    // Stage 1: Execute / Address Calculation
    //---------------------------------------------------------
    reg                 valid_e1;
    reg [P_WIDTH-1:0]   ctrl_e1;
    reg [31:0]          pc_e1_reg;
    reg [31:0]          npc_e1;
    reg [31:0]          instr_e1;
    reg [31:0]          ra_e1;
    reg [31:0]          rb_e1;
    reg [5:0]           exc_e1;

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            valid_e1    <= 0;
            ctrl_e1     <= 0;
            pc_e1_reg   <= 0;
            npc_e1      <= 0;
            instr_e1    <= 0;
            ra_e1       <= 0;
            rb_e1       <= 0;
            exc_e1      <= 0;
        end else if (iss_stall) begin
            // Hold state
        end else if (iss_valid && iss_accept && !(squash_out || squash_in)) begin
            valid_e1    <= 1;
            
            ctrl_e1[P_ALU]      <= ~(iss_lsu | iss_csr | iss_div | iss_mul);
            ctrl_e1[P_LOAD]     <= iss_lsu & iss_rd_valid & ~take_intr;
            ctrl_e1[P_STORE]    <= iss_lsu & ~iss_rd_valid & ~take_intr;
            ctrl_e1[P_CSR]      <= iss_csr & ~take_intr;
            ctrl_e1[P_DIV]      <= iss_div & ~take_intr;
            ctrl_e1[P_MUL]      <= iss_mul & ~take_intr;
            ctrl_e1[P_BRANCH]   <= iss_branch & ~take_intr;
            ctrl_e1[P_RD_VALID] <= iss_rd_valid & ~take_intr;
            ctrl_e1[P_INTR]     <= take_intr;
            ctrl_e1[P_COMPLETE] <= 1;

            pc_e1_reg   <= iss_pc;
            npc_e1      <= iss_br_taken ? iss_br_tgt : (iss_pc + 4);
            instr_e1    <= iss_instr;
            ra_e1       <= iss_ra_val;
            rb_e1       <= iss_rb_val;
            
            if (|iss_exc) exc_e1 <= iss_exc;
            else if (iss_br_taken && iss_br_tgt[1:0] != 0) exc_e1 <= `EXCEPTION_MISALIGNED_FETCH;
            else exc_e1 <= 0;

        end else begin
            valid_e1    <= 0;
            ctrl_e1     <= 0;
            exc_e1      <= 0;
        end
    end

    assign is_load_e1   = ctrl_e1[P_LOAD];
    assign is_store_e1  = ctrl_e1[P_STORE];
    assign is_mul_e1    = ctrl_e1[P_MUL];
    assign is_branch_e1 = ctrl_e1[P_BRANCH];
    assign rd_e1        = ctrl_e1[P_RD_VALID] ? instr_e1[11:7] : 5'b0;
    assign pc_e1        = pc_e1_reg;
    assign opcode_e1    = instr_e1;
    assign ra_val_e1    = ra_e1;
    assign rb_val_e1    = rb_e1;

    //---------------------------------------------------------
    // Stage 2: Memory / Multiply Completion
    //---------------------------------------------------------
    reg                 valid_e2;
    reg [P_WIDTH-1:0]   ctrl_e2;
    reg                 csr_we_e2;
    reg [31:0]          csr_wdata_e2;
    reg [31:0]          res_e2_reg;
    reg [31:0]          pc_e2;
    reg [31:0]          npc_e2;
    reg [31:0]          instr_e2;
    reg [31:0]          ra_e2;
    reg [31:0]          rb_e2;
    reg [5:0]           exc_e2;

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            valid_e2 <= 0;
            ctrl_e2 <= 0;
            exc_e2 <= 0;
        end else if (iss_stall) begin
            // Hold
        end else if (squash_out || squash_in) begin
            valid_e2 <= 0;
            ctrl_e2 <= 0;
            exc_e2 <= 0;
        end else begin
            valid_e2    <= valid_e1;
            ctrl_e2     <= ctrl_e1;
            csr_we_e2   <= csr_we_e1;
            csr_wdata_e2<= csr_wdata_e1;
            pc_e2       <= pc_e1_reg;
            npc_e2      <= npc_e1;
            instr_e2    <= instr_e1;
            ra_e2       <= ra_e1;
            rb_e2       <= rb_e1;

            if (ctrl_e1[P_INTR]) exc_e2 <= `EXCEPTION_INTERRUPT;
            else if (|exc_e1) begin
                valid_e2 <= 0; // Drop valid, handle exception
                exc_e2   <= exc_e1;
            end else begin
                exc_e2   <= csr_exc_e1;
            end

            // Result Muxing for E2 register
            if (ctrl_e1[P_DIV])      res_e2_reg <= div_res;
            else if (ctrl_e1[P_CSR]) res_e2_reg <= csr_val_e1;
            else                     res_e2_reg <= alu_res_e1;
        end
    end

    // Result Selection (Bypass logic needs this combinatorial)
    reg [31:0] final_res_e2;
    wire valid_e2_active = valid_e2 && !iss_stall;

    always @(*) begin
        final_res_e2 = res_e2_reg;
        if (SUPPORT_LOAD_BYPASS && valid_e2_active && (ctrl_e2[P_LOAD] || ctrl_e2[P_STORE]))
            final_res_e2 = mem_res_e2;
        else if (SUPPORT_MUL_BYPASS && valid_e2_active && ctrl_e2[P_MUL])
            final_res_e2 = mul_res_e2;
    end

    assign is_load_e2 = ctrl_e2[P_LOAD];
    assign is_mul_e2  = ctrl_e2[P_MUL];
    assign rd_e2      = (valid_e2_active && ctrl_e2[P_RD_VALID]) ? instr_e2[11:7] : 5'b0;
    assign res_e2     = final_res_e2;

    // Stall Generation
    wire stall_div = ctrl_e1[P_DIV] && !div_complete;
    wire stall_mem = (ctrl_e2[P_LOAD] || ctrl_e2[P_STORE]) && !mem_complete;
    
    assign stall_pipe = stall_div || stall_mem;

    // Exception handling at E2
    reg [5:0] final_exc_e2;
    always @(*) begin
        if (valid_e2 && (ctrl_e2[P_LOAD] || ctrl_e2[P_STORE]) && mem_complete)
            final_exc_e2 = mem_exc_e2;
        else
            final_exc_e2 = exc_e2;
    end

    assign squash_out = (|final_exc_e2);

    //---------------------------------------------------------
    // Writeback Stage
    //---------------------------------------------------------
    reg                 valid_wb_reg;
    reg [P_WIDTH-1:0]   ctrl_wb;
    reg                 csr_we_wb_reg;
    reg [31:0]          csr_wdata_wb_reg;
    reg [31:0]          res_wb_reg;
    reg [31:0]          pc_wb_reg;
    reg [31:0]          npc_wb;
    reg [31:0]          instr_wb;
    reg [31:0]          ra_wb;
    reg [31:0]          rb_wb;
    reg [5:0]           exc_wb_reg;

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            valid_wb_reg <= 0;
            exc_wb_reg   <= 0;
            ctrl_wb      <= 0;
        end else if (iss_stall) begin
            // Hold
        end else if (squash_wb) begin
            valid_wb_reg <= 0;
            exc_wb_reg   <= 0;
            ctrl_wb      <= 0;
        end else begin
            // If exception at E2, kill writeback valid
            if (|final_exc_e2) begin
                valid_wb_reg <= 0;
                // But keep control info minus register write
                ctrl_wb      <= ctrl_e2 & ~(1 << P_RD_VALID);
            end else begin
                valid_wb_reg <= valid_e2;
                ctrl_wb      <= ctrl_e2;
            end
            
            csr_we_wb_reg    <= csr_we_e2;
            csr_wdata_wb_reg <= csr_wdata_e2;
            exc_wb_reg       <= final_exc_e2;
            pc_wb_reg        <= pc_e2;
            npc_wb           <= npc_e2;
            instr_wb         <= instr_e2;
            ra_wb            <= ra_e2;
            rb_wb            <= rb_e2;

            if (valid_e2_active && (ctrl_e2[P_LOAD] || ctrl_e2[P_STORE]))
                res_wb_reg <= mem_res_e2;
            else if (valid_e2_active && ctrl_e2[P_MUL])
                res_wb_reg <= mul_res_e2;
            else
                res_wb_reg <= res_e2_reg;
        end
    end

    assign valid_wb     = valid_wb_reg && !iss_stall;
    assign csr_wb       = ctrl_wb[P_CSR] && !iss_stall;
    assign rd_wb        = (valid_wb && ctrl_wb[P_RD_VALID]) ? instr_wb[11:7] : 5'b0;
    assign res_wb       = res_wb_reg;
    assign pc_wb        = pc_wb_reg;
    assign opcode_wb    = instr_wb;
    assign ra_val_wb    = ra_wb;
    assign rb_val_wb    = rb_wb;
    assign exc_wb       = exc_wb_reg;
    assign csr_we_wb    = csr_we_wb_reg;
    assign csr_addr_wb  = instr_wb[31:20];
    assign csr_wdata_wb = csr_wdata_wb_reg;
    
    // Exception address defaults to result (badaddr) or PC
    assign exc_addr_wb  = res_wb_reg;

endmodule