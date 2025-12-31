module riscv_issue
#(
    parameter SUPPORT_MULDIV = 1,
    parameter SUPPORT_LOAD_BYPASS = 1,
    parameter SUPPORT_MUL_BYPASS = 1
)
(
    input  wire        clk,
    input  wire        rst_n,

    // Fetch / Decode Input
    input  wire        fetch_valid,
    input  wire [31:0] fetch_instr,
    input  wire [31:0] fetch_pc,
    input  wire        fetch_fault,
    input  wire        page_fault,
    
    // Instruction Type Flags
    input  wire        is_exec,
    input  wire        is_lsu,
    input  wire        is_branch,
    input  wire        is_mul,
    input  wire        is_div,
    input  wire        is_csr,
    input  wire        rd_valid,
    input  wire        is_invalid,

    // Feedback from Exec
    input  wire        br_exec_req,
    input  wire        br_taken,
    input  wire        br_ntaken,
    input  wire [31:0] br_source,
    input  wire        br_is_call,
    input  wire        br_is_ret,
    input  wire        br_is_jmp,
    input  wire [31:0] br_pc,
    
    // Delayed feedback
    input  wire        br_d_req,
    input  wire [31:0] br_d_pc,
    input  wire [1:0]  br_d_priv,

    // CSR Branching
    input  wire        br_csr_req,
    input  wire [31:0] br_csr_pc,
    input  wire [1:0]  br_csr_priv,

    // Writeback Values (Forwarding)
    input  wire [31:0] wb_exec_val,
    input  wire        wb_mem_valid,
    input  wire [31:0] wb_mem_val,
    input  wire [5:0]  wb_mem_exc,
    input  wire [31:0] wb_mul_val,
    input  wire        wb_div_valid,
    input  wire [31:0] wb_div_val,
    
    // CSR Results
    input  wire [31:0] csr_res_val,
    input  wire        csr_res_write,
    input  wire [31:0] csr_res_wdata,
    input  wire [5:0]  csr_res_exc,

    input  wire        lsu_stall,
    input  wire        take_intr,

    // Outputs to Execution Units
    output wire        fetch_accept,
    output wire        br_req_out,
    output wire [31:0] br_pc_out,
    output wire [1:0]  br_priv_out,
    
    output wire        valid_exec,
    output wire        valid_lsu,
    output wire        valid_csr,
    output wire        valid_mul,
    output wire        valid_div,
    
    output wire [31:0] op_instr,
    output wire [31:0] op_pc,
    output wire        op_invalid,
    output wire [4:0]  op_rd,
    output wire [4:0]  op_ra,
    output wire [4:0]  op_rb,
    output wire [31:0] op_ra_val,
    output wire [31:0] op_rb_val,
    
    // Specific LSU outputs (duplicates for cleanliness)
    output wire [31:0] lsu_instr,
    output wire [31:0] lsu_pc,
    output wire        lsu_invalid,
    output wire [4:0]  lsu_rd,
    output wire [4:0]  lsu_ra,
    output wire [4:0]  lsu_rb,
    output wire [31:0] lsu_ra_val,
    output wire [31:0] lsu_rb_val,

    // CSR Writeback interface
    output wire        csr_wb_write,
    output wire [11:0] csr_wb_addr,
    output wire [31:0] csr_wb_data,
    output wire [5:0]  csr_wb_exc,
    output wire [31:0] csr_wb_exc_pc,
    output wire [31:0] csr_wb_exc_addr,

    output wire        exec_hold,
    output wire        mul_hold,
    output wire        intr_inhibit
);

`include "riscv_defs.v"

    //---------------------------------------------------------
    // Priv Mode Tracking
    //---------------------------------------------------------
    reg [1:0] current_priv;
    always @(posedge clk or posedge rst_n)
        if (rst_n) current_priv <= `PRIV_MACHINE;
        else if (br_csr_req) current_priv <= br_csr_priv;

    //---------------------------------------------------------
    // Issue Logic
    //---------------------------------------------------------
    wire squash_pipe;
    wire pipe_stall;
    wire issue_allowed = fetch_valid & ~squash_pipe & ~br_csr_req;

    // Decoder Aliases
    wire [4:0] dec_ra = fetch_instr[19:15];
    wire [4:0] dec_rb = fetch_instr[24:20];
    wire [4:0] dec_rd = fetch_instr[11:7];

    reg issue_go;
    reg accept_go;
    reg [31:0] scoreboard; // 1 bit per register

    // Pending Long Latency Ops
    reg div_busy;
    reg csr_busy;

    always @(posedge clk or posedge rst_n)
        if (rst_n) div_busy <= 0;
        else if (squash_pipe) div_busy <= 0;
        else if (valid_div && is_div) div_busy <= 1;
        else if (wb_div_valid) div_busy <= 0;

    always @(posedge clk or posedge rst_n)
        if (rst_n) csr_busy <= 0;
        else if (squash_pipe) csr_busy <= 0;
        else if (valid_csr && is_csr) csr_busy <= 1;
        // CSRs complete in pipe_ctrl writeback
        else if (csr_wb_write) csr_busy <= 0; 

    // Pipeline Control Instantiation will drive these wires
    wire p_load_e1, p_store_e1, p_mul_e1;
    wire [4:0] p_rd_e1;
    wire p_load_e2, p_mul_e2;
    wire [4:0] p_rd_e2;
    wire [4:0] p_rd_wb;
    wire [31:0] p_res_wb;
    wire [31:0] p_res_e2;

    // Scoreboard Logic
    always @(*) begin
        issue_go  = 0;
        accept_go = 0;
        scoreboard = 32'b0;

        // Mark registers busy if they are destinations in the pipeline
        if (!SUPPORT_LOAD_BYPASS && p_load_e2) scoreboard[p_rd_e2] = 1;
        if (!SUPPORT_MUL_BYPASS  && p_mul_e2)  scoreboard[p_rd_e2] = 1;
        
        if (p_load_e1 || p_mul_e1) scoreboard[p_rd_e1] = 1;

        // RAW Hazard on Load/Store
        // If E1 is a load/store, we can't issue a MUL/DIV/CSR immediately after
        // (Simplified hazard logic from original)
        if ((p_load_e1 || p_store_e1) && (is_mul || is_div || is_csr))
            scoreboard = 32'hFFFFFFFF; // Block all

        // Stall conditions
        if (lsu_stall || pipe_stall || div_busy || csr_busy) begin
            // Do nothing
        end 
        else if (issue_allowed &&
                 !scoreboard[dec_ra] && 
                 !scoreboard[dec_rb] && 
                 !scoreboard[dec_rd]) 
        begin
            issue_go  = 1;
            accept_go = 1;
            // Mark destination as busy for next cycle check
            if (rd_valid && |dec_rd) scoreboard[dec_rd] = 1;
        end
    end

    // Outputs
    assign valid_lsu = issue_go & ~take_intr;
    assign valid_exec = issue_go;
    assign valid_mul  = SUPPORT_MULDIV & issue_go;
    assign valid_div  = SUPPORT_MULDIV & issue_go;
    assign valid_csr  = issue_go & ~take_intr;
    
    assign fetch_accept = issue_allowed ? (accept_go & ~take_intr) : 1'b1;
    assign intr_inhibit = csr_busy || is_csr;

    // Branch Outputs
    assign br_req_out  = br_csr_req | br_d_req;
    assign br_pc_out   = br_csr_req ? br_csr_pc : br_d_pc;
    assign br_priv_out = br_csr_req ? br_csr_priv : current_priv;

    // Operand Fetching (Register File + Bypassing)
    wire [31:0] rf_rdata1, rf_rdata2;
    
    riscv_regfile u_rf (
        .clk      (clk),
        .rst_n    (rst_n),
        .rs1_addr (dec_ra),
        .rs1_data (rf_rdata1),
        .rs2_addr (dec_rb),
        .rs2_data (rf_rdata2),
        .rd_addr  (p_rd_wb),
        .rd_data  (p_res_wb),
        .wr_en    (|p_rd_wb) // Implicit write enable if RD != 0 logic in regfile
    );

    reg [31:0] op_a, op_b;
    
    always @(*) begin
        op_a = rf_rdata1;
        op_b = rf_rdata2;

        // Forwarding Priority: WB -> E2 -> E1
        // Writeback Stage
        if (p_rd_wb == dec_ra) op_a = p_res_wb;
        if (p_rd_wb == dec_rb) op_b = p_res_wb;

        // E2 Stage
        if (p_rd_e2 == dec_ra) op_a = p_res_e2;
        if (p_rd_e2 == dec_rb) op_b = p_res_e2;

        // E1 Stage (ALU result available)
        if (p_rd_e1 == dec_ra) op_a = wb_exec_val;
        if (p_rd_e1 == dec_rb) op_b = wb_exec_val;

        // Zero Register
        if (dec_ra == 0) op_a = 0;
        if (dec_rb == 0) op_b = 0;
    end

    // Assignment to outputs
    assign op_instr = fetch_instr;
    assign op_pc    = fetch_pc;
    assign op_rd    = dec_rd;
    assign op_ra    = dec_ra;
    assign op_rb    = dec_rb;
    assign op_ra_val= op_a;
    assign op_rb_val= op_b;
    assign op_invalid = 0;

    // LSU duplicates
    assign lsu_instr = fetch_instr;
    assign lsu_pc    = fetch_pc;
    assign lsu_rd    = dec_rd;
    assign lsu_ra    = dec_ra;
    assign lsu_rb    = dec_rb;
    assign lsu_ra_val= op_a;
    assign lsu_rb_val= op_b;
    assign lsu_invalid = 0;

    //---------------------------------------------------------
    // Pipeline Controller Instance
    //---------------------------------------------------------
    // Mapping exceptions
    wire [5:0] issue_exc = fetch_fault ? `EXCEPTION_FAULT_FETCH :
                           page_fault  ? `EXCEPTION_PAGE_FAULT_INST : 6'b0;

    riscv_pipe_ctrl u_pipe_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        
        .issue_valid    (issue_go),
        .issue_stall    (pipe_stall),
        .issue_lsu      (is_lsu),
        .issue_csr      (is_csr),
        .issue_div      (is_div),
        .issue_mul      (is_mul),
        .issue_branch   (is_branch),
        .issue_rd_valid (rd_valid),
        .issue_rd       (dec_rd),
        .issue_pc       (fetch_pc),
        .issue_instr    (fetch_instr),
        .issue_exc      (issue_exc),
        .take_intr      (take_intr),
        .branch_taken   (br_d_req),
        .branch_target  (br_d_pc),

        // E1 Inputs
        .alu_res_e1     (wb_exec_val),
        .csr_val_e1     (csr_res_val),
        .csr_we_e1      (csr_res_write),
        .csr_wdata_e1   (csr_res_wdata),
        .csr_exc_e1     (csr_res_exc),

        // E1 Outputs (Status)
        .is_load_e1     (p_load_e1),
        .is_store_e1    (p_store_e1),
        .is_mul_e1      (p_mul_e1),
        .rd_e1          (p_rd_e1),

        // E2 Inputs
        .mem_complete   (wb_mem_valid),
        .mem_res_e2     (wb_mem_val),
        .mem_exc_e2     (wb_mem_exc),
        .mul_res_e2     (wb_mul_val),

        // E2 Outputs
        .is_load_e2     (p_load_e2),
        .is_mul_e2      (p_mul_e2),
        .rd_e2          (p_rd_e2),
        .res_e2         (p_res_e2),

        // Control
        .stall_pipe     (pipe_stall),
        .squash_out     (squash_pipe),
        
        // WB Outputs
        .rd_wb          (p_rd_wb),
        .res_wb         (p_res_wb),
        .exc_wb         (csr_wb_exc),
        .pc_wb          (csr_wb_exc_pc),
        .csr_we_wb      (csr_wb_write),
        .csr_addr_wb    (csr_wb_addr),
        .csr_wdata_wb   (csr_wb_data),
        
        // Exception Address (Usually ALU result or BadAddr)
        .exc_addr_wb    (csr_wb_exc_addr)
    );

    assign exec_hold = pipe_stall;
    assign mul_hold  = pipe_stall;

endmodule