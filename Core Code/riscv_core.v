module riscv_core
#(
    parameter SUPPORT_MULDIV   = 1,
    parameter SUPPORT_MMU      = 0,
    parameter MEM_CACHE_ADDR_MIN = 32'h80000000,
    parameter MEM_CACHE_ADDR_MAX = 32'h8fffffff
)
(
    input  wire        clk_i,
    input  wire        rst_i, // Active high reset
    
    // Instruction Memory Interface
    input  wire        mem_i_accept_i,
    input  wire        mem_i_valid_i,
    input  wire        mem_i_error_i,
    input  wire [31:0] mem_i_inst_i,
    output wire        mem_i_rd_o,
    output wire        mem_i_flush_o,
    output wire        mem_i_invalidate_o,
    output wire [31:0] mem_i_pc_o,

    // Data Memory Interface
    input  wire [31:0] mem_d_data_rd_i,
    input  wire        mem_d_accept_i,
    input  wire        mem_d_ack_i,
    input  wire        mem_d_error_i,
    input  wire [10:0] mem_d_resp_tag_i,
    output wire [31:0] mem_d_addr_o,
    output wire [31:0] mem_d_data_wr_o,
    output wire        mem_d_rd_o,
    output wire [3:0]  mem_d_wr_o,
    output wire        mem_d_cacheable_o,
    output wire [10:0] mem_d_req_tag_o,
    output wire        mem_d_invalidate_o,
    output wire        mem_d_writeback_o,
    output wire        mem_d_flush_o,

    // Interrupts
    input  wire        intr_i,
    input  wire [31:0] reset_vector_i,
    input  wire [31:0] cpu_id_i
);

    wire        rst_n = ~rst_i; // Logic uses active low internally usually

    //---------------------------------------------------------
    // Wires
    //---------------------------------------------------------
    wire        fetch_accept;
    wire        fetch_valid;
    wire [31:0] fetch_instr;
    wire [31:0] fetch_pc;
    wire        fetch_fault_fetch;
    wire        fetch_fault_page;

    wire        dec_accept;
    wire        dec_valid;
    wire [31:0] dec_instr;
    wire [31:0] dec_pc;
    wire        dec_fault_fetch;
    wire        dec_fault_page;
    wire        dec_exec;
    wire        dec_lsu;
    wire        dec_branch;
    wire        dec_mul;
    wire        dec_div;
    wire        dec_csr;
    wire        dec_rd_valid;
    wire        dec_invalid;
    wire        squash_decode;

    wire        br_req;
    wire [31:0] br_pc;
    wire [1:0]  br_priv;
    wire        exec_br_req;
    wire        exec_br_taken;
    wire        exec_br_ntaken;
    wire [31:0] exec_br_source;
    wire        exec_br_call;
    wire        exec_br_ret;
    wire        exec_br_jmp;
    wire [31:0] exec_br_pc;
    wire        exec_br_d_req;
    wire [31:0] exec_br_d_pc;
    wire [1:0]  exec_br_d_priv;

    wire        csr_br_req;
    wire [31:0] csr_br_pc;
    wire [1:0]  csr_br_priv;

    wire        wb_mem_valid;
    wire [31:0] wb_mem_val;
    wire [5:0]  wb_mem_exc;
    wire        wb_div_valid;
    wire [31:0] wb_div_val;
    wire [31:0] wb_mul_val;
    wire [31:0] wb_exec_val;

    wire [31:0] csr_res_val;
    wire        csr_res_write;
    wire [31:0] csr_res_wdata;
    wire [5:0]  csr_res_exc;
    
    wire        exec_valid_out;
    wire        lsu_valid_out;
    wire        csr_valid_out;
    wire        mul_valid_out;
    wire        div_valid_out;
    
    wire [31:0] op_instr;
    wire [31:0] op_pc;
    wire        op_invalid;
    wire [4:0]  op_rd, op_ra, op_rb;
    wire [31:0] op_ra_val, op_rb_val;
    
    wire        lsu_stall;
    wire        take_intr;
    wire        exec_hold;
    wire        mul_hold;
    wire        intr_inhibit;

    wire        csr_wb_write;
    wire [11:0] csr_wb_addr;
    wire [31:0] csr_wb_data;
    wire [5:0]  csr_wb_exc;
    wire [31:0] csr_wb_exc_pc;
    wire [31:0] csr_wb_exc_addr;
    
    wire [1:0]  mmu_priv_d;
    wire        mmu_sum, mmu_mxr, mmu_flush;
    wire [31:0] mmu_satp;
    
    // MMU Wires
    wire        if_mmu_accept, if_mmu_valid, if_mmu_error, if_mmu_fault;
    wire [31:0] if_mmu_inst;
    wire        lsu_mmu_accept, lsu_mmu_ack, lsu_mmu_error, lsu_mmu_load_fault, lsu_mmu_store_fault;
    wire [31:0] lsu_mmu_rdata;
    wire [10:0] lsu_mmu_resp_tag;
    
    wire [31:0] lsu_addr_o, lsu_data_wr_o;
    wire        lsu_rd_o, lsu_cacheable_o, lsu_invalidate_o, lsu_wb_o, lsu_flush_o;
    wire [3:0]  lsu_wr_o;
    wire [10:0] lsu_req_tag_o;


    //---------------------------------------------------------
    // Modules
    //---------------------------------------------------------

    riscv_fetch #( .SUPPORT_MMU(SUPPORT_MMU) ) u_fetch (
        .clk        (clk_i),
        .rst_n      (rst_n),
        .accept_in  (dec_accept),
        .valid_out  (fetch_valid),
        .instr_out  (fetch_instr),
        .pc_out     (fetch_pc),
        .fault_fetch_out(fetch_fault_fetch),
        .fault_page_out (fetch_fault_page),
        
        // MMU/Cache Interface
        .mem_accept (if_mmu_accept),
        .mem_valid  (if_mmu_valid),
        .mem_error  (if_mmu_error),
        .mem_inst   (if_mmu_inst),
        .mem_page_fault(if_mmu_fault),
        .mem_rd     (mem_i_rd_o),
        .mem_flush  (mem_i_flush_o),
        .mem_invalidate(mem_i_invalidate_o),
        .mem_pc     (mem_i_pc_o),
        .mem_priv   (/* unused here in simple core */),

        .invalidate_req(1'b0), // Tied off
        .branch_req (br_req),
        .branch_target(br_pc),
        .branch_priv(br_priv),
        .decode_squash(squash_decode)
    );

    riscv_decode #( .SUPPORT_MULDIV(SUPPORT_MULDIV) ) u_decode (
        .clk        (clk_i),
        .rst_n      (rst_n),
        .valid_in   (fetch_valid),
        .instr_in   (fetch_instr),
        .pc_in      (fetch_pc),
        .fault_fetch(fetch_fault_fetch),
        .fault_page (fetch_fault_page),
        .accept_out (fetch_accept),
        .squash     (squash_decode),
        
        .accept_in  (dec_accept),
        .valid_out  (dec_valid),
        .instr_out  (dec_instr),
        .pc_out     (dec_pc),
        .fault_fetch_out(dec_fault_fetch),
        .fault_page_out(dec_fault_page),
        
        .is_exec    (dec_exec),
        .is_lsu     (dec_lsu),
        .is_branch  (dec_branch),
        .is_mul     (dec_mul),
        .is_div     (dec_div),
        .is_csr     (dec_csr),
        .is_rd_valid(dec_rd_valid),
        .is_invalid (dec_invalid)
    );

    riscv_issue #( .SUPPORT_MULDIV(SUPPORT_MULDIV) ) u_issue (
        .clk            (clk_i),
        .rst_n          (rst_n),
        .fetch_valid    (dec_valid),
        .fetch_instr    (dec_instr),
        .fetch_pc       (dec_pc),
        .fetch_fault    (dec_fault_fetch),
        .page_fault     (dec_fault_page),
        .is_exec        (dec_exec),
        .is_lsu         (dec_lsu),
        .is_branch      (dec_branch),
        .is_mul         (dec_mul),
        .is_div         (dec_div),
        .is_csr         (dec_csr),
        .rd_valid       (dec_rd_valid),
        .is_invalid     (dec_invalid),

        .br_exec_req    (exec_br_req),
        .br_taken       (exec_br_taken),
        .br_ntaken      (exec_br_ntaken),
        .br_source      (exec_br_source),
        .br_is_call     (exec_br_call),
        .br_is_ret      (exec_br_ret),
        .br_is_jmp      (exec_br_jmp),
        .br_pc          (exec_br_pc),
        .br_d_req       (exec_br_d_req),
        .br_d_pc        (exec_br_d_pc),
        .br_d_priv      (exec_br_d_priv),

        .br_csr_req     (csr_br_req),
        .br_csr_pc      (csr_br_pc),
        .br_csr_priv    (csr_br_priv),

        .wb_exec_val    (wb_exec_val),
        .wb_mem_valid   (wb_mem_valid),
        .wb_mem_val     (wb_mem_val),
        .wb_mem_exc     (wb_mem_exc),
        .wb_mul_val     (wb_mul_val),
        .wb_div_valid   (wb_div_valid),
        .wb_div_val     (wb_div_val),
        
        .csr_res_val    (csr_res_val),
        .csr_res_write  (csr_res_write),
        .csr_res_wdata  (csr_res_wdata),
        .csr_res_exc    (csr_res_exc),

        .lsu_stall      (lsu_stall),
        .take_intr      (take_intr),

        .fetch_accept   (fetch_accept),
        .br_req_out     (br_req),
        .br_pc_out      (br_pc),
        .br_priv_out    (br_priv),
        
        .valid_exec     (exec_valid_out),
        .valid_lsu      (lsu_valid_out),
        .valid_csr      (csr_valid_out),
        .valid_mul      (mul_valid_out),
        .valid_div      (div_valid_out),
        
        .op_instr       (op_instr),
        .op_pc          (op_pc),
        .op_invalid     (op_invalid),
        .op_rd          (op_rd),
        .op_ra          (op_ra),
        .op_rb          (op_rb),
        .op_ra_val      (op_ra_val),
        .op_rb_val      (op_rb_val),

        .csr_wb_write   (csr_wb_write),
        .csr_wb_addr    (csr_wb_addr),
        .csr_wb_data    (csr_wb_data),
        .csr_wb_exc     (csr_wb_exc),
        .csr_wb_exc_pc  (csr_wb_exc_pc),
        .csr_wb_exc_addr(csr_wb_exc_addr),

        .exec_hold      (exec_hold),
        .mul_hold       (mul_hold),
        .intr_inhibit   (intr_inhibit)
    );

    riscv_exec u_exec (
        .clk        (clk_i),
        .rst_n      (rst_n),
        .valid      (exec_valid_out),
        .instr      (op_instr),
        .pc         (op_pc),
        .invalid    (op_invalid),
        .rd         (op_rd),
        .ra         (op_ra),
        .rb         (op_rb),
        .rdata1     (op_ra_val),
        .rdata2     (op_rb_val),
        .hold       (exec_hold),
        
        .br_req     (exec_br_req),
        .br_taken   (exec_br_taken),
        .br_untaken (exec_br_ntaken),
        .br_source  (exec_br_source),
        .br_is_call (exec_br_call),
        .br_is_ret  (exec_br_ret),
        .br_is_jmp  (exec_br_jmp),
        .br_pc      (exec_br_pc),
        .br_d_req   (exec_br_d_req),
        .br_d_pc    (exec_br_d_pc),
        .br_d_priv  (exec_br_d_priv),
        .result     (wb_exec_val)
    );

    riscv_lsu #( .MEM_CACHE_ADDR_MAX(MEM_CACHE_ADDR_MAX) ) u_lsu (
        .clk            (clk_i),
        .rst_n          (rst_n),
        .opcode_valid   (lsu_valid_out),
        .opcode_instr   (op_instr), // Use generic op bus
        .opcode_pc      (op_pc),
        .opcode_rd_idx  (op_rd),
        .opcode_ra_idx  (op_ra),
        .opcode_rb_idx  (op_rb),
        .opcode_ra_oper (op_ra_val),
        .opcode_rb_oper (op_rb_val),
        
        // MMU Interface
        .mem_data_rd    (lsu_mmu_rdata),
        .mem_accept     (lsu_mmu_accept),
        .mem_ack        (lsu_mmu_ack),
        .mem_error      (lsu_mmu_error),
        .mem_resp_tag   (lsu_mmu_resp_tag),
        .mem_load_fault (lsu_mmu_load_fault),
        .mem_store_fault(lsu_mmu_store_fault),

        .mem_addr_o     (lsu_addr_o),
        .mem_data_wr_o  (lsu_data_wr_o),
        .mem_rd_o       (lsu_rd_o),
        .mem_wr_o       (lsu_wr_o),
        .mem_cacheable_o(lsu_cacheable_o),
        .mem_req_tag_o  (lsu_req_tag_o),
        .mem_invalidate_o(lsu_invalidate_o),
        .mem_writeback_o(lsu_wb_o),
        .mem_flush_o    (lsu_flush_o),
        
        .writeback_valid(wb_mem_valid),
        .writeback_value(wb_mem_val),
        .writeback_exc  (wb_mem_exc),
        .stall_o        (lsu_stall)
    );

    riscv_mmu #( .SUPPORT_MMU(SUPPORT_MMU) ) u_mmu (
        .clk            (clk_i),
        .rst_n          (rst_n),
        .priv_lvl       (mmu_priv_d),
        .sum_en         (mmu_sum),
        .mxr_en         (mmu_mxr),
        .flush_tlb      (mmu_flush),
        .satp_val       (mmu_satp),
        
        // Fetch Side
        .if_read        (1'b1), // Simplified
        .if_flush       (mem_i_flush_o),
        .if_inval       (mem_i_invalidate_o),
        .if_pc          (mem_i_pc_o), // from fetch
        .if_priv        (2'b11), // TODO connect real priv
        .if_accept      (if_mmu_accept),
        .if_valid       (if_mmu_valid),
        .if_error       (if_mmu_error),
        .if_inst        (if_mmu_inst),
        .if_fault       (if_mmu_fault),
        
        .mem_if_accept  (mem_i_accept_i),
        .mem_if_valid   (mem_i_valid_i),
        .mem_if_error   (mem_i_error_i),
        .mem_if_inst    (mem_i_inst_i),
        .mem_if_rd      (), // Outputs to mem...
        .mem_if_flush   (),
        .mem_if_inval   (),
        .mem_if_pc      (), // Output to mem

        // LSU Side
        .ls_addr        (lsu_addr_o),
        .ls_wdata       (lsu_data_wr_o),
        .ls_rd          (lsu_rd_o),
        .ls_wr          (lsu_wr_o),
        .ls_cacheable   (lsu_cacheable_o),
        .ls_req_tag     (lsu_req_tag_o),
        .ls_inval       (lsu_invalidate_o),
        .ls_wb          (lsu_wb_o),
        .ls_flush       (lsu_flush_o),
        
        .ls_rdata       (lsu_mmu_rdata),
        .ls_accept      (lsu_mmu_accept),
        .ls_ack         (lsu_mmu_ack),
        .ls_error       (lsu_mmu_error),
        .ls_resp_tag    (lsu_mmu_resp_tag),
        .ls_load_fault  (lsu_mmu_load_fault),
        .ls_store_fault (lsu_mmu_store_fault),
        
        .mem_ls_rdata   (mem_d_data_rd_i),
        .mem_ls_accept  (mem_d_accept_i),
        .mem_ls_ack     (mem_d_ack_i),
        .mem_ls_error   (mem_d_error_i),
        .mem_ls_resp_tag(mem_d_resp_tag_i),
        
        .mem_ls_addr    (mem_d_addr_o),
        .mem_ls_wdata   (mem_d_data_wr_o),
        .mem_ls_rd      (mem_d_rd_o),
        .mem_ls_wr      (mem_d_wr_o),
        .mem_ls_cacheable(mem_d_cacheable_o),
        .mem_ls_req_tag (mem_d_req_tag_o),
        .mem_ls_inval   (mem_d_invalidate_o),
        .mem_ls_wb      (mem_d_writeback_o),
        .mem_ls_flush   (mem_d_flush_o)
    );

    riscv_csr u_csr (
        .clk            (clk_i),
        .rst_n          (rst_n),
        .intr_req       (intr_i),
        .addr           (op_instr[31:20]),
        .wdata          (op_ra_val),
        .op             (op_instr[14:12]),
        .wen            (csr_valid_out), // simplified enable
        .rdata          (csr_res_val),
        
        // Writeback event for state update
        .exc_req        (|csr_wb_exc),
        .exc_cause      (csr_wb_exc[3:0]),
        .exc_pc         (csr_wb_exc_pc),
        .exc_val        (csr_wb_exc_addr),
        .trap_taken     (take_intr),
        .trap_pc        (csr_br_pc),
        
        .priv_lvl       (mmu_priv_d), // Use as debug/mmu priv
        .mie            (),
        .sie            ()
    );
    
    // Note: MULDIV are instantiated similarly using signals from u_issue.

endmodule