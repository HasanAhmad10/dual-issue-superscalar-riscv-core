module riscv_mmu
#(
    parameter ADDR_MIN  = 32'h80000000,
    parameter ADDR_MAX  = 32'h8fffffff,
    parameter ENABLE_MMU = 1
)
(
    input  wire        clk,
    input  wire        rst_n,
    
    // MMU Config
    input  wire [1:0]  priv_lvl,
    input  wire        sum_en,
    input  wire        mxr_en,
    input  wire        flush_tlb,
    input  wire [31:0] satp_val,

    // Instruction Fetch Interface
    input  wire        if_read,
    input  wire        if_flush,
    input  wire        if_inval,
    input  wire [31:0] if_pc,
    input  wire [1:0]  if_priv,
    output wire        if_accept,
    output wire        if_valid,
    output wire        if_error,
    output wire [31:0] if_inst,
    output wire        if_fault,
    
    // Fetch -> Memory Out
    input  wire        mem_if_accept,
    input  wire        mem_if_valid,
    input  wire        mem_if_error,
    input  wire [31:0] mem_if_inst,

    output wire        mem_if_rd,
    output wire        mem_if_flush,
    output wire        mem_if_inval,
    output wire [31:0] mem_if_pc,

    // Data Load/Store Interface
    input  wire [31:0] ls_addr,
    input  wire [31:0] ls_wdata,
    input  wire        ls_rd,
    input  wire [3:0]  ls_wr,
    input  wire        ls_cacheable,
    input  wire [10:0] ls_req_tag,
    input  wire        ls_inval,
    input  wire        ls_wb,
    input  wire        ls_flush,
    
    output wire [31:0] ls_rdata,
    output wire        ls_accept,
    output wire        ls_ack,
    output wire        ls_error,
    output wire [10:0] ls_resp_tag,
    output wire        ls_load_fault,
    output wire        ls_store_fault,

    // LSU -> Memory Out
    input  wire [31:0] mem_ls_rdata,
    input  wire        mem_ls_accept,
    input  wire        mem_ls_ack,
    input  wire        mem_ls_error,
    input  wire [10:0] mem_ls_resp_tag,

    output wire [31:0] mem_ls_addr,
    output wire [31:0] mem_ls_wdata,
    output wire        mem_ls_rd,
    output wire [3:0]  mem_ls_wr,
    output wire        mem_ls_cacheable,
    output wire [10:0] mem_ls_req_tag,
    output wire        mem_ls_inval,
    output wire        mem_ls_wb,
    output wire        mem_ls_flush
);

`include "riscv_defs.v"

localparam S_IDLE   = 0;
localparam S_LEVEL1 = 1;
localparam S_LEVEL2 = 2;
localparam S_UPDATE = 3;

generate
if (ENABLE_MMU) begin : MMU_LOGIC

    // State Machine
    reg [1:0] state;
    wire      is_idle = (state == S_IDLE);

    // Reserved tag for hardware page table walks
    wire        is_mmu_resp = (mem_ls_resp_tag[9:7] == 3'b111);
    wire        resp_valid  = is_mmu_resp & mem_ls_ack;
    wire        resp_err    = is_mmu_resp & mem_ls_error;
    wire [31:0] resp_data   = mem_ls_rdata;
    wire        cpu_accept;

    // Latch Load/Store requests
    reg       latched_load;
    reg [3:0] latched_store;
    reg [31:0] latched_addr;

    always @ (posedge clk or posedge rst_n)
    if (rst_n) begin
        latched_load <= 1'b0;
        latched_store <= 4'b0;
        latched_addr <= 32'b0;
    end else begin
        if (ls_rd) latched_load <= ~ls_accept;
        
        if (|ls_wr) 
            latched_store <= ls_accept ? 4'b0 : ls_wr;
        
        if ((ls_rd | latched_load) || (|ls_wr) || (|latched_store))
            if (ls_rd || |ls_wr) latched_addr <= ls_addr;
    end

    wire       req_load  = ls_rd | latched_load;
    wire [3:0] req_store = ls_wr | latched_store;
    wire [31:0] req_addr = (req_load || (|req_store)) ? ls_addr : latched_addr;

    // Page Table Walk Logic
    wire        itlb_hit, dtlb_hit;
    reg         walking_dtlb; // 1 = D-side walk, 0 = I-side walk

    wire        mode_en     = satp_val[`SATP_MODE_R];
    wire [31:0] pt_base     = {satp_val[`SATP_PPN_R], 12'b0};
    wire        virt_i      = (if_priv != `PRIV_MACHINE);
    wire        virt_d      = (priv_lvl != `PRIV_MACHINE);
    wire        super_i     = (if_priv == `PRIV_SUPER);
    wire        super_d     = (priv_lvl == `PRIV_SUPER);
    
    wire        trans_i     = virt_i;
    wire        trans_d     = (mode_en & virt_d);

    wire        itlb_miss   = if_read & trans_i & ~itlb_hit;
    wire        dtlb_miss   = (req_load || (|req_store)) & trans_d & ~dtlb_hit;

    wire [31:0] walk_addr   = is_idle ? (dtlb_miss ? req_addr : if_pc) :
                              walking_dtlb ? req_addr : if_pc;

    reg [31:0]  pte_base;
    reg [31:0]  pte_cached;
    reg [31:0]  vaddr_captured;

    wire [31:0] pte_ppn     = {`PAGE_PFN_SHIFT'b0, resp_data[31:`PAGE_PFN_SHIFT]};
    wire [9:0]  pte_flags   = resp_data[9:0];

    always @ (posedge clk or posedge rst_n)
    if (rst_n) begin
        pte_base    <= 32'b0;
        pte_cached  <= 32'b0;
        vaddr_captured <= 32'b0;
        walking_dtlb <= 1'b0;
        state       <= S_IDLE;
    end else begin
        case (state)
        S_IDLE: begin
            if (itlb_miss || dtlb_miss) begin
                pte_base    <= pt_base + {20'b0, walk_addr[31:22], 2'b0};
                vaddr_captured <= walk_addr;
                walking_dtlb <= dtlb_miss; // D-side priority
                state       <= S_LEVEL1;
            end
        end
        
        S_LEVEL1: begin
            if (resp_valid) begin
                if (resp_err || !resp_data[`PAGE_PRESENT]) begin
                    pte_cached <= 32'b0; // Fault
                    state      <= S_UPDATE;
                end else if (!(resp_data[`PAGE_READ] || resp_data[`PAGE_WRITE] || resp_data[`PAGE_EXEC])) begin
                    // Pointer to next level
                    pte_base <= {resp_data[29:10], 12'b0} + {20'b0, walk_addr[21:12], 2'b0};
                    state    <= S_LEVEL2;
                end else begin
                    // Megapage
                    pte_cached <= ((pte_ppn | {22'b0, walk_addr[21:12]}) << `MMU_PGSHIFT) | {22'b0, pte_flags};
                    state      <= S_UPDATE;
                end
            end
        end

        S_LEVEL2: begin
            if (resp_valid) begin
                if (resp_data[`PAGE_PRESENT]) begin
                    pte_cached <= (pte_ppn << `MMU_PGSHIFT) | {22'b0, pte_flags};
                end else begin
                    pte_cached <= 32'b0;
                end
                state <= S_UPDATE;
            end
        end

        S_UPDATE: state <= S_IDLE;
        endcase
    end

    // ITLB Logic
    reg         itlb_valid;
    reg [31:12] itlb_tag;
    reg [31:0]  itlb_data;

    always @ (posedge clk or posedge rst_n)
    if (rst_n) itlb_valid <= 1'b0;
    else if (flush_tlb) itlb_valid <= 1'b0;
    else if (state == S_UPDATE && !walking_dtlb)
        itlb_valid <= (itlb_tag == if_pc[31:12]);
    else if (!is_idle && !walking_dtlb)
        itlb_valid <= 1'b0;

    always @ (posedge clk) 
    if (state == S_UPDATE && !walking_dtlb) begin
        itlb_tag  <= vaddr_captured[31:12];
        itlb_data <= pte_cached;
    end

    assign itlb_hit = if_read & itlb_valid & (itlb_tag == if_pc[31:12]);

    // ITLB Permissions
    reg perm_fault_i;
    always @(*) begin
        perm_fault_i = 0;
        if (trans_i && itlb_hit) begin
            if (super_i) begin
                if (itlb_data[`PAGE_USER]) perm_fault_i = 1;
                else perm_fault_i = ~itlb_data[`PAGE_EXEC];
            end else begin
                perm_fault_i = ~itlb_data[`PAGE_EXEC] | ~itlb_data[`PAGE_USER];
            end
        end
    end

    assign mem_if_rd      = (~trans_i & if_read) || (itlb_hit & ~perm_fault_i);
    assign mem_if_pc      = trans_i ? {itlb_data[31:12], if_pc[11:0]} : if_pc;
    assign mem_if_flush   = if_flush;
    assign mem_if_inval   = if_inval;

    assign if_accept      = (~trans_i & mem_if_accept) | (trans_i & itlb_hit & mem_if_accept) | perm_fault_i;
    assign if_valid       = mem_if_valid | perm_fault_i;
    assign if_error       = mem_if_valid & mem_if_error;
    assign if_fault       = perm_fault_i;
    assign if_inst        = mem_if_inst;

    // DTLB Logic
    reg         dtlb_valid;
    reg [31:12] dtlb_tag;
    reg [31:0]  dtlb_data;

    always @ (posedge clk or posedge rst_n)
    if (rst_n) dtlb_valid <= 1'b0;
    else if (flush_tlb) dtlb_valid <= 1'b0;
    else if (state == S_UPDATE && walking_dtlb) dtlb_valid <= 1'b1;

    always @ (posedge clk)
    if (state == S_UPDATE && walking_dtlb) begin
        dtlb_tag  <= vaddr_captured[31:12];
        dtlb_data <= pte_cached;
    end

    assign dtlb_hit = dtlb_valid & (dtlb_tag == req_addr[31:12]);

    // DTLB Permissions
    reg fault_load, fault_store;
    
    always @(*) begin
        fault_load = 0;
        if (trans_d && req_load && dtlb_hit) begin
            if (super_d) begin
                if (dtlb_data[`PAGE_USER] && !sum_en) fault_load = 1;
                else fault_load = ~(dtlb_data[`PAGE_READ] | (mxr_en & dtlb_data[`PAGE_EXEC]));
            end else begin
                fault_load = ~dtlb_data[`PAGE_READ] | ~dtlb_data[`PAGE_USER];
            end
        end
    end

    always @(*) begin
        fault_store = 0;
        if (trans_d && (|req_store) && dtlb_hit) begin
            if (super_d) begin
                if (dtlb_data[`PAGE_USER] && !sum_en) fault_store = 1;
                else fault_store = ~(dtlb_data[`PAGE_READ] | ~dtlb_data[`PAGE_WRITE]);
            end else begin
                fault_store = ~dtlb_data[`PAGE_READ] | ~dtlb_data[`PAGE_WRITE] | ~dtlb_data[`PAGE_USER];
            end
        end
    end

    reg r_fault_st, r_fault_ld;
    always @(posedge clk or posedge rst_n)
    if (rst_n) begin r_fault_st <= 0; r_fault_ld <= 0; end
    else begin r_fault_st <= fault_store; r_fault_ld <= fault_load; end

    // Output Muxing (Translation vs Direct)
    assign mem_ls_rd      = trans_d ? (req_load & dtlb_hit & ~fault_load) : ls_rd;
    assign mem_ls_wr      = trans_d ? (req_store & {4{dtlb_hit & ~fault_store}}) : ls_wr;
    assign mem_ls_addr    = trans_d ? {dtlb_data[31:12], req_addr[11:0]} : req_addr;
    assign mem_ls_wdata   = ls_wdata;
    assign mem_ls_inval   = ls_inval;
    assign mem_ls_wb      = ls_wb;
    assign mem_ls_flush   = ls_flush;
    
    reg cacheable_buff;
    always @(*) begin
        if (ls_inval | ls_wb | ls_flush) cacheable_buff = 1;
        else cacheable_buff = (mem_ls_addr >= ADDR_MIN && mem_ls_addr <= ADDR_MAX);
    end
    
    assign ls_ack         = (mem_ls_ack & ~is_mmu_resp) | r_fault_st | r_fault_ld;
    assign ls_resp_tag    = mem_ls_resp_tag;
    assign ls_error       = (mem_ls_error & ~is_mmu_resp) | r_fault_st | r_fault_ld;
    assign ls_rdata       = mem_ls_rdata;
    assign ls_store_fault = r_fault_st;
    assign ls_load_fault  = r_fault_ld;
    assign ls_accept      = (~trans_d & cpu_accept) | (trans_d & dtlb_hit & cpu_accept) | fault_store | fault_load;

    // Page Table Walk Arbitration
    reg req_active;
    always @ (posedge clk or posedge rst_n)
    if (rst_n) req_active <= 0;
    else if (is_idle && (itlb_miss || dtlb_miss)) req_active <= 1;
    else if (state == S_LEVEL1 && resp_valid && !resp_err && resp_data[`PAGE_PRESENT] && !resp_data[`PAGE_READ]) req_active <= 1;
    else if (cpu_accept) req_active <= 0;

    reg hold_read;
    reg use_mmu_src;
    wire mmu_controls_bus = hold_read ? use_mmu_src : req_active;

    always @ (posedge clk or posedge rst_n)
    if (rst_n) begin hold_read <= 0; use_mmu_src <= 0; end
    else if ((mem_ls_rd || |mem_ls_wr) && !mem_ls_accept) begin
        hold_read <= 1;
        use_mmu_src <= mmu_controls_bus;
    end else if (mem_ls_accept) hold_read <= 0;

    // Bus Muxing (MMU walker vs LSU)
    assign ls_accept      = mmu_controls_bus & mem_ls_accept; // Typo fix in logic: MMU steals bus
    assign cpu_accept     = ~mmu_controls_bus & mem_ls_accept;

    assign mem_ls_rd      = mmu_controls_bus ? req_active : (trans_d ? (req_load & dtlb_hit & ~fault_load) : ls_rd);
    // ... (rest of muxing logic inferred for brevity in reconstruction)

end else begin : NO_MMU
    // Pass-through logic
    assign mem_if_rd = if_read;
    assign mem_if_pc = if_pc;
    assign mem_if_flush = if_flush;
    assign mem_if_inval = if_inval;
    assign if_accept = mem_if_accept;
    assign if_valid = mem_if_valid;
    assign if_error = mem_if_error;
    assign if_fault = 0;
    assign if_inst = mem_if_inst;

    assign mem_ls_rd = ls_rd;
    assign mem_ls_wr = ls_wr;
    assign mem_ls_addr = ls_addr;
    assign mem_ls_wdata = ls_wdata;
    assign mem_ls_inval = ls_inval;
    assign mem_ls_wb = ls_wb;
    assign mem_ls_flush = ls_flush;
    
    assign ls_ack = mem_ls_ack;
    assign ls_error = mem_ls_error;
    assign ls_rdata = mem_ls_rdata;
    assign ls_store_fault = 0;
    assign ls_load_fault = 0;
    assign ls_accept = mem_ls_accept;
    
    // Tag passthrough
    assign mem_ls_cacheable = ls_cacheable;
    assign mem_ls_req_tag = ls_req_tag;
    assign ls_resp_tag = mem_ls_resp_tag;
end
endgenerate

endmodule