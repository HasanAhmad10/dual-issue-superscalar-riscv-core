module riscv_csr_regfile
#(
    parameter SUPPORT_MTIMECMP = 1,
    parameter SUPPORT_SUPER    = 0
)
(
    input  wire        clk,
    input  wire        rst_n,

    // Interrupt Inputs
    input  wire        ext_intr,
    input  wire        timer_intr,
    input  wire [31:0] cpu_id,
    input  wire [31:0] misa_val,

    // Exceptions
    input  wire [5:0]  exception,
    input  wire [31:0] exception_pc,
    input  wire [31:0] exception_addr,

    // Read Port
    input  wire        ren,
    input  wire [11:0] raddr,
    output reg  [31:0] rdata,

    // Write Port
    input  wire [11:0] waddr,
    input  wire [31:0] wdata,

    // Branching (Traps/Returns)
    output reg         csr_branch,
    output reg  [31:0] csr_target,

    // System Status Outputs
    output wire [1:0]  priv_lvl,
    output wire [31:0] status_reg,
    output wire [31:0] satp_reg,
    output wire [31:0] interrupt_pending
);

`include "riscv_defs.v"

    //---------------------------------------------------------
    // Registers
    //---------------------------------------------------------
    reg [31:0] mstatus, mepc, mcause, mtval, mtvec, mscratch;
    reg [31:0] mip, mie_reg;
    reg [63:0] mcycle;
    reg [31:0] mtimecmp;
    reg        mtime_ie; // Timer interrupt enable latch
    reg [31:0] medeleg, mideleg;

    // Supervisor
    reg [31:0] sepc, stvec, scause, stval, satp, sscratch;
    
    // Priv Level
    reg [1:0]  cur_priv;

    //---------------------------------------------------------
    // Interrupt Logic
    //---------------------------------------------------------
    wire [31:0] irq_pending = (mip & mie_reg);
    
    // Determine if interrupts are enabled based on priv level
    wire m_enabled = (cur_priv < `PRIV_MACHINE) || (cur_priv == `PRIV_MACHINE && mstatus[`SR_MIE_R]);
    wire s_enabled = (cur_priv < `PRIV_SUPER)   || (cur_priv == `PRIV_SUPER   && mstatus[`SR_SIE_R]);

    wire [31:0] m_irq = m_enabled ? (irq_pending & ~mideleg) : 32'b0;
    wire [31:0] s_irq = s_enabled ? (irq_pending &  mideleg) : 32'b0;

    assign interrupt_pending = (|m_irq) ? m_irq : s_irq;

    //---------------------------------------------------------
    // Read Logic
    //---------------------------------------------------------
    always @(*) begin
        rdata = 32'b0;
        if (ren) begin
            case (raddr)
                `CSR_MSTATUS:   rdata = mstatus & `CSR_MSTATUS_MASK;
                `CSR_MEPC:      rdata = mepc;
                `CSR_MTVEC:     rdata = mtvec;
                `CSR_MCAUSE:    rdata = mcause;
                `CSR_MTVAL:     rdata = mtval;
                `CSR_MIP:       rdata = mip;
                `CSR_MIE:       rdata = mie_reg;
                `CSR_MSCRATCH:  rdata = mscratch;
                `CSR_MCYCLE:    rdata = mcycle[31:0];
                `CSR_MCYCLE+12'h80: rdata = mcycle[63:32];
                `CSR_MHARTID:   rdata = cpu_id;
                `CSR_MISA:      rdata = misa_val;
                `CSR_MEDELEG:   rdata = medeleg;
                `CSR_MIDELEG:   rdata = mideleg;
                `CSR_MTIMECMP:  rdata = mtimecmp;
                
                // Supervisor
                `CSR_SSTATUS:   rdata = mstatus & `CSR_SSTATUS_MASK;
                `CSR_SEPC:      rdata = sepc;
                `CSR_STVEC:     rdata = stvec;
                `CSR_SCAUSE:    rdata = scause;
                `CSR_STVAL:     rdata = stval;
                `CSR_SATP:      rdata = satp;
                `CSR_SSCRATCH:  rdata = sscratch;
                `CSR_SIE:       rdata = mie_reg & `CSR_SIE_MASK;
                `CSR_SIP:       rdata = mip & `CSR_SIP_MASK;
                default:        rdata = 32'b0;
            endcase
        end
    end

    //---------------------------------------------------------
    // Write / State Update Logic
    //---------------------------------------------------------
    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            mstatus <= 32'h00001800; // MPP=Machine
            mepc <= 0; mcause <= 0; mtval <= 0; mtvec <= 0; mscratch <= 0;
            mip <= 0; mie_reg <= 0; mcycle <= 0; mtimecmp <= 0; mtime_ie <= 0;
            medeleg <= 0; mideleg <= 0;
            sepc <= 0; stvec <= 0; scause <= 0; stval <= 0; satp <= 0; sscratch <= 0;
            cur_priv <= `PRIV_MACHINE;
            csr_branch <= 0;
            csr_target <= 0;
        end else begin
            // Cycle Counter
            mcycle <= mcycle + 1;
            csr_branch <= 0;

            // Interrupt Sampling
            if (ext_intr)   mip[`SR_IP_MEIP_R] <= 1;
            else            mip[`SR_IP_MEIP_R] <= 0;
            
            if (timer_intr) mip[`SR_IP_MTIP_R] <= 1;
            else            mip[`SR_IP_MTIP_R] <= 0;

            // Trap Handling
            if (|exception) begin
                // ... (Trap logic similar to riscv_csr but handling reg updates)
                // If interrupt:
                if (exception == `EXCEPTION_INTERRUPT) begin
                    // Check if delegating
                    if (interrupt_pending & ~mideleg) begin // Machine
                         mstatus[`SR_MPIE_R] <= mstatus[`SR_MIE_R];
                         mstatus[`SR_MIE_R]  <= 0;
                         mstatus[`SR_MPP_R]  <= cur_priv;
                         cur_priv <= `PRIV_MACHINE;
                         mepc <= exception_pc;
                         // mcause logic...
                         csr_branch <= 1;
                         csr_target <= mtvec;
                    end else begin // Supervisor
                         mstatus[`SR_SPIE_R] <= mstatus[`SR_SIE_R];
                         mstatus[`SR_SIE_R]  <= 0;
                         mstatus[`SR_SPP_R]  <= (cur_priv == `PRIV_SUPER);
                         cur_priv <= `PRIV_SUPER;
                         sepc <= exception_pc;
                         csr_branch <= 1;
                         csr_target <= stvec;
                    end
                end 
                // xRET instructions
                else if (exception == `EXCEPTION_ERET_M) begin
                    cur_priv <= mstatus[`SR_MPP_R];
                    mstatus[`SR_MIE_R] <= mstatus[`SR_MPIE_R];
                    mstatus[`SR_MPIE_R] <= 1;
                    mstatus[`SR_MPP_R] <= `PRIV_USER;
                    csr_branch <= 1;
                    csr_target <= mepc;
                end
                // ... other exceptions ...
            end
            // Regular Writes
            else if (waddr != 0) begin
                case (waddr)
                    `CSR_MSTATUS: mstatus <= wdata & `CSR_MSTATUS_MASK;
                    `CSR_MIE:     mie_reg <= wdata & `CSR_MIE_MASK;
                    // ... etc ...
                endcase
            end
        end
    end

    assign priv_lvl = cur_priv;
    assign status_reg = mstatus;
    assign satp_reg = satp;

endmodule