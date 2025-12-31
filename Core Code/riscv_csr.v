module riscv_csr
(
    input  wire        clk,
    input  wire        rst_n,

    // Core Interface (Read/Write)
    input  wire [11:0] addr,
    input  wire [31:0] wdata,
    input  wire [2:0]  op, // CSRRW, CSRRS, CSRRC
    input  wire        wen,
    output reg  [31:0] rdata,

    // Interrupts & Exceptions
    input  wire        intr_req,
    input  wire [3:0]  intr_cause,
    input  wire        exc_req,
    input  wire [3:0]  exc_cause,
    input  wire [31:0] exc_pc,
    input  wire [31:0] exc_val, // Bad address etc.
    output wire        trap_taken,
    output wire [31:0] trap_pc,

    // System State output to Core
    output wire [1:0]  priv_lvl,
    output wire        mie, // Machine Interrupt Enable
    output wire        sie  // Supervisor Interrupt Enable
);

`include "riscv_defs.v"

    //---------------------------------------------------------
    // CSR Registers
    //---------------------------------------------------------
    reg [31:0] mstatus;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;
    reg [31:0] mtvec;
    reg [31:0] mscratch;
    reg [31:0] mie_reg;
    reg [31:0] mip_reg;
    
    // Counters
    reg [63:0] mcycle;
    
    // Supervisor (Shadowed or separate depending on config)
    reg [31:0] sepc;
    reg [31:0] scause;
    reg [31:0] stval;
    reg [31:0] stvec;
    reg [31:0] sscratch;
    reg [31:0] satp;

    //---------------------------------------------------------
    // Operation Decoding
    //---------------------------------------------------------
    wire [1:0] priv = mstatus[12:11]; // MPP field usually
    assign priv_lvl = priv;
    assign mie = mstatus[3];
    assign sie = mstatus[1];

    // Cycle Counter
    always @(posedge clk or posedge rst_n)
        if (rst_n) mcycle <= 64'b0;
        else mcycle <= mcycle + 1;

    //---------------------------------------------------------
    // CSR Read Logic
    //---------------------------------------------------------
    always @(*) begin
        rdata = 32'b0;
        case (addr)
            // Machine Info
            `CSR_MSTATUS:  rdata = mstatus;
            `CSR_MISA:     rdata = `MISA_RV32 | `MISA_RVI | `MISA_RVM; // Basic features
            `CSR_MIE:      rdata = mie_reg;
            `CSR_MTVEC:    rdata = mtvec;
            `CSR_MSCRATCH: rdata = mscratch;
            `CSR_MEPC:     rdata = mepc;
            `CSR_MCAUSE:   rdata = mcause;
            `CSR_MTVAL:    rdata = mtval;
            `CSR_MIP:      rdata = mip_reg;
            
            // Timers
            `CSR_MCYCLE:   rdata = mcycle[31:0];
            `CSR_MCYCLE+12'h80: rdata = mcycle[63:32]; // High word logic if 32-bit split

            // Supervisor
            `CSR_SSTATUS:  rdata = mstatus & `CSR_SSTATUS_MASK;
            `CSR_SIE:      rdata = mie_reg & `CSR_SIE_MASK;
            `CSR_STVEC:    rdata = stvec;
            `CSR_SSCRATCH: rdata = sscratch;
            `CSR_SEPC:     rdata = sepc;
            `CSR_SCAUSE:   rdata = scause;
            `CSR_STVAL:    rdata = stval;
            `CSR_SATP:     rdata = satp;
            
            default:       rdata = 32'b0;
        endcase
    end

    //---------------------------------------------------------
    // CSR Write Logic
    //---------------------------------------------------------
    reg [31:0] write_val;
    
    always @(*) begin
        case (op[1:0])
            2'b01: write_val = wdata;              // CSRRW
            2'b10: write_val = rdata | wdata;      // CSRRS
            2'b11: write_val = rdata & ~wdata;     // CSRRC
            default: write_val = wdata;
        endcase
    end

    always @(posedge clk or posedge rst_n) begin
        if (rst_n) begin
            mstatus  <= 32'h00001800; // MPP=11 (Machine)
            mepc     <= 0;
            mcause   <= 0;
            mtval    <= 0;
            mtvec    <= 0;
            mscratch <= 0;
            mie_reg  <= 0;
            sepc     <= 0;
            scause   <= 0;
            stval    <= 0;
            stvec    <= 0;
            sscratch <= 0;
            satp     <= 0;
        end else begin
            // Trap Handling (Higher priority than SW writes)
            if (exc_req || intr_req) begin
                // Save PC
                mepc <= exc_pc;
                // Update Cause
                mcause <= intr_req ? {1'b1, 27'b0, intr_cause} : {1'b0, 27'b0, exc_cause};
                // Save bad address/opcode
                mtval  <= exc_val;
                
                // Push Attribute Stack (MIE->MPIE, etc)
                mstatus[`SR_MPIE_R] <= mstatus[`SR_MIE_R];
                mstatus[`SR_MIE_R]  <= 1'b0; // Disable interrupts
                mstatus[`SR_MPP_R]  <= priv;
                
                // If delegating to Supervisor, logic would go here (omitted for brevity)
            end
            // Software Write
            else if (wen) begin
                case (addr)
                    `CSR_MSTATUS:  mstatus  <= (write_val & `CSR_MSTATUS_MASK);
                    `CSR_MIE:      mie_reg  <= (write_val & `CSR_MIE_MASK);
                    `CSR_MTVEC:    mtvec    <= write_val;
                    `CSR_MSCRATCH: mscratch <= write_val;
                    `CSR_MEPC:     mepc     <= write_val;
                    `CSR_MCAUSE:   mcause   <= write_val;
                    `CSR_MTVAL:    mtval    <= write_val;
                    
                    `CSR_SSTATUS:  mstatus  <= (mstatus & ~`CSR_SSTATUS_MASK) | (write_val & `CSR_SSTATUS_MASK);
                    `CSR_SIE:      mie_reg  <= (mie_reg & ~`CSR_SIE_MASK) | (write_val & `CSR_SIE_MASK);
                    `CSR_STVEC:    stvec    <= write_val;
                    `CSR_SSCRATCH: sscratch <= write_val;
                    `CSR_SEPC:     sepc     <= write_val;
                    `CSR_SCAUSE:   scause   <= write_val;
                    `CSR_STVAL:    stval    <= write_val;
                    `CSR_SATP:     satp     <= write_val;
                endcase
            end
            
            // Interrupt Pending updates (external)
            // mip_reg <= ... (wired from external lines usually)
        end
    end

    //---------------------------------------------------------
    // Trap Control
    //---------------------------------------------------------
    assign trap_taken = exc_req | intr_req;
    
    // Vector logic: if mtvec[0] is 1, vectored interrupts. Here simple base.
    wire [31:0] base_vec = mtvec[31:2] << 2;
    assign trap_pc = base_vec; 

endmodule