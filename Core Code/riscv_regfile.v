module riscv_regfile
(
    input  wire        clk,
    input  wire        rst_n,

    // Read Port A
    input  wire [4:0]  rs1_addr,
    output wire [31:0] rs1_data,

    // Read Port B
    input  wire [4:0]  rs2_addr,
    output wire [31:0] rs2_data,

    // Write Port
    input  wire [4:0]  rd_addr,
    input  wire [31:0] rd_data,
    input  wire        wr_en
);

    //---------------------------------------------------------
    // Register Storage (Split for synthesis optimization)
    //---------------------------------------------------------
    reg [31:0] bank0 [0:15]; // R0-R15
    reg [31:0] bank1 [0:15]; // R16-R31

    //---------------------------------------------------------
    // Write Logic
    //---------------------------------------------------------
    wire write_b0 = wr_en && (rd_addr[4] == 1'b0);
    wire write_b1 = wr_en && (rd_addr[4] == 1'b1);
    
    // We don't actually write to R0, logic handles it by always reading 0
    // but for simplicity we write to the memory array anyway or gate it.
    // The strict check "rd_addr != 0" is usually done in control logic, 
    // but here we ensure Bank0[0] isn't corrupting logic if written.
    
    always @(posedge clk) begin
        if (write_b0 && (rd_addr[3:0] != 4'd0))
            bank0[rd_addr[3:0]] <= rd_data;
    end

    always @(posedge clk) begin
        if (write_b1)
            bank1[rd_addr[3:0]] <= rd_data;
    end

    //---------------------------------------------------------
    // Read Logic
    //---------------------------------------------------------
    wire [31:0] rs1_b0 = bank0[rs1_addr[3:0]];
    wire [31:0] rs1_b1 = bank1[rs1_addr[3:0]];
    
    wire [31:0] rs2_b0 = bank0[rs2_addr[3:0]];
    wire [31:0] rs2_b1 = bank1[rs2_addr[3:0]];

    // Muxing based on high bit of address
    reg [31:0] rs1_out;
    reg [31:0] rs2_out;

    always @(*) begin
        if (rs1_addr == 5'b0)
            rs1_out = 32'b0; // R0 is always 0
        else if (rs1_addr[4])
            rs1_out = rs1_b1;
        else
            rs1_out = rs1_b0;
    end

    always @(*) begin
        if (rs2_addr == 5'b0)
            rs2_out = 32'b0; // R0 is always 0
        else if (rs2_addr[4])
            rs2_out = rs2_b1;
        else
            rs2_out = rs2_b0;
    end

    assign rs1_data = rs1_out;
    assign rs2_data = rs2_out;

endmodule