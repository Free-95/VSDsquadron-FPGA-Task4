module gpio_control_ip (
    input             clk,
    input             resetn,
    // Bus Interface
    input             i_sel,      // Chip Select
    input             i_we,       // Write Enable
    input      [3:0]  i_addr,     // Address Offset
    input      [31:0] i_wdata,    // Data from CPU
    output reg [31:0] o_rdata,    // Data to CPU (Readback)
    // External Interface
    inout      [3:0]  gpio_pins   // Bidirectional GPIO pins
);

    // Register Map Offsets
    localparam DATA = 4'h0;
    localparam DIR  = 4'h4;
    localparam READ = 4'h8;

    // Registers
    reg [3:0] gpio_data;
    reg [3:0] gpio_dir;

    // --- Write Logic ---
    always @(posedge clk) begin
        if (!resetn) begin
            gpio_data <= 32'b0;
            gpio_dir  <= 32'b0;
        end else if (i_sel && i_we) begin
            case (i_addr)
                DATA: gpio_data <= i_wdata[3:0];
                DIR : gpio_dir  <= i_wdata[3:0];
            endcase
        end
    end

    // --- Read Logic ---
    always @(*) begin
        if (i_sel && !i_we) begin
            case (i_addr)
                DATA   : o_rdata = gpio_data;          // Read data
                DIR    : o_rdata = gpio_dir;           // Read direction
                READ   : o_rdata = {28'b0, gpio_pins}; // Read actual pin state
                default: o_rdata = 32'b0;
            endcase
        end
    end

    // --- Output Tri-state Logic ---
    // 1 (Output mode), 0 (Input mode)
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gpio_ctrl
            assign gpio_pins[i] = gpio_dir[i] ? gpio_data[i] : 1'bz;
        end
    endgenerate

endmodule
