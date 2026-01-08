module pwm_ip (
    input             clk,
    input             resetn,
    // Bus Interface
    input             i_sel,      // Chip Select
    input             i_we,       // Write Enable
    input      [3:0]  i_addr,     // Address Offset (mem_addr[3:0])
    input      [31:0] i_wdata,    // Data from CPU
    output reg [31:0] o_rdata,    // Data to CPU
    // External Output
    output reg        pwm_out     // PWM signal to SoC
);

    // Register Map Offsets
    localparam CTRL   = 4'h0;
    localparam PERIOD = 4'h4;
    localparam DUTY   = 4'h8;
    localparam STATUS = 4'hC;

    // Registers
    reg [31:0] reg_ctrl;   // Bit 0: EN, Bit 1: POL
    reg [31:0] reg_period; // Period in ticks
    reg [31:0] reg_duty;   // Duty cycle in ticks
    
    // Internal Counter
    reg [31:0] counter;

    // Control Bit Aliases
    wire ctrl_en  = reg_ctrl[0];
    wire ctrl_pol = reg_ctrl[1];

    // --- Write Logic ---
    always @(posedge clk) begin
        if (!resetn) begin
            reg_ctrl   <= 32'b0;
            reg_period <= 32'd1; // Default minimum value
            reg_duty   <= 32'b0;
        end else if (i_sel && i_we) begin
            case (i_addr)
                CTRL:   reg_ctrl   <= i_wdata;
                PERIOD: reg_period <= i_wdata;
                DUTY:   reg_duty   <= i_wdata;
            endcase
        end
    end

    // --- Read Logic ---
    always @(*) begin
        if (i_sel && !i_we) begin
            case (i_addr)
                CTRL:   o_rdata = reg_ctrl;
                PERIOD: o_rdata = reg_period;
                DUTY:   o_rdata = reg_duty;
                STATUS: o_rdata = {counter, 15'b0, ctrl_en}; // Bit 0: RUNNING, Bits [31:16]: CURRENT COUNTER
                default: o_rdata = 32'b0;
            endcase
        end else begin
            o_rdata = 32'b0;
        end
    end

    // --- PWM Logic ---
    always @(posedge clk) begin
        if (!resetn) begin
            counter <= 32'b0;
            pwm_out <= 1'b0;
        end else if (ctrl_en) begin
            // Counter Management
            if (counter >= (reg_period - 1)) begin
                counter <= 32'b0;
            end else begin
                counter <= counter + 1;
            end

            // Output Generation
            // (Apply Polarity: if POL=1, invert the result)
            if (counter < reg_duty) begin
                pwm_out <= ctrl_pol ? 1'b0 : 1'b1;
            end else begin
                pwm_out <= ctrl_pol ? 1'b1 : 1'b0;
            end
        end else begin
            // When Disabled: Force inactive level
            counter <= 32'b0;
            pwm_out <= ctrl_pol ? 1'b1 : 1'b0; 
        end
    end

endmodule
