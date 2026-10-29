//------------------------------------------------------------------------------
// The confidential and proprietary information contained in this file may
// only be used by a person authorised under and to the extent permitted
// by a subsisting licensing agreement from ARM Limited.
//
//            (C) COPYRIGHT 2010-2015  ARM Limited or its affiliates.
//                ALL RIGHTS RESERVED
//
// This entire notice must be reproduced on all copies of this file
// and copies of this file may only be made by a person if such person is
// permitted to do so under the terms of a subsisting license agreement
// from ARM Limited.
//
//  Version and Release Control Information:
//
//  File Revision       : $Revision: $
//  File Date           : $Date: $
//
//  Release Information : Cortex-M0 DesignStart-r1p0-00rel0
//------------------------------------------------------------------------------
// Verilog-2001 (IEEE Std 1364-2001)
//------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
// Abstract : Simple AHB RAM behavioral model
//-----------------------------------------------------------------------------
module cmsdk_ahb_ram_beh #(
  parameter AW       = 16,// Address width
  parameter filename = "",
  parameter WS_N     = 0, // First access wait state
  parameter WS_S     = 0) // Subsequent access wait state
 (
  input  wire          HCLK,    // Clock
  input  wire          HRESETn, // Reset
  input  wire          HSEL,    // Device select
  input  wire [1:0]    HTRANS,  // Transfer control
  input  wire [2:0]    HSIZE,   // Transfer size
  input  wire          HWRITE,  // Write control
  input  wire [AW-1:0] HADDR,   // Address
  input  wire [31:0]   HWDATA,  // Write data
  input  wire          HREADY,  // Transfer phase done
  output wire          HREADYOUT, // Device ready
  output wire [31:0]   HRDATA,  // Read data output
  output wire          HRESP);   // Device response (always OKAY)

  // Internal signals
  reg    [31:0] ram_data[0:((1<<(AW-2))-1)]; // word-addressed RAM
  wire          read_valid;     // Address phase read valid
  wire          write_valid;    // Address phase write valid
  reg           read_enable;    // Data phase read enable
  reg           write_enable;   // Data phase write enable
  reg    [3:0]  reg_byte_lane;  // Data phase byte lane
  reg    [3:0]  next_byte_lane; // Next state of reg_byte_lane

  reg    [7:0]  rdata_out_0;    // Read Data Output byte#0
  reg    [7:0]  rdata_out_1;    // Read Data Output byte#1
  reg    [7:0]  rdata_out_2;    // Read Data Output byte#2
  reg    [7:0]  rdata_out_3;    // Read Data Output byte#3
  reg [AW-1:0]  word_addr;      // Word aligned address
  wire [AW-1:0] nxt_word_addr;  // Word aligned address


  // Wait state control
  wire  [31:0]  nxt_waitstate_cnt;
  reg   [31:0]  reg_waitstate_cnt;
  wire          sequential_access;

  // Start of main code
  // Initialize ROM
  initial
    begin




    if (filename != "")
    begin
      $readmemh(filename, ram_data);
    end
    end

  // Generate read control (address phase)
  assign read_valid  = HSEL & HREADY & HTRANS[1] & (~HWRITE);
  // Generate write control (address phase)
  assign write_valid = HSEL & HREADY & HTRANS[1] & HWRITE;

  // Read enable for each byte (address phase)
  always @(read_valid or write_valid or HADDR or HSIZE)
  begin
  if (read_valid | write_valid)
    begin
    case (HSIZE)
      0 : // Byte
        begin
        case (HADDR[1:0])
          0: next_byte_lane = 4'b0001; // Byte 0
          1: next_byte_lane = 4'b0010; // Byte 1
          2: next_byte_lane = 4'b0100; // Byte 2
          3: next_byte_lane = 4'b1000; // Byte 3
          default:next_byte_lane = 4'b0000; // Address not valid
        endcase
        end
      1 : // Halfword
        begin
        if (HADDR[1])
          next_byte_lane = 4'b1100; // Upper halfword
        else
          next_byte_lane = 4'b0011; // Lower halfword
        end
      default : // Word
        next_byte_lane = 4'b1111; // Whole word
    endcase
    end
  else
    next_byte_lane = 4'b0000; // Not reading
  end

  // Registering control signals to data phase
  always @(posedge HCLK or negedge HRESETn)
  begin
    if (~HRESETn)
      begin
      reg_byte_lane <= 4'b0000;
      read_enable   <= 1'b0;
      write_enable  <= 1'b0;
      word_addr     <= {AW{1'b0}};
      end
    else if (HREADY)
      begin
      reg_byte_lane <= next_byte_lane;
      read_enable   <= read_valid;
      write_enable  <= write_valid;
      word_addr     <= nxt_word_addr;
      end
  end

  assign nxt_word_addr = {HADDR[AW-1:2], 2'b00};

  // Read : single registered read for BRAM inference
  reg [31:0] rdata_q;
  always @(posedge HCLK)
  begin
    if (read_enable)
      rdata_q <= ram_data[word_addr];
  end
  assign rdata_out_0 = (reg_byte_lane[0]) ? rdata_q[ 7: 0] : 8'h00;
  assign rdata_out_1 = (reg_byte_lane[1]) ? rdata_q[15: 8] : 8'h00;
  assign rdata_out_2 = (reg_byte_lane[2]) ? rdata_q[23:16] : 8'h00;
  assign rdata_out_3 = (reg_byte_lane[3]) ? rdata_q[31:24] : 8'h00;

  // Write : single full-word for BRAM inference
  always @(posedge HCLK)
  begin
    if (write_enable)
      ram_data[word_addr] <= HWDATA;
  end

  // Wait state control
    //  Wait state generate treat access as sequential if
    //  HTRANS = 2'b11, or access address is in the same word,
    //  or if the access is in the next word
  assign sequential_access = (HTRANS==2'b11) |
                            (HADDR[AW-1:2] == word_addr[AW-1:2]) |
                            (HADDR[AW-1:2] == (word_addr[AW-1:2]+1));
  assign nxt_waitstate_cnt = (read_valid|write_valid) ?
                             ((sequential_access) ? WS_S : WS_N) :
                             ((reg_waitstate_cnt!=0) ? (reg_waitstate_cnt - 1) : 0);
    // Register wait state counter
  always @(posedge HCLK or negedge HRESETn)
  begin
    if (~HRESETn)
      reg_waitstate_cnt <= 0;
    else
      reg_waitstate_cnt <= nxt_waitstate_cnt;
  end

  // Connect to top level
  assign HREADYOUT = (reg_waitstate_cnt==0) ? 1'b1 : 1'b0;
  assign HRESP     = 1'b0; // Always response with OKAY
  // Read data output
  assign HRDATA    = {rdata_out_3, rdata_out_2, rdata_out_1,rdata_out_0};

endmodule
