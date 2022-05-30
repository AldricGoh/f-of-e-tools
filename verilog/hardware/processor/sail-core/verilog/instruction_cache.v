/*
    Authored 2018-2019, Ryan Voo.

    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    *   Redistributions of source code must retain the above
        copyright notice, this list of conditions and the following
        disclaimer.

    *   Redistributions in binary form must reproduce the above
        copyright notice, this list of conditions and the following
        disclaimer in the documentation and/or other materials
        provided with the distribution.

    *   Neither the name of the author nor the names of its
        contributors may be used to endorse or promote products
        derived from this software without specific prior written
        permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/



/*
 *  Description:
 *
 *      This module implements the instruction cache.
 */

module instruction_cache (
    // Requested instruction address in memory
    input clk,
    input[31:0] addr,

    // Instruction data out
    output[31:0] data_out,
    

    // Register to get data from instruction memory
    output reg[26:0] mem_block_addr,

    // Bit to allow the reading of instruction memory
    output reg readmem,

    // Inserting the data set
    input[255:0] new_line,
    output reg clk_stall
    );
     
    /*
        instruction cache
        *** Address ***
        |       [31:8] Tag          |   [7:5] Index     |   [4:2] Word ID   |

        *** Data storage format ***
                                          |7|6|5|4|3|2|1|0| => Word
        |   Index   |   valid   |   tag   |  Instr_cache  |
    */   

    reg[255:0] instr_cache[0:7];
    reg[23:0] tag[0:7];
    reg valid[0:7];

    //states
    parameter INTI = 3;
    parameter IDLE = 0;
    parameter CACHE_MISS = 1;
    parameter READ_BLOCK = 2;

    //current state
    integer state;

    //address buffer register
    reg[31:0] addr_buf;

    //split address signal into tag, index and offset
    wire[23:0] addr_tag;
    wire[2:0] addr_index;
    wire[2:0] addr_word;
    assign addr_tag = addr[31:8];
    assign addr_index = addr[7:5];
    assign addr_word = addr[4:2];   // For some reason we ignore first 2 bits

    //instruction word buffer
    reg[31:0] instr_buf;

    //split address buffer signal into tag, index and offset
    wire[23:0] addr_buf_tag;
    wire[2:0] addr_buf_index;
    wire[2:0] addr_buf_word_offset;
    assign addr_buf_tag = addr_buf[31:8];
    assign addr_buf_index = addr_buf[7:5];
    assign addr_buf_word = addr_buf[4:2];

    //instruction word buffer
    reg[31:0] instr_buf;

    //line buffer
    reg[255:0] line_buf;

    //combinational logic to select word from block
    always @(*) begin
        case (addr_buf_word)
            3'b000: begin
                instr_buf = line_buf[31:0];
            end

            3'b001: begin
                instr_buf = line_buf[63:32];
            end

            3'b010: begin
                instr_buf = line_buf[95:64];
            end

            3'b011: begin
                instr_buf = line_buf[127:96];
            end

            3'b100: begin
                instr_buf = line_buf[159:128];
            end

            3'b101: begin
                instr_buf = line_buf[191:160];
            end

            3'b110: begin
                instr_buf = line_buf[223:192];
            end

            3'b111: begin
                instr_buf = line_buf[255:224];
            end
        endcase
    end

    integer i;
    initial begin
        //$readmemh("verilog/program.hex",instruction_memory);
        // Set buffer to all 0s
        addr_buf = 32'b0;
        state = IDLE;

        // Set all valid bits and tags to 0 => allows for data to be written in
        for(i=0; i<8; i++) begin
            valid[i] = 0;
            tag[i] = 23'b0;
        end
    end

    always @(posedge clk) begin
        case(state)
            IDLE: begin
                clk_stall <= 0;
                addr_buf <= addr;
                readmem <= 0;
                if(tag[addr_index] != addr_tag || valid[addr_index] != 1) begin
                    clk_stall <= 1;
                    readmem <= 1;
                    mem_block_addr <= addr[31:5];
                    state <= CACHE_MISS;
                end else begin
                    // Retrive data from that particular row
                    line_buf <= instr_cache[addr_index];
                end
            end

            CACHE_MISS: begin
                valid[addr_buf_index] <= 1;
                tag[addr_buf_index] <= addr_buf_tag;
                state <= READ_BLOCK;
            end

            READ_BLOCK: begin
                instr_cache[addr_buf_index] <= new_line;
                line_buf <= new_line;
                readmem <= 0;
                clk_stall <= 0;
                state <= IDLE;
            end

            default: begin
                //do nothing
            end
        endcase
    end

    assign data_out = instr_buf;

endmodule
