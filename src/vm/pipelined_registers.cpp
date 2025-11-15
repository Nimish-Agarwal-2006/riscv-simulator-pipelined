#include "vm/pipelined_registers.h"
#include <limits>

// Define and initialize all pipeline registers with "invalid" values
IF_ID if_id = {
    .pc = std::numeric_limits<uint64_t>::max(),
    .instruction = 0xFFFFFFFF,
    .valid = false
};

ID_EX id_ex = {
    .pc = std::numeric_limits<uint64_t>::max(),
    .reg1_val = std::numeric_limits<uint64_t>::max(),
    .reg2_val = std::numeric_limits<uint64_t>::max(),
    .imm = INT32_MIN,
    .rs1 = 0xFF, .rs2 = 0xFF,.rs3=0xFF, .rd = 0xFF, .csr=0xFF,
    .funct3 = 0xFF, .funct7 = 0xFF, .opcode = 0xFF, .funct2=0xFF,.funct5=0xFF,
    .regWrite = false, .memRead = false, .memWrite = false, .branch = false,.branch_flag=false,.branch_predicted=false,
    .aluOp = false,
    .aluSrc = false,
    .execute_type= 0,.rs1_type=0,.rs2_type=0,.rs3_type=0,.rd_type=0,//0 for normal 1 for float 2 for double 3 for csr
    .valid = false
};

EX_MEM ex_mem = {
    .alu_result = std::numeric_limits<uint64_t>::max(),
    .reg2_val = std::numeric_limits<uint64_t>::max(),
    .rd = 0xFF,
    .regWrite = false, .memRead = false, .memWrite = false,
    .opcode=0xFF,.funct3 = 0xFF,.funct7=0xFF,
    .execute_type=0,.rd_type=0,
    .valid = false
};

MEM_WB mem_wb = {
    .opcode=0xFF,.funct3=0xFF,.funct7=0xFF,
    .mem_data = std::numeric_limits<uint64_t>::max(),
    .alu_result = std::numeric_limits<uint64_t>::max(),
    .rd = 0xFF,
    .regWrite = false,
    .memToReg = false,
    .execute_type=0,.rd_type=0,
    .valid = false
};
