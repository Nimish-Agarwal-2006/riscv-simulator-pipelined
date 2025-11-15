#ifndef PIPELINE_REGISTERS_H
#define PIPELINE_REGISTERS_H

#include <cstdint>
#include "alu.h"

struct IF_ID {
    uint64_t pc;
    uint32_t instruction;
    bool valid;
};

struct ID_EX {
    uint64_t pc;
    uint64_t reg1_val;
    uint64_t reg2_val;
    int32_t imm;
    uint8_t rs1, rs2,rs3, rd;
    uint16_t csr;
    uint8_t funct3, funct7, opcode,funct2,funct5;
    bool regWrite, memRead, memWrite, branch, branch_flag,branch_predicted;
    bool aluOp;
    bool aluSrc;
    uint8_t execute_type,rs1_type,rs2_type,rs3_type,rd_type;
    bool valid;
};

struct EX_MEM {
    uint64_t alu_result;
    uint64_t reg2_val;
    uint8_t rd;
    bool regWrite, memRead, memWrite;
    uint8_t opcode,funct3,funct7;
    uint8_t execute_type,rd_type;
    bool valid;
};

struct MEM_WB {
    uint8_t opcode,funct3,funct7;
    uint64_t mem_data;
    uint64_t alu_result;
    uint8_t rd;
    bool regWrite;
    bool memToReg;
    uint8_t execute_type,rd_type;
    bool valid;
};

// Global extern declarations
extern IF_ID if_id;
extern ID_EX id_ex;
extern EX_MEM ex_mem;
extern MEM_WB mem_wb;

#endif // PIPELINE_REGISTERS_H
