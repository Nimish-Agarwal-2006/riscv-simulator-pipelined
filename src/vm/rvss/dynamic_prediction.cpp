/**
 * @file stages.cpp
 * @brief RVSS VM pipeline stages implementation (renamed class: Dynamic)
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#include "vm/rvss/dynamic_prediction.h"

#include "utils.h"
#include "globals.h"
#include "common/instructions.h"
#include "config.h"
#include "vm/pipelined_registers.h"
#include <cctype>
#include <cstdint>
#include <iostream>
#include <tuple>
#include <stack>  
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

using instruction_set::Instruction;
using instruction_set::get_instr_encoding;

Dynamic::Dynamic() : VmBase() {
  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
}

Dynamic::~Dynamic() = default;

void Dynamic::Branch_Prediction(){
  if(id_ex.valid && id_ex.branch){

    if(id_ex.opcode==get_instr_encoding(Instruction::kjal).opcode){
        int last_execution=0;

        if(branch_map_.find(id_ex.pc)!=branch_map_.end()){
            last_execution=branch_map_[id_ex.pc];
        }
        if(last_execution==1){
            id_ex.branch_predicted=true;
            UpdateProgramCounter(-program_counter_ + id_ex.pc + id_ex.imm);
        }
    }
    else if(id_ex.opcode==get_instr_encoding(Instruction::kjalr).opcode){

        bool overflow = false;
        uint64_t alu_operand_2 = id_ex.reg2_val;
        int64_t res;
        if (id_ex.aluSrc) {
          alu_operand_2 = static_cast<uint64_t>(static_cast<int64_t>(id_ex.imm));
        }
        alu::AluOp aluOperation = control_unit_.GetAluSignal_pipelined(id_ex.aluOp);
        std::tie(res, overflow) = alu_.execute(aluOperation, id_ex.reg1_val, alu_operand_2);

        int last_execution=0;

        if(branch_map_.find(id_ex.pc)!=branch_map_.end()){
            last_execution=branch_map_[id_ex.pc];
        }

        if(last_execution==1){
            UpdateProgramCounter(-program_counter_ + res);
            id_ex.branch_predicted=true;
        }

    }
    else {
        int last_execution=0;

        if(branch_map_.find(id_ex.pc)!=branch_map_.end()){
            last_execution=branch_map_[id_ex.pc];
        }
        if(last_execution==1){
            id_ex.branch_predicted=true;
            UpdateProgramCounter(-program_counter_+id_ex.pc+id_ex.imm);
        }

    }
  }
}

void Dynamic:: Check_Prediction(){
  if(!id_ex.valid || !id_ex.branch) return;
  std::cout<<"prediction and flag "<<id_ex.branch_predicted <<" "<<id_ex.branch_flag<<"\n";
  if(id_ex.branch_predicted != id_ex.branch_flag){
    // only need to check for branch
    if(id_ex.opcode==0b1100011){
        if(id_ex.branch_predicted){
            UpdateProgramCounter(-program_counter_ + id_ex.pc+4);
        }
        else {
            UpdateProgramCounter(-program_counter_ + id_ex.pc + id_ex.imm);
        }
        if_id.valid=false;
    }
    else if(id_ex.opcode==get_instr_encoding(Instruction::kjal).opcode){
        // assuming prediction false flag true
        // later on change
        if_id.valid=false;
        UpdateProgramCounter(-program_counter_ + id_ex.pc + id_ex.imm);
    }
    else if(id_ex.opcode==get_instr_encoding(Instruction::kjalr).opcode){
        // same problem as predict not taken  then taken
        bool overflow = false;
        uint64_t alu_operand_2 = static_cast<uint64_t>(static_cast<int64_t>(id_ex.imm));;
        int64_t res;
        alu::AluOp aluOperation = control_unit_.GetAluSignal_pipelined(id_ex.aluOp);
        std::tie(res, overflow) = alu_.execute(aluOperation, id_ex.reg1_val, alu_operand_2);
        if_id.valid=false;
        UpdateProgramCounter(-program_counter_ + res);
    }
  }
}

void Dynamic::Forward_Data(){

bool stall=false;
  if(!id_ex.valid) return;
  if(mem_wb.valid){
        // prev to prev load no nop 
        if(mem_wb.memToReg){

                if(mem_wb.rd==id_ex.rs1){
                  if(mem_wb.rd_type==id_ex.rs1_type){
                    if(!(mem_wb.rd_type==0 && mem_wb.rd==0)){
                      id_ex.reg1_val=mem_wb.mem_data;
                    }
                  }
                }
                if(mem_wb.rd==id_ex.rs2){
                  if(mem_wb.rd_type==id_ex.rs2_type){
                    if(!(mem_wb.rd_type==0 && mem_wb.rd==0)){
                      id_ex.reg2_val=mem_wb.mem_data;
                    }
                  }
                }
    
        }
        // prev to prev alu pass
        /* ===========================================
        regwrite is on for load so earlier it was if statement overwriting the data 
        ============================================ */
        else if(mem_wb.regWrite){
            //std::cout<<""
            if(mem_wb.rd==id_ex.rs1){
              if(mem_wb.rd_type==id_ex.rs1_type){
                if(!(mem_wb.rd_type==0 && mem_wb.rd==0)){
                  id_ex.reg1_val=mem_wb.alu_result;
                }
              }
            }
            if(mem_wb.rd==id_ex.rs2){
              if(mem_wb.rd_type==id_ex.rs2_type){
                if(!(mem_wb.rd_type==0 && mem_wb.rd==0)){
                  id_ex.reg2_val=mem_wb.alu_result;
                }
              }
            }

        }
        
    } 
    if(ex_mem.valid){
        // prev instruction is load
        if(ex_mem.memRead){

            if(ex_mem.rd==id_ex.rs1){
              if(ex_mem.rd_type==id_ex.rs1_type){
                if(!(ex_mem.rd_type==0 && ex_mem.rd==0)){
                    stall=true;
                }
              }
            }
            if(ex_mem.rd==id_ex.rs2){
              if(ex_mem.rd_type==id_ex.rs2_type){
                if(!(ex_mem.rd_type==0 && ex_mem.rd==0)){
                  stall=true;
                }
              }
            }
            
        }
        // prev is alu pass
        else if(ex_mem.regWrite){


            if(ex_mem.rd==id_ex.rs1){
              if(ex_mem.rd_type==id_ex.rs1_type){
                if(!(ex_mem.rd_type==0 && ex_mem.rd==0)){
                  std::cout<<"alu result rs1 is ex "<<ex_mem.alu_result;
                  id_ex.reg1_val=ex_mem.alu_result;
                }
              }
            }
            if(ex_mem.rd==id_ex.rs2){
              if(ex_mem.rd_type==id_ex.rs2_type){
                if(!(ex_mem.rd_type==0 && ex_mem.rd==0)){
                  id_ex.reg2_val=ex_mem.alu_result;
                }
              }
            }

        }
    }


    if(stall){
        UpdateProgramCounter(-4);
        id_ex.valid=false;
    }

}


void Dynamic::Fetch() {
  
  current_instruction_ = memory_controller_.ReadWord(program_counter_);
  if_id.instruction=current_instruction_;
  //std::cout<<"Debug : instruction fetched: 0x"<<std::hex<<if_id.instruction<<std::dec<<std::endl;
  if_id.pc=program_counter_;
  if_id.valid=true;
  UpdateProgramCounter(4);
}



void Dynamic::Decode() {

    if (!if_id.valid) {
        id_ex.valid = false;
        
        id_ex.branch_flag = false;
        id_ex.branch_predicted = false;
        id_ex.branch = false; // Ensure control signal is off
        return;
    }
            id_ex.branch_flag = false;
        id_ex.branch_predicted = false;

    // --- 1. Decode and set basic info ---
    control_unit_.Decoding_the_instruction(if_id.instruction);
    std::cout<<"rs2 type just after decoding "<<(unsigned int)id_ex.rs2_type<<"\n";
    // Pass-through values to the next pipeline register
    id_ex.imm = ImmGenerator(if_id.instruction);
    id_ex.pc = if_id.pc;
    id_ex.valid = true;

    if (id_ex.opcode == 0b0100011 || // S-type (int store)
        id_ex.opcode == 0b0100111 || // S-type (FPU store)
        id_ex.opcode == 0b1100011)   // B-type (branch)
    {
        id_ex.rd = 0;      // Use 32 for "no register"
        id_ex.rd_type = 4;  // Set a default type
    }

    // --- 2. Set Execution Type ---
    if (instruction_set::isFInstruction(if_id.instruction)) {
        id_ex.execute_type = 1; // 'F' Type
    } else if (instruction_set::isDInstruction(if_id.instruction)) {
        id_ex.execute_type = 2; // 'D' Type
    } else if (id_ex.opcode == 0b1110011) {
        id_ex.execute_type = 3; // 'CSR' Type
    } else {
        id_ex.execute_type = 0; // 'Integer' Type
    }

    // --- 3. Read rs1 (from GPR or FPR) ---
    // Handles all instruction types
    
    // Instructions that DON'T use rs1
    if (id_ex.opcode == 111 /* JAL */ || 
        id_ex.opcode == 0b0110111 /* LUI */ || 
        id_ex.opcode == 0b0010111 /* AUIPC */) 
    {
        id_ex.rs1 = 32; // Mark as unused
        id_ex.rs1_type = 0;
    } 
    // F/D instructions (must check for GPR vs FPR)
    else if (id_ex.execute_type == 1 || id_ex.execute_type == 2) 
    {
        bool is_fpu_alu = (id_ex.opcode == 0b1010011);
        // Check for ops that use GPR as rs1 (e.g., FCVT.S.W, FMV.W.X)
        bool fpu_reads_gpr = (id_ex.funct7 == 0b1101000 || id_ex.funct7 == 0b1111000 || // Float
                              id_ex.funct7 == 0b1101001 || id_ex.funct7 == 0b1111001); // Double

        if (is_fpu_alu && !fpu_reads_gpr) {
            // FPU ALU ops: FADD, FSUB, FCVT.W.S, etc.
            id_ex.reg1_val = registers_.ReadFpr(id_ex.rs1);
            //id_ex.rs1_type = 1; // FPR
        } else {
            // FPU Loads/Stores (base address) or FCVT.S.W (integer data)
            id_ex.reg1_val = registers_.ReadGpr(id_ex.rs1);
            id_ex.rs1_type = 0; // GPR
        }
    } 
    // All other integer instructions (R, I, S, B, JALR)
    else 
    {
        id_ex.reg1_val = registers_.ReadGpr(id_ex.rs1);
        id_ex.rs1_type = 0; // GPR
    }


    // --- 4. Read rs2 (from GPR or FPR) ---
    
    // Check if it's an FPU convert/move op (like fcvt, fmv)
    // These use funct7[6:5] == 0b11 and do NOT use rs2 as a register.
    bool is_op_fp = (id_ex.opcode == 0b1010011);
    bool is_fpu_conv_or_move = is_op_fp && ((id_ex.funct7 >> 5) == 0b11);

    // An OP-FP instruction only uses rs2 if it's *NOT* a convert/move op
    bool op_fp_uses_rs2 = is_op_fp && !is_fpu_conv_or_move;
    
    // Check for opcodes that use rs2 (R, S, B types)
    bool uses_rs2 = (id_ex.opcode == 0b0110011 || // R-type (int)
                     id_ex.opcode == 0b0100011 || // S-type (int store)
                     id_ex.opcode == 0b1100011 || // B-type (branch)
                     op_fp_uses_rs2 ||            // R-type (FPU) <-- FIXED
                     id_ex.opcode == 0b0100111);  // S-type (FPU store)

    if (uses_rs2) {
        if (id_ex.execute_type == 1 || id_ex.execute_type == 2) { 
            // FPU R-type (FADD.S) or S-type (FSW)
            id_ex.reg2_val = registers_.ReadFpr(id_ex.rs2); 
            //id_ex.rs2_type = 1; // FPR
        } else {
            // Integer R-type, S-type, or B-type
            id_ex.reg2_val = registers_.ReadGpr(id_ex.rs2); 
            //id_ex.rs2_type = 0; // GPR
        }
    } else {
        // Doesn't use rs2 (I-type, U-type, J-type, FPU Loads, FPU Cvt/Move)
        id_ex.rs2 = 32; // Mark as unused
        //id_ex.rs2_type = 0;
    }
}
// void Dynamic::Decode() {
//   // if(if_id.valid) std::cout<<"decode is valid \n";
//   // else std::cout<<"decode is invalid\n";

//   if(if_id.valid==true)
//   {
//         control_unit_.Decoding_the_instruction(if_id.instruction);
//         //std::cout<<"Debug : if_id.instruction : 0x" << std::hex << if_id.instruction << std::endl;
//         id_ex.imm = ImmGenerator(if_id.instruction);
//         id_ex.execute_type=0;
//   if (instruction_set::isFInstruction(if_id.instruction)) { // RV64 F
//     id_ex.execute_type=1;
//     id_ex.reg1_val = registers_.ReadFpr(id_ex.rs1);
//     id_ex.reg2_val= registers_.ReadFpr(id_ex.rs2);
//   } else if (instruction_set::isDInstruction(if_id.instruction)) {
//     id_ex.execute_type=2;
//   } else if (id_ex.opcode==0b1110011) {
//     id_ex.execute_type=3;
//   }
//   else if(id_ex.opcode==19 || id_ex.opcode==3 || id_ex.opcode == 103){
//       id_ex.reg1_val = registers_.ReadGpr(id_ex.rs1);
//       id_ex.rs2=32;
//   }
//   else if(id_ex.opcode==111) { // jal rs1 rs2 not req
//           id_ex.rs1=32; 
//           id_ex.rs2=32; 
//     }
//     /// rs2 change rs2 not in instr
//   else
//   {
//     id_ex.reg1_val = registers_.ReadGpr(id_ex.rs1);
//     // std::cout<<"reg1_1 val after decoding"<< (unsigned int ) id_ex.reg1_val<<"\n";
//     id_ex.reg2_val= registers_.ReadGpr(id_ex.rs2);
//   }

//   std::cout << "Debug : rs1 index:"<< std::hex <<(unsigned int)id_ex.rs1 << '\n';
//   std::cout << "Debug : rs1 val:"<<std::hex<<(unsigned int)id_ex.reg1_val << '\n';
//   //std::cout << "Debug : rs2 index:"<<std::hex << (unsigned int)id_ex.rs2 << '\n';
//   id_ex.valid=true;
//   //std::cout << "Debug : id_exvalid :" << id_ex.valid << std::endl;
//     }
//     else
//     id_ex.valid=false;
// }

void Dynamic::Execute() {
 // uint8_t opcode = current_instruction_ & 0b1111111;
  //uint8_t funct3 = (current_instruction_ >> 12) & 0b111;


  // if(id_ex.valid) std::cout<<"execute is valid \n";
  // else std::cout<<"execute is invlaid\n";
  if(id_ex.valid==true)
  {
    ex_mem.execute_type=id_ex.execute_type;
  if (id_ex.opcode == get_instr_encoding(Instruction::kecall).opcode && 
      id_ex.funct3 == get_instr_encoding(Instruction::kecall).funct3) {
    HandleSyscall();
    return;
  }

  
  if (id_ex.execute_type==1) { // RV64 F
    //std::cout<<"reg2 val "<<id_ex.reg2_val<<'\n';
    ExecuteFloat();
    //std::cout<<"reg2 val "<<id_ex.reg2_val<<'\n';
    ex_mem.alu_result = execution_result_;
    ex_mem.reg2_val =id_ex.reg2_val;
    ex_mem.rd = id_ex.rd;
    ex_mem.regWrite = id_ex.regWrite, ex_mem.memRead = id_ex.memRead, ex_mem.memWrite = id_ex.memWrite;
    ex_mem.funct3 = id_ex.funct3;
    ex_mem.funct7 = id_ex.funct7;
    ex_mem.opcode=id_ex.opcode;
    ex_mem.rd_type=id_ex.rd_type;
    ex_mem.valid=true;
    return;
  } else if (id_ex.execute_type==2) {
    ExecuteDouble();
    ex_mem.alu_result = execution_result_;
    ex_mem.reg2_val =id_ex.reg2_val;
    ex_mem.rd = id_ex.rd;
    ex_mem.regWrite = id_ex.regWrite, ex_mem.memRead = id_ex.memRead, ex_mem.memWrite = id_ex.memWrite;
    ex_mem.funct3 = id_ex.funct3;
    ex_mem.funct7 = id_ex.funct7;
    ex_mem.opcode=id_ex.opcode;
    ex_mem.rd_type=id_ex.rd_type;
    ex_mem.valid=true;
    return;
  } else if (id_ex.execute_type==3) {
    ExecuteCsr();
    ex_mem.alu_result = execution_result_;
    ex_mem.reg2_val =id_ex.reg2_val;
    ex_mem.rd = id_ex.rd;
    ex_mem.regWrite = id_ex.regWrite, ex_mem.memRead = id_ex.memRead, ex_mem.memWrite = id_ex.memWrite;
    ex_mem.funct3 = id_ex.funct3;
    ex_mem.funct7 = id_ex.funct7;
    ex_mem.opcode=id_ex.opcode;
    ex_mem.rd_type=id_ex.rd_type;
    ex_mem.valid=true;
    return;
  }

  

  bool overflow = false;
//=======================================
  //fixed store bug 
  uint64_t alu_operand_2 = id_ex.reg2_val;
  
  if (id_ex.aluSrc) {
    alu_operand_2 = static_cast<uint64_t>(static_cast<int64_t>(id_ex.imm));
  }

  // if(id_ex.opcode==0b0110111){
  //   id_ex.reg1_val=0;
  //   alu_operand_2=id_ex.funct7;
  //   alu_operand_2<<=5;
  //   alu_operand_2+=id_ex.rs2;
  //   alu_operand_2<<=5;
  //   alu_operand_2+=id_ex.rs1;
  //   alu_operand_2<<=3;
  //   alu_operand_2+=id_ex.funct3;
  //   //alu_operand_2<<=12;
  // }
//==========================================
// change from nimish as argument controlunit.alup to 
  alu::AluOp aluOperation = control_unit_.GetAluSignal_pipelined(id_ex.aluOp);
  std::tie(execution_result_, overflow) = alu_.execute(aluOperation, id_ex.reg1_val, alu_operand_2);
  
  if(id_ex.opcode==55){
    //std::cout<<" lui imm"<<id_ex.imm<<"\n";
    execution_result_=id_ex.imm<<12;
  }
    //==============================
  // FOR JALR EXECUTION_RESULT IS
  //===============================
  //uint64_t final_val=execution_result_; 
// change in if getBranch 
  if (id_ex.branch){
    if (id_ex.opcode==get_instr_encoding(Instruction::kjalr).opcode || 
        id_ex.opcode==get_instr_encoding(Instruction::kjal).opcode) {
      next_pc_ = static_cast<int64_t>(program_counter_); // PC was already updated in Fetch()
      //UpdateProgramCounter(-4);
      //return_address_ = program_counter_ + 4;
      id_ex.branch_flag=true;
      return_address_=id_ex.pc+4;
      // need to check here
      //============================
      if (id_ex.opcode==get_instr_encoding(Instruction::kjalr).opcode) { 
        //UpdateProgramCounter(-program_counter_ + (execution_result_));
      } 
      //=============================
      else if (id_ex.opcode==get_instr_encoding(Instruction::kjal).opcode) {
        //UpdateProgramCounter(-program_counter_+id_ex.pc+id_ex.imm);
      }
    } else if (id_ex.opcode==get_instr_encoding(Instruction::kbeq).opcode ||
               id_ex.opcode==get_instr_encoding(Instruction::kbne).opcode ||
               id_ex.opcode==get_instr_encoding(Instruction::kblt).opcode ||
               id_ex.opcode==get_instr_encoding(Instruction::kbge).opcode ||
               id_ex.opcode==get_instr_encoding(Instruction::kbltu).opcode ||
               id_ex.opcode==get_instr_encoding(Instruction::kbgeu).opcode) {
      switch (id_ex.funct3) {
        case 0b000: {// BEQ
          id_ex.branch_flag = (execution_result_==0);
          break;
        }
        case 0b001: {// BNE
          id_ex.branch_flag = (execution_result_!=0);
          break;
        }
        case 0b100: {// BLT
          id_ex.branch_flag = (execution_result_==1);
          break;
        }
        case 0b101: {// BGE
          id_ex.branch_flag = (execution_result_==0);
          break;
        }
        case 0b110: {// BLTU
          id_ex.branch_flag = (execution_result_==1);
          break;
        }
        case 0b111: {// BGEU
          id_ex.branch_flag = (execution_result_==0);
          break;
        }
      }

    }



  }
  if(id_ex.branch){
    uint64_t instruction_map=id_ex.pc;
    if(id_ex.branch_flag){
        branch_map_[instruction_map]=1;
    }
    else branch_map_[instruction_map]=0;
  }




  if (id_ex.opcode==get_instr_encoding(Instruction::kauipc).opcode) { // AUIPC
    execution_result_ = static_cast<int64_t>(program_counter_) - 12 + (id_ex.imm << 12);

  }
  if(id_ex.branch){
    if (id_ex.opcode==get_instr_encoding(Instruction::kjalr).opcode || 
        id_ex.opcode==get_instr_encoding(Instruction::kjal).opcode) {
          ex_mem.alu_result = return_address_;
        }
        else ex_mem.alu_result=execution_result_;
  }
  else  ex_mem.alu_result=execution_result_;
    ex_mem.reg2_val =id_ex.reg2_val;
    ex_mem.rd = id_ex.rd;
    ex_mem.regWrite = id_ex.regWrite, ex_mem.memRead = id_ex.memRead, ex_mem.memWrite = id_ex.memWrite;
    ex_mem.funct3 = id_ex.funct3;
    ex_mem.funct7 = id_ex.funct7;
    ex_mem.opcode=id_ex.opcode;
    ex_mem.rd_type=id_ex.rd_type;
    ex_mem.valid=true;
}
else
ex_mem.valid=false;

//std::cout<<"after float "<<(unsigned int)id_ex.rs2_type<<"\n";
}

void Dynamic::ExecuteFloat() {
  // uint8_t opcode = current_instruction_ & 0b1111111;
  // uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
  // uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
  uint8_t rm = id_ex.funct3;
  // uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
  // uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
  // uint8_t rs3 = (current_instruction_ >> 27) & 0b11111;

  uint8_t fcsr_status = 0;

  //int32_t imm = ImmGenerator(current_instruction_);

  if (rm==0b111) {
    rm = registers_.ReadCsr(0x002);
  }
  //changed
  uint64_t reg1_value = id_ex.reg1_val;
  uint64_t reg2_value = id_ex.reg2_val;
  uint64_t reg3_value = registers_.ReadFpr(id_ex.rs3);

  if (id_ex.funct7==0b1101000 || id_ex.funct7==0b1111000 || id_ex.opcode==0b0000111 || id_ex.opcode==0b0100111) {
    reg1_value = id_ex.reg1_val;
  }
  //doubt
  if (id_ex.aluSrc) {
    reg2_value = static_cast<uint64_t>(static_cast<int64_t>(id_ex.imm));
  }

  alu::AluOp aluOperation = control_unit_.GetAluSignal_pipelined(id_ex.aluOp);
  //std::cout<<"aluop : "<<aluOperation<<" reg1_val: "<<reg1_value<<" reg2_val: "<<reg2_value<<" reg3_val: "<<reg3_value<<"\n";
  std::tie(execution_result_, fcsr_status) = alu::Alu::fpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);

  // std::cout << "+++++ Float execution result: " << execution_result_ << std::endl;


  registers_.WriteCsr(0x003, fcsr_status);
}

void Dynamic::ExecuteDouble() {
  // uint8_t opcode = current_instruction_ & 0b1111111;
  // uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
  // uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
   uint8_t rm = id_ex.funct3;
  // uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
  // uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
  // uint8_t rs3 = (current_instruction_ >> 27) & 0b11111;

  uint8_t fcsr_status = 0;

  //int32_t imm = ImmGenerator(current_instruction_);

  uint64_t reg1_value = id_ex.reg1_val;
  uint64_t reg2_value = id_ex.reg2_val;
  uint64_t reg3_value = registers_.ReadFpr(id_ex.rs3);

  if (id_ex.funct7==0b1101001 || id_ex.funct7==0b1111001 || id_ex.opcode==0b0000111 || id_ex.opcode==0b0100111) {
    reg1_value = id_ex.reg1_val;
  }
  // donno
  if (id_ex.aluSrc) {
    reg2_value = static_cast<uint64_t>(static_cast<int64_t>(id_ex.imm));
  }

  alu::AluOp aluOperation = control_unit_.GetAluSignal_pipelined(id_ex.aluOp);
  std::tie(execution_result_, fcsr_status) = alu::Alu::dfpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);
}

void Dynamic::ExecuteCsr() {
  //uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
  //uint16_t csr = (current_instruction_ >> 20) & 0xFFF;
  uint64_t csr_val = registers_.ReadCsr(id_ex.csr);

  csr_target_address_ = id_ex.csr;
  csr_old_value_ = csr_val;
  csr_write_val_ = registers_.ReadGpr(id_ex.rs1);
  csr_uimm_ = id_ex.rs1;
}

// TODO: implement writeback for syscalls
void Dynamic::HandleSyscall() {
  uint64_t syscall_number = registers_.ReadGpr(17);
  switch (syscall_number) {
    case SYSCALL_PRINT_INT: {
        if (!globals::vm_as_backend) {
            std::cout << "[Syscall output: ";
        } else {
          std::cout << "VM_STDOUT_START";
        }
        std::cout << static_cast<int64_t>(registers_.ReadGpr(10)); // Print signed integer
        if (!globals::vm_as_backend) {
            std::cout << "]" << std::endl;
        } else {
          std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_FLOAT: { // print float
        if (!globals::vm_as_backend) {
            std::cout << "[Syscall output: ";
        } else {
          std::cout << "VM_STDOUT_START";
        }
        float float_value;
        uint64_t raw = registers_.ReadGpr(10);
        std::memcpy(&float_value, &raw, sizeof(float_value));
        std::cout << std::setprecision(std::numeric_limits<float>::max_digits10) << float_value;
        if (!globals::vm_as_backend) {
            std::cout << "]" << std::endl;
        } else {
          std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_DOUBLE: { // print double
        if (!globals::vm_as_backend) {
            std::cout << "[Syscall output: ";
        } else {
          std::cout << "VM_STDOUT_START";
        }
        double double_value;
        uint64_t raw = registers_.ReadGpr(10);
        std::memcpy(&double_value, &raw, sizeof(double_value));
        std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << double_value;
        if (!globals::vm_as_backend) {
            std::cout << "]" << std::endl;
        } else {
          std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_STRING: {
        if (!globals::vm_as_backend) {
            std::cout << "[Syscall output: ";
        }
        PrintString(registers_.ReadGpr(10)); // Print string
        if (!globals::vm_as_backend) {
            std::cout << "]" << std::endl;
        }
        break;
    }
    case SYSCALL_EXIT: {
        stop_requested_ = true; // Stop the VM
        if (!globals::vm_as_backend) {
            std::cout << "VM_EXIT" << std::endl;
        }
        output_status_ = "VM_EXIT";
        std::cout << "Exited with exit code: " << registers_.ReadGpr(10) << std::endl;
        exit(0); // Exit the program
        break;
    }
    case SYSCALL_READ: { // Read
      uint64_t file_descriptor = registers_.ReadGpr(10);
      uint64_t buffer_address = registers_.ReadGpr(11);
      uint64_t length = registers_.ReadGpr(12);

      if (file_descriptor == 0) {
        // Read from stdin
        std::string input;
        {
          std::cout << "VM_STDIN_START" << std::endl;
          output_status_ = "VM_STDIN_START";
          std::unique_lock<std::mutex> lock(input_mutex_);
          input_cv_.wait(lock, [this]() { 
            return !input_queue_.empty(); 
          });
          output_status_ = "VM_STDIN_END";
          std::cout << "VM_STDIN_END" << std::endl;

          input = input_queue_.front();
          input_queue_.pop();
        }


        std::vector<uint8_t> old_bytes_vec(length, 0);
        std::vector<uint8_t> new_bytes_vec(length, 0);

        for (size_t i = 0; i < length; ++i) {
          old_bytes_vec[i] = memory_controller_.ReadByte(buffer_address + i);
        }
        
        for (size_t i = 0; i < input.size() && i < length; ++i) {
          memory_controller_.WriteByte(buffer_address + i, static_cast<uint8_t>(input[i]));
        }
        if (input.size() < length) {
          memory_controller_.WriteByte(buffer_address + input.size(), '\0');
        }

        for (size_t i = 0; i < length; ++i) {
          new_bytes_vec[i] = memory_controller_.ReadByte(buffer_address + i);
        }

        current_delta_.memory_changes.push_back({
          buffer_address, 
          old_bytes_vec, 
          new_bytes_vec
        });

        uint64_t old_reg = registers_.ReadGpr(10);
        unsigned int reg_index = 10;
        unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
        uint64_t new_reg = std::min(static_cast<uint64_t>(length), static_cast<uint64_t>(input.size()));
        registers_.WriteGpr(10, new_reg); 
        if (old_reg != new_reg) {
          current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
        }

      } else {
          std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
      }
      break;
    }
    case SYSCALL_WRITE: { // Write
        uint64_t file_descriptor = registers_.ReadGpr(10);
        uint64_t buffer_address = registers_.ReadGpr(11);
        uint64_t length = registers_.ReadGpr(12);

        if (file_descriptor == 1) { // stdout
          std::cout << "VM_STDOUT_START";
          output_status_ = "VM_STDOUT_START";
          uint64_t bytes_printed = 0;
          for (uint64_t i = 0; i < length; ++i) {
              char c = memory_controller_.ReadByte(buffer_address + i);
              // if (c == '\0') {
              //     break;
              // }
              std::cout << c;
              bytes_printed++;
          }
          std::cout << std::flush; 
          output_status_ = "VM_STDOUT_END";
          std::cout << "VM_STDOUT_END" << std::endl;

          uint64_t old_reg = registers_.ReadGpr(10);
          unsigned int reg_index = 10;
          unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
          uint64_t new_reg = std::min(static_cast<uint64_t>(length), bytes_printed);
          registers_.WriteGpr(10, new_reg);
          if (old_reg != new_reg) {
            current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
          }
        } else {
            std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
        }
        break;
    }
    default: {
      std::cerr << "Unknown syscall number: " << syscall_number << std::endl;
      break;
    }
  }
}

void Dynamic::WriteMemory() {
  // uint8_t opcode = current_instruction_ & 0b1111111;
  // uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
  // uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
  if(ex_mem.valid==true)
  {

    mem_wb.execute_type=ex_mem.execute_type;
  if (ex_mem.opcode == 0b1110011 && ex_mem.funct3 == 0b000) {
    mem_wb.opcode=ex_mem.opcode,mem_wb.funct3=ex_mem.funct3,mem_wb.funct7=ex_mem.funct7;
    mem_wb.mem_data = memory_result_;
    mem_wb.alu_result = ex_mem.alu_result;
    mem_wb.rd = ex_mem.rd,
    mem_wb.regWrite = ex_mem.regWrite;
    mem_wb.memToReg = ex_mem.memRead;
    mem_wb.rd_type=ex_mem.rd_type;
    mem_wb.valid=true;
    return;
  }

  if (ex_mem.execute_type==1) { // RV64 F
    WriteMemoryFloat();
    mem_wb.opcode=ex_mem.opcode,mem_wb.funct3=ex_mem.funct3,mem_wb.funct7=ex_mem.funct7;
    mem_wb.mem_data = memory_result_;
    mem_wb.alu_result = ex_mem.alu_result;
    mem_wb.rd = ex_mem.rd,
    mem_wb.regWrite = ex_mem.regWrite;
    mem_wb.memToReg = ex_mem.memRead;
    mem_wb.rd_type=ex_mem.rd_type;
    mem_wb.valid=true;
    return;
  } else if (ex_mem.execute_type==2) {
    WriteMemoryDouble();
    mem_wb.opcode=ex_mem.opcode,mem_wb.funct3=ex_mem.funct3,mem_wb.funct7=ex_mem.funct7;
    mem_wb.mem_data = memory_result_;
    mem_wb.alu_result = ex_mem.alu_result;
    mem_wb.rd = ex_mem.rd,
    mem_wb.regWrite = ex_mem.regWrite;
    mem_wb.memToReg = ex_mem.memRead;
    mem_wb.rd_type=ex_mem.rd_type;
    mem_wb.valid=true;
    return;
  }

  if (ex_mem.memRead==true) {
    switch (ex_mem.funct3) {
      case 0b000: {// LB
        memory_result_ = static_cast<int8_t>(memory_controller_.ReadByte(ex_mem.alu_result));
        break;
      }
      case 0b001: {// LH
        memory_result_ = static_cast<int16_t>(memory_controller_.ReadHalfWord(ex_mem.alu_result));
        break;
      }
      case 0b010: {// LW
        memory_result_ = static_cast<int32_t>(memory_controller_.ReadWord(ex_mem.alu_result));
        break;
      }
      case 0b011: {// LD
        memory_result_ = memory_controller_.ReadDoubleWord(ex_mem.alu_result);
        break;
      }
      case 0b100: {// LBU
        memory_result_ = static_cast<uint8_t>(memory_controller_.ReadByte(ex_mem.alu_result));
        break;
      }
      case 0b101: {// LHU
        memory_result_ = static_cast<uint16_t>(memory_controller_.ReadHalfWord(ex_mem.alu_result));
        break;
      }
      case 0b110: {// LWU
        memory_result_ = static_cast<uint32_t>(memory_controller_.ReadWord(ex_mem.alu_result));
        break;
      }
    }
  }

  uint64_t addr = 0;
  std::vector<uint8_t> old_bytes_vec;
  std::vector<uint8_t> new_bytes_vec;

  // TODO: use direct read to read memory for undo/redo functionality, i.e. ReadByte -> ReadByte_d


  if (ex_mem.memWrite==true) {
    switch (ex_mem.funct3) {
      case 0b000: {// SB
        addr = ex_mem.alu_result;
        old_bytes_vec.push_back(memory_controller_.ReadByte(addr));
        memory_controller_.WriteByte(ex_mem.alu_result, ex_mem.reg2_val & 0xFF);
        new_bytes_vec.push_back(memory_controller_.ReadByte(addr));
        break;
      }
      case 0b001: {// SH
        addr = ex_mem.alu_result;
        for (size_t i = 0; i < 2; ++i) {
          old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        memory_controller_.WriteHalfWord(ex_mem.alu_result, ex_mem.reg2_val & 0xFFFF);
        for (size_t i = 0; i < 2; ++i) {
          new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        break;
      }
      case 0b010: {// SW
        addr = ex_mem.alu_result;
        for (size_t i = 0; i < 4; ++i) {
          old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        memory_controller_.WriteWord(ex_mem.alu_result, ex_mem.reg2_val & 0xFFFFFFFF);
        for (size_t i = 0; i < 4; ++i) {
          new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        break;
      }
      case 0b011: {// SD
        addr = ex_mem.alu_result;
        for (size_t i = 0; i < 8; ++i) {
          old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        memory_controller_.WriteDoubleWord(ex_mem.alu_result, ex_mem.reg2_val & 0xFFFFFFFFFFFFFFFF);
        for (size_t i = 0; i < 8; ++i) {
          new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        break;
      }
    }
  }

  if (old_bytes_vec != new_bytes_vec) {
    current_delta_.memory_changes.push_back({
      addr,
      old_bytes_vec,
      new_bytes_vec
    });
  }
  mem_wb.opcode=ex_mem.opcode,mem_wb.funct3=ex_mem.funct3,mem_wb.funct7=ex_mem.funct7;
    mem_wb.mem_data = memory_result_;
    mem_wb.alu_result = ex_mem.alu_result;
    mem_wb.rd = ex_mem.rd,
    mem_wb.regWrite = ex_mem.regWrite;
    mem_wb.memToReg = ex_mem.memRead;
    mem_wb.rd_type=ex_mem.rd_type;
    mem_wb.valid=true;
}
// bsdi bug here 
else
  mem_wb.valid=false;
}

void Dynamic::WriteMemoryFloat() {
  //uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

  if (ex_mem.memRead) { // FLW
    //std::cout<<"memory to read from "<<ex_mem.alu_result;
    memory_result_ = memory_controller_.ReadWord(ex_mem.alu_result);
    //std::cout<<"memory result "<<memory_result_<<"\n";
  }

  // std::cout << "+++++ Memory result: " << memory_result_ << std::endl;

  uint64_t addr = 0;
  std::vector<uint8_t> old_bytes_vec;
  std::vector<uint8_t> new_bytes_vec;

  if (ex_mem.memWrite) { // FSW
    addr = ex_mem.alu_result;
    for (size_t i = 0; i < 4; ++i) {
      old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
    }
    //std::cout<<"befoe masala "<<id_ex.reg2_val<<'\n';
    uint32_t val = ex_mem.reg2_val & 0xFFFFFFFF;
    //std::cout<<"write back val"<<val<<"\n"; 
    memory_controller_.WriteWord(ex_mem.alu_result, val);
    // new_bytes_vec.push_back(memory_controller_.ReadByte(addr));
    for (size_t i = 0; i < 4; ++i) {
      new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
    }
  }

  if (old_bytes_vec!=new_bytes_vec) {
    current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
  }
  mem_wb.mem_data=memory_result_;
}

void Dynamic::WriteMemoryDouble() {
  //uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

  if (ex_mem.memRead) {// FLD
    memory_result_ = memory_controller_.ReadDoubleWord(ex_mem.alu_result);
  }

  uint64_t addr = 0;
  std::vector<uint8_t> old_bytes_vec;
  std::vector<uint8_t> new_bytes_vec;

  if (ex_mem.memWrite) {// FSD
    addr = ex_mem.alu_result;
    for (size_t i = 0; i < 8; ++i) {
      old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
    }
    memory_controller_.WriteDoubleWord(ex_mem.alu_result, ex_mem.reg2_val);
    for (size_t i = 0; i < 8; ++i) {
      new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
    }
  }

  if (old_bytes_vec!=new_bytes_vec) {
    current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
  }
  mem_wb.mem_data=memory_result_;
}

void Dynamic::WriteBack() {

  // if(mem_wb.valid) std::cout<<"write back is valid\n";
  // else std::cout<<"write back is invaldi\n";
  if(mem_wb.valid==true)
  {
    if(mem_wb.rd==0 && mem_wb.rd_type!=0 ){
      std::cout<<"hello world\n";
    }
  // uint8_t opcode = current_instruction_ & 0b1111111;
  // uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
  // uint8_t rd = (current_instruction_ >> 7) & 0b11111;
  // int32_t imm = ImmGenerator(current_instruction_);

  if (mem_wb.opcode == get_instr_encoding(Instruction::kecall).opcode && 
      mem_wb.funct3 == get_instr_encoding(Instruction::kecall).funct3) { // ecall
    return;
  }

  if (mem_wb.execute_type==1) { 
    //std::cout<<"memory result in writeback"<<mem_wb.mem_data<<"\n";// RV64 F
    WriteBackFloat();
    return;
  } else if (mem_wb.execute_type==2) {
    WriteBackDouble();
    return;
  } else if (mem_wb.opcode==0b1110011) { // CSR opcode
    WriteBackCsr();
    return;
  }

  uint64_t old_reg = registers_.ReadGpr(mem_wb.rd);
  unsigned int reg_index = mem_wb.rd;
  unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR


  if (mem_wb.regWrite) { 
    switch (mem_wb.opcode) {
      case get_instr_encoding(Instruction::kRtype).opcode: /* R-Type */
      case get_instr_encoding(Instruction::kItype).opcode: /* I-Type */
      case get_instr_encoding(Instruction::kauipc).opcode: /* AUIPC */ {
        registers_.WriteGpr(mem_wb.rd, mem_wb.alu_result);
        break;
      }
      case get_instr_encoding(Instruction::kLoadType).opcode: /* Load */ { 
        registers_.WriteGpr(mem_wb.rd, mem_wb.mem_data);
        break;
      }
      case get_instr_encoding(Instruction::kjalr).opcode: /* JALR */
      case get_instr_encoding(Instruction::kjal).opcode: /* JAL */ {
        registers_.WriteGpr(mem_wb.rd, mem_wb.alu_result);
        break;
      }
      case get_instr_encoding(Instruction::klui).opcode: /* LUI */ {
        //std::cout<<"lui in writeback "<<mem_wb.alu_result<<"\n";
        //std::cout<<" lui in wirte rd "<<+mem_wb.rd<<"\n";
        registers_.WriteGpr(mem_wb.rd, mem_wb.alu_result);
        break;
      }
      default: break;
    }
  }

  if (mem_wb.opcode==get_instr_encoding(Instruction::kjal).opcode) /* JAL */ {
    // Updated in Execute()
  }
  if (mem_wb.opcode==get_instr_encoding(Instruction::kjalr).opcode) /* JALR */ {
    // registers_.WriteGpr(rd, return_address_); // Write back to rs1
    // Updated in Execute()
  }

  uint64_t new_reg = registers_.ReadGpr(mem_wb.rd);
  if (old_reg!=new_reg) {
    current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
  }
  }
}

void Dynamic::WriteBackFloat() {
  // uint8_t opcode = current_instruction_ & 0b1111111;
  // uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
  // uint8_t rd = (current_instruction_ >> 7) & 0b11111;

  uint64_t old_reg = 0;
  unsigned int reg_index = mem_wb.rd;
  unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
  uint64_t new_reg = 0;

  if (mem_wb.regWrite==true) {
    switch(mem_wb.funct7) {
      // write to GPR
      case get_instr_encoding(Instruction::kfle_s).funct7: // f(eq|lt|le).s
      case get_instr_encoding(Instruction::kfcvt_w_s).funct7: // fcvt.(w|wu|l|lu).s
      case get_instr_encoding(Instruction::kfmv_x_w).funct7: // fmv.x.w , fclass.s
      {
        old_reg = registers_.ReadGpr(mem_wb.rd);
        registers_.WriteGpr(mem_wb.rd, mem_wb.alu_result);
        new_reg = mem_wb.alu_result;
        reg_type = 0; // GPR
        break;
      }

      // write to FPR
      default: {
        switch (mem_wb.opcode) {
          case get_instr_encoding(Instruction::kflw).opcode: {
            old_reg = registers_.ReadFpr(mem_wb.rd);
            std::cout<<"to wtrite "<<mem_wb.mem_data<<" \n";
            registers_.WriteFpr(mem_wb.rd, mem_wb.mem_data);
            new_reg = mem_wb.mem_data;
            reg_type = 2; // FPR
            break;
          }

          default: {
            old_reg = registers_.ReadFpr(mem_wb.rd);
            registers_.WriteFpr(mem_wb.rd, mem_wb.alu_result);
            new_reg = mem_wb.alu_result;
            reg_type = 2; // FPR
            break;
          }
        }
      }
    }

    if (mem_wb.funct7==0b1010000
        || mem_wb.funct7==0b1100000
        || mem_wb.funct7==0b1110000) { // f(eq|lt|le).s, fcvt.(w|wu|l|lu).s
      old_reg = registers_.ReadGpr(mem_wb.rd);
      registers_.WriteGpr(mem_wb.rd, mem_wb.alu_result);
      new_reg = mem_wb.alu_result;
      reg_type = 0; // GPR

    }
    // write to FPR
    else if (mem_wb.opcode==get_instr_encoding(Instruction::kflw).opcode) {
      old_reg = registers_.ReadFpr(mem_wb.rd);
      std::cout<<"to wtrite if else"<<mem_wb.mem_data<<" \n";
      registers_.WriteFpr(mem_wb.rd, mem_wb.mem_data);
      new_reg = mem_wb.mem_data;
      reg_type = 2; // FPR
    } else {
      old_reg = registers_.ReadFpr(mem_wb.rd);
      //if(mem_wb.rd_type==1)
      registers_.WriteFpr(mem_wb.rd, mem_wb.alu_result);
      new_reg = mem_wb.alu_result;
      reg_type = 2; // FPR
    }
  }

  if (old_reg!=new_reg) {
    current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
  }
}

void Dynamic::WriteBackDouble() {
  // uint8_t opcode = current_instruction_ & 0b1111111;
  // uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
  // uint8_t rd = (current_instruction_ >> 7) & 0b11111;

  uint64_t old_reg = 0;
  unsigned int reg_index = mem_wb.rd;
  unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
  uint64_t new_reg = 0;

  if (mem_wb.regWrite==true) {
    // write to GPR
    if (mem_wb.funct7==0b1010001
        || mem_wb.funct7==0b1100001
        || mem_wb.funct7==0b1110001) { // f(eq|lt|le).d, fcvt.(w|wu|l|lu).d
      old_reg = registers_.ReadGpr(mem_wb.rd);
      registers_.WriteGpr(mem_wb.rd, mem_wb.alu_result);
      new_reg = mem_wb.alu_result;
      reg_type = 0; // GPR
    }
      // write to FPR
    else if (mem_wb.opcode==0b0000111) {
      old_reg = registers_.ReadFpr(mem_wb.rd);
      registers_.WriteFpr(mem_wb.rd, mem_wb.mem_data);
      new_reg = mem_wb.mem_data;
      reg_type = 2; // FPR
    } else {
      old_reg = registers_.ReadFpr(mem_wb.rd);
      registers_.WriteFpr(mem_wb.rd, mem_wb.alu_result);
      new_reg = mem_wb.alu_result;
      reg_type = 2; // FPR
    }
  }

  if (old_reg!=new_reg) {
    current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
  }

  return;
}

void Dynamic::WriteBackCsr() {
  uint8_t rd = (current_instruction_ >> 7) & 0b11111;
  uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

  switch (funct3) {
    case get_instr_encoding(Instruction::kcsrrw).funct3: { // CSRRW
      registers_.WriteGpr(rd, csr_old_value_);
      registers_.WriteCsr(csr_target_address_, csr_write_val_);
      break;
    }
    case get_instr_encoding(Instruction::kcsrrs).funct3: { // CSRRS
      registers_.WriteGpr(rd, csr_old_value_);
      if (csr_write_val_!=0) {
        registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_write_val_);
      }
      break;
    }
    case get_instr_encoding(Instruction::kcsrrc).funct3: { // CSRRC
      registers_.WriteGpr(rd, csr_old_value_);
      if (csr_write_val_!=0) {
        registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_write_val_);
      }
      break;
    }
    case get_instr_encoding(Instruction::kcsrrwi).funct3: { // CSRRWI
      registers_.WriteGpr(rd, csr_old_value_);
      registers_.WriteCsr(csr_target_address_, csr_uimm_);
      break;
    }
    case get_instr_encoding(Instruction::kcsrrsi).funct3: { // CSRRSI
      registers_.WriteGpr(rd, csr_old_value_);
      if (csr_uimm_!=0) {
        registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_uimm_);
      }
      break;
    }
    case get_instr_encoding(Instruction::kcsrrci).funct3: { // CSRRCI
      registers_.WriteGpr(rd,csr_old_value_);
      if (csr_uimm_!=0) {
        registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_uimm_);
      }
      break;
    }
  }

}

void Dynamic::Run() {
  ClearStop();
  uint64_t instruction_executed = 0;
  int count=1;
  bool prev_stall=false;
  while (!stop_requested_  && count<500 &&(program_counter_  < program_size_ + 16)) {
    //if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
    //break

    bool stall=false;
    bool branch_stall=false;

    //std::cout<<"count "<<count<<"\n\n";
    //
    std::cout<<"Current PC "<<program_counter_<<"\n";
    if(mem_wb.valid==true)
    instruction_executed++;
    if(prev_stall) id_ex.valid=false;

    WriteBack();
    WriteMemory();
    Execute();
    //Control_Hazard();
    //std::cout<<"branch flag after execute "<<id_ex.branch_flag<<"\n";
    Check_Prediction();
    Decode();
    //std::cout<<"pc after decode"<<program_counter_<<"\n";
    Forward_Data();
    Branch_Prediction();
    //std::cout<<"pc after update"<<program_counter_<<"\n";
    Fetch();





    //if(stall) std::cout<<"HALOOOOOOOOOOOOOOOOOO\n";

    //else
    //if_id.valid=false;
    // --- Cycle End ---
    std::cout << "\n================= CYCLE " << std::dec << cycle_s_ << " END =================\n";
    std::cout << "Current PC: 0x" << std::hex << program_counter_ << std::dec << "\n";

    // --- IF/ID Register State ---
    // (Note: rs1, rs2, rd, imm are not yet known in this stage)
    std::cout << "IF/ID: "
              << "valid: " << if_id.valid
              << " | PC: 0x" << std::hex << if_id.pc
              << " | Instr: 0x" << if_id.instruction << std::dec
              << "\n";

    // --- ID/EX Register State ---
    std::cout << "ID/EX: "
              << "valid: " << id_ex.valid
              << " | rs1: " << (unsigned int)id_ex.rs1  // Cast to int to print number, not char
              << " | rs1_type: "<< (unsigned int)id_ex.rs1_type
              << " | rs2: " << (unsigned int)id_ex.rs2
              << " | rs2_type: "<< (unsigned int)id_ex.rs2_type
              << " | rd: " << (unsigned int)id_ex.rd
              << " | rd_type: "<< (unsigned int)id_ex.rd_type
              << " | opcode: "<<(unsigned int)id_ex.opcode
              << " | funct7: "<<(unsigned int)id_ex.funct7
              << " | predict: "<<(unsigned int)id_ex.branch_predicted
              << " | branch_flag: "<<(unsigned int)id_ex.branch_flag
              << " | branch: "<<(unsigned int)id_ex.branch
              << " | imm: 0x" << std::hex << id_ex.imm << std::dec
              << "\n";

    // --- EX/MEM Register State ---
    // (Note: rs1, rs2, imm are used and gone; rd is passed through)
    std::cout << "EX/MEM: "
              << "valid: " << ex_mem.valid
              << " | rd: " << (unsigned int)ex_mem.rd
              << " | rd_type: "<< (unsigned int)ex_mem.rd_type
              << " | ALU_Result: 0x" << std::hex << ex_mem.alu_result << std::dec
              << " | regWrite: " << ex_mem.regWrite
              << " | memRead: " << ex_mem.memRead

              << "\n";

    // --- MEM/WB Register State ---
    // (Note: rd is passed through for the final write)
    uint64_t write_data = mem_wb.memToReg ? mem_wb.mem_data : mem_wb.alu_result;
    std::cout << "MEM/WB: "
              << "valid: " << mem_wb.valid
              << " | rd: " << (unsigned int)mem_wb.rd
              << " | rd_type: "<< (unsigned int)mem_wb.rd_type
              << " | WriteData: 0x" << std::hex << write_data << std::dec
              << " | regWrite: " << mem_wb.regWrite
              << "\n";
    std::cout << "====================================================\n\n";


    // This line should already be in your code:
    cycle_s_++;

    instructions_retired_++;
    count++;
    //std::cout<<"rd "<<+id_ex.rd<<" imm "<<id_ex.imm<<"\n\n";
    //std::cout << "Program Counter: " << program_counter_ << std::endl;
  }
  if (program_counter_ >= program_size_) {
    std::cout << "VM_PROGRAM_END" << std::endl;
    output_status_ = "VM_PROGRAM_END";
  }
  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
}

void Dynamic::DebugRun() {
  ClearStop();
  uint64_t instruction_executed = 0;
  while (!stop_requested_ && program_counter_ < program_size_) {
    if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
      break;
    current_delta_.old_pc = program_counter_;
    if (std::find(breakpoints_.begin(), breakpoints_.end(), program_counter_) == breakpoints_.end()) {
      Fetch();
      Decode();
      Execute();
      WriteMemory();
      WriteBack();
      instructions_retired_++;
      instruction_executed++;
      cycle_s_++;
      std::cout << "Program Counter: " << program_counter_ << std::endl;

      current_delta_.new_pc = program_counter_;
      // history_.push(current_delta_);
      undo_stack_.push(current_delta_);
      while (!redo_stack_.empty()) {
        redo_stack_.pop();
      }
      current_delta_ = StepDelta();
      if (program_counter_ < program_size_) {
        std::cout << "VM_STEP_COMPLETED" << std::endl;
        output_status_ = "VM_STEP_COMPLETED";
      } else if (program_counter_ >= program_size_) {
        std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
        output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
      }
      DumpRegisters(globals::registers_dump_file_path, registers_);
      DumpState(globals::vm_state_dump_file_path);

      unsigned int delay_ms = vm_config::config.getRunStepDelay();
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
      
    } else {
      std::cout << "VM_BREAKPOINT_HIT " << program_counter_ << std::endl;
      output_status_ = "VM_BREAKPOINT_HIT";
      break;
    }
  }
  if (program_counter_ >= program_size_) {
    std::cout << "VM_PROGRAM_END" << std::endl;
    output_status_ = "VM_PROGRAM_END";
  }
  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
}

void Dynamic::Step() {
  current_delta_.old_pc = program_counter_;
  if (program_counter_ < program_size_) {
    Fetch();
    Decode();
    Execute();
    WriteMemory();
    WriteBack();
    instructions_retired_++;
    cycle_s_++;
    std::cout << "Program Counter: " << std::hex << program_counter_ << std::dec << std::endl;

    current_delta_.new_pc = program_counter_;

    // history_.push(current_delta_);

    undo_stack_.push(current_delta_);
    while (!redo_stack_.empty()) {
      redo_stack_.pop();
    }

    current_delta_ = StepDelta();


    if (program_counter_ < program_size_) {
      std::cout << "VM_STEP_COMPLETED" << std::endl;
      output_status_ = "VM_STEP_COMPLETED";
    } else if (program_counter_ >= program_size_) {
      std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
      output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
    }

  } else if (program_counter_ >= program_size_) {
    std::cout << "VM_PROGRAM_END" << std::endl;
    output_status_ = "VM_PROGRAM_END";
  }
  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
}

void Dynamic::Undo() {
  if (undo_stack_.empty()) {
    std::cout << "VM_NO_MORE_UNDO" << std::endl;
    output_status_ = "VM_NO_MORE_UNDO";
    return;
  }

  StepDelta last = undo_stack_.top();
  undo_stack_.pop();

  // if (!history_.can_undo()) {
  //     std::cout << "Nothing to undo.\n";
  //     return;
  // }

  // StepDelta last = history_.undo();

  for (const auto &change : last.register_changes) {
    switch (change.reg_type) {
      case 0: { // GPR
        registers_.WriteGpr(change.reg_index, change.old_value);
        break;
      }
      case 1: { // CSR
        registers_.WriteCsr(change.reg_index, change.old_value);
        break;
      }
      case 2: { // FPR
        registers_.WriteFpr(change.reg_index, change.old_value);
        break;
      }
      default:std::cerr << "Invalid register type: " << change.reg_type << std::endl;
        break;
    }
  }

  for (const auto &change : last.memory_changes) {
    for (size_t i = 0; i < change.old_bytes_vec.size(); ++i) {
      memory_controller_.WriteByte(change.address + i, change.old_bytes_vec[i]);
    }
  }

  program_counter_ = last.old_pc;
  instructions_retired_--;
  cycle_s_--;
  std::cout << "Program Counter: " << program_counter_ << std::endl;

  redo_stack_.push(last);

  output_status_ = "VM_UNDO_COMPLETED";
  std::cout << "VM_UNDO_COMPLETED" << std::endl;

  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
}

void Dynamic::Redo() {
  if (redo_stack_.empty()) {
    std::cout << "VM_NO_MORE_REDO" << std::endl;
    return;
  }

  StepDelta next = redo_stack_.top();
  redo_stack_.pop();

  // if (!history_.can_redo()) {
  //       std::cout << "Nothing to redo.\n";
  //       return;
  //   }

  //   StepDelta next = history_.redo();

  for (const auto &change : next.register_changes) {
    switch (change.reg_type) {
      case 0: { // GPR
        registers_.WriteGpr(change.reg_index, change.new_value);
        break;
      }
      case 1: { // CSR
        registers_.WriteCsr(change.reg_index, change.new_value);
        break;
      }
      case 2: { // FPR
        registers_.WriteFpr(change.reg_index, change.new_value);
        break;
      }
      default:std::cerr << "Invalid register type: " << change.reg_type << std::endl;
        break;
    }
  }

  for (const auto &change : next.memory_changes) {
    for (size_t i = 0; i < change.new_bytes_vec.size(); ++i) {
      memory_controller_.WriteByte(change.address + i, change.new_bytes_vec[i]);
    }
  }

  program_counter_ = next.new_pc;
  instructions_retired_++;
  cycle_s_++;
  DumpRegisters(globals::registers_dump_file_path, registers_);
  DumpState(globals::vm_state_dump_file_path);
  std::cout << "Program Counter: " << program_counter_ << std::endl;
  undo_stack_.push(next);

}

void Dynamic::Reset() {
  program_counter_ = 0;
  instructions_retired_ = 0;
  cycle_s_ = 0;
  registers_.Reset();
  memory_controller_.Reset();
  control_unit_.Reset();
  branch_flag_ = false;
  next_pc_ = 0;
  execution_result_ = 0;
  memory_result_ = 0;

  return_address_ = 0;
  csr_target_address_ = 0;
  csr_old_value_ = 0;
  csr_write_val_ = 0;
  csr_uimm_ = 0;
  current_delta_.register_changes.clear();
  current_delta_.memory_changes.clear();
  current_delta_.old_pc = 0;
  current_delta_.new_pc = 0;
  undo_stack_ = std::stack<StepDelta>();
  redo_stack_ = std::stack<StepDelta>();

}