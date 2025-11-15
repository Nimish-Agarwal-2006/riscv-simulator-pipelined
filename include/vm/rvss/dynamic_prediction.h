/**
 * @file stages.h
 * @brief Pipeline Dynamic definition (formerly RVSS VM)
 * @author Vishank
 */

#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "vm/vm_base.h"
#include "rvss_control_unit.h"
#include "rvss_vm.h"

#include <stack>
#include <vector>
#include <iostream>
#include <cstdint>
#include <atomic>
#include <map>
#include <unordered_map>
// struct RegisterChange {
//   unsigned int reg_index;
//   unsigned int reg_type; // 0 for GPR, 1 for CSR, 2 for FPR
//   uint64_t old_value;
//   uint64_t new_value;
// };

// struct MemoryChange {
//   uint64_t address;
//   std::vector<uint8_t> old_bytes_vec; 
//   std::vector<uint8_t> new_bytes_vec; 
// };

// struct StepDelta {
//   uint64_t old_pc;
//   uint64_t new_pc;
//   std::vector<RegisterChange> register_changes;
//   std::vector<MemoryChange> memory_changes;
// };

class Dynamic : public VmBase {
 public:
  RVSSControlUnit control_unit_;
  std::atomic<bool> stop_requested_ = false;

  std::stack<StepDelta> undo_stack_;
  std::stack<StepDelta> redo_stack_;
  std::map<uint64_t,int> branch_map_;
  StepDelta current_delta_;

  // Intermediate variables
  int64_t execution_result_{};
  int64_t memory_result_{};
  uint64_t return_address_{};

  bool branch_flag_ = false;
  int64_t next_pc_{}; // for jal, jalr, etc.

  // CSR intermediate variables
  uint16_t csr_target_address_{};
  uint64_t csr_old_value_{};
  uint64_t csr_write_val_{};
  uint8_t csr_uimm_{};

  // --- Pipeline Stage Functions ---
  void Forward_Data();
  void Branch_Control();
  void Branch_Prediction();
  void Check_Prediction();
  void Fetch();
  void Decode();
  void Execute();
  void ExecuteFloat();
  void ExecuteDouble();
  void ExecuteCsr();
  void HandleSyscall();

  void WriteMemory();
  void WriteMemoryFloat();
  void WriteMemoryDouble();

  void WriteBack();
  void WriteBackFloat();
  void WriteBackDouble();
  void WriteBackCsr();

  // --- Lifecycle Methods ---
  Dynamic();
  ~Dynamic();

  void Run() override;
  void DebugRun() override;
  void Step() override;
  void Undo() override;
  void Redo() override;
  void Reset() override;

  // --- Control Helpers ---
  void RequestStop() {
    stop_requested_ = true;
  }

  bool IsStopRequested() const {
    return stop_requested_;
  }

  void ClearStop() {
    stop_requested_ = false;
  }

  void PrintType() {
    std::cout << "stages" << std::endl;
  }
};

#endif // FORWARD_H
