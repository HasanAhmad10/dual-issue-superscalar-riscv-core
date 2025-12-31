# dual-issue-superscalar-riscv-core
A Verilog implementation of a RISC-V Core (CAO Semester Project)

Here is a short, professional `README.md` file you can use for your repository.

You can create a new file named `README.md` in your repository and paste this content into it.

---

# RISC-V Core (Verilog)

A synthesizable, pipelined 32-bit RISC-V processor core implemented in Verilog. This project targets the **RV32IM** instruction set architecture, supporting hardware multiplication/division and privileged architecture specifications.

## Features

* **Architecture:** RV32IM (Integer + Multiply/Divide Extension).
* **Pipeline:** 5-stage pipeline (Fetch, Decode, Issue, Execute, Writeback).
* **Memory Management:** Optional SV32 MMU support for virtual memory.
* **Privilege Modes:** Supports Machine, Supervisor, and User modes.
* **Bus Interface:** Standard Load/Store Unit (LSU) with decoupled memory interfaces.
* **Simulation:** Includes support for Verilator-based simulation traces.

## Core Modules

* `riscv_core.v`: Top-level processor core.
* `riscv_fetch.v`: Instruction fetch unit with simple branch prediction buffering.
* `riscv_decode.v` / `riscv_issue.v`: Instruction decoding and issue logic.
* `riscv_exec.v`: Execution unit (ALU, Branch resolution).
* `riscv_lsu.v`: Load/Store Unit handling data memory access and alignment.
* `riscv_csr.v`: Control and Status Register file handling exceptions and interrupts.
* `riscv_mmu.v`: Memory Management Unit (TLB implementation).

## Getting Started

### Prerequisites

* Verilog Simulator (e.g., Verilator, Icarus Verilog, or ModelSim).
* RISC-V Toolchain (for compiling C/Assembly tests).

### Simulation

This core is designed to be compatible with **Verilator**. To run simulations, ensure all `.v` files are included in your project hierarchy, with `riscv_core.v` acting as the top-level module.

### Configuration

Key parameters in `riscv_core.v` can be adjusted for your specific implementation:

* `SUPPORT_MULDIV`: Enable/Disable hardware multiplier/divider.
* `SUPPORT_MMU`: Enable/Disable virtual memory logic.
* `MEM_CACHE_ADDR_MIN/MAX`: Define cacheable memory regions.
