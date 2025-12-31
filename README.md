# dual-issue-superscalar-riscv-core
A Verilog implementation of a RISC-V Core (CAO Semester Project)


# RISC-V Core (Verilog)

A synthesisable, pipelined 32-bit RISC-V processor core implemented in Verilog. This project targets the **RV32IM** instruction set architecture, supporting hardware multiplication/division and privileged architecture specifications.

## Getting Started

### Prerequisites

* Verilog Simulator (e.g., Verilator, Icarus Verilog, or ModelSim).
* RISC-V Toolchain (for compiling C/Assembly tests).

### Simulation

This core is designed to be compatible with **Verilator**. To run simulations, ensure all `.v` files are included in your project hierarchy, with `riscv_core.v` acting as the top-level module.

### Configuration

Key parameters in `riscv_core.v` can be adjusted for your specific implementation:

* `SUPPORT_MULDIV`: Enable/Disable hardware multiplier/divider.
