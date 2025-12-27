# 8-bit-Harvard-Pipelined-RISC-Processor-
Design and implementation of a simple 8-bit pipelined RISC processor with FSM-based control unit, supporting interrupts and a custom ISA. Course project for ELC3030 – Advanced Processor Architecture, Cairo University.
# 8-bit Pipelined RISC Processor – ELC3030

This repository contains the design and implementation of a simple **8-bit pipelined processor**
developed as part of the **ELC3030 – Advanced Processor Architecture** course at
the Faculty of Engineering, Cairo University.

The processor follows a RISC-like Instruction Set Architecture (ISA) and is implemented using
**Verilog HDL**.

---

## 📌 Project Overview

- 8-bit processor with **4 general-purpose registers** (R0–R3)
- R3 acts as the **Stack Pointer (SP)**
- 256-byte byte-addressable memory
- FSM-based **Control Unit**
- Support for **interrupt handling**
- Single shared memory (Von Neumann architecture – if implemented)
- Designed and verified through simulation and waveform analysis

---

## 🧠 Processor Features

- **Instruction Length:** 1-byte and 2-byte instructions
- **Instruction Formats:**
  - A-format (Arithmetic / Logic / Stack / I/O)
  - B-format (Branch / Jump / Call / Interrupt)
  - L-format (Load / Store / Immediate)
- **Condition Code Register (CCR):**
  - Z (Zero)
  - N (Negative)
  - C (Carry)
  - V (Overflow)

---

## 🧾 Supported Instructions

The processor supports a wide range of instructions including:

- Arithmetic: `ADD`, `SUB`, `INC`, `DEC`, `NEG`
- Logic: `AND`, `OR`, `NOT`
- Data Transfer: `MOV`, `LDM`, `LDD`, `STD`, `LDI`, `STI`
- Control Flow: `JMP`, `JZ`, `JN`, `JC`, `JV`, `CALL`, `RET`, `LOOP`
- Stack & I/O: `PUSH`, `POP`, `IN`, `OUT`
- Interrupt Handling: `RTI`

(Refer to the project specification for full ISA details.)

---

## 🛠️ Tools & Simulation

- **HDL:** Verilog
- **Simulation:** ModelSim / EDA Playground
- **Verification:** Waveform analysis using VCD dumps


