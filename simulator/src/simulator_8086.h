#pragma once
#include <cassert>
#include <cstdio>
#include <span>
#include <stdint.h>

typedef uint32_t u32;
typedef uint8_t u8;
typedef uint16_t u16;

struct SimulatorState {
  u8 *Code{};
  size_t CodeLength{};

  u32 PC{};
  bool Error{};

  u8 AdvanceInstructionByte() { return ReadByte(PC++); }
  u16 AdvanceInstructionWord() {
    u16 result = ReadWord(PC);
    PC += 2;
    return result;
  }

  u8 ReadByte(u32 address) {
    assert(address < CodeLength);

#ifdef DEBUG_INSTRUCTIONS
    printf("D %x\n", Code[address]);
#endif
    return Code[address];
  }

  u16 ReadWord(u32 address) {
    assert(address + 1 < CodeLength);
    u16 lower = Code[address + 0];
    u16 upper = Code[address + 1];

#ifdef DEBUG_INSTRUCTIONS
    printf("D %x\n", Code[address]);
    printf("D %x\n", Code[address + 1]);
#endif
    return lower | (upper << 8);
  }
};

struct Simulator8086 {
  Simulator8086(const std::span<uint8_t> &code);
  ~Simulator8086();
  void Execute();

private:
  SimulatorState mState{};
};
