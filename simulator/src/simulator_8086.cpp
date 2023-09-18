#include "simulator_8086.h"
#include <cassert>
#include <cstdio>
#include <cstring>

namespace {

typedef void (*OperationExecutor)(u8 opcodeByte, SimulatorState &state);
struct OpcodeMatcher {
  u8 Mask{};
  u8 Opcode{};
  OperationExecutor Func{};
};

enum class Mode : u8 {
  NoDisplacement = 0b00,
  Memory8BitDisplacement = 0b01,
  Memory16BitDisplacement = 0b10,
  Register = 0b11,
};

enum class Register {
  AL = 0b0000,
  CL = 0b0001,
  DL = 0b0010,
  BL = 0b0011,
  AH = 0b0100,
  CH = 0b0101,
  DH = 0b0110,
  BH = 0b0111,

  AX = 0b1000,
  CX = 0b1001,
  DX = 0b1010,
  BX = 0b1011,
  SP = 0b1100,
  BP = 0b1101,
  SI = 0b1110,
  DI = 0b1111,
  MAX = DI
};

Register RegisterFromFlags(u8 reg, bool wide) {
  u8 code = (wide << 3) | reg;
  assert(Register(code) <= Register::MAX);
  return Register(code);
}

const char *GetRegisterName(Register reg) {
  switch (reg) {
  case Register::AL:
    return "al";
  case Register::CL:
    return "cl";
  case Register::DL:
    return "dl";
  case Register::BL:
    return "bl";
  case Register::AH:
    return "ah";
  case Register::CH:
    return "ch";
  case Register::DH:
    return "dh";
  case Register::BH:
    return "bh";

  case Register::AX:
    return "ax";
  case Register::CX:
    return "cx";
  case Register::DX:
    return "dx";
  case Register::BX:
    return "bx";
  case Register::SP:
    return "sp";
  case Register::BP:
    return "bp";
  case Register::SI:
    return "si";
  case Register::DI:
    return "di";
  }

  assert(false && "No matching register found!");
  return nullptr;
}

enum class OperandType {
  RegisterValue,
  RegisterAddress,
  AddressCalculation,
  Immediate,
};

struct AddressCalculation {
  Register RegisterA{};
  Register RegisterB{};
  u16 Displacement{};
  u8 RegisterCount{};
};

struct Operand {
  OperandType Type{};
  union {
    Register RegisterValue;
    Register RegisterAddress;
    AddressCalculation AddressCalculation;
    u16 Immediate;
  };

  void Print() {
    switch (Type) {
    case OperandType::RegisterValue:
      printf("%s", GetRegisterName(RegisterValue));
      break;
    case OperandType::RegisterAddress:
      printf("[%s]", GetRegisterName(RegisterValue));
      break;
    case OperandType::Immediate:
      printf("%u", Immediate);
      break;
    case OperandType::AddressCalculation: {
      if (AddressCalculation.RegisterCount == 0) {
        printf("%u", AddressCalculation.Displacement);
      } else {
        printf("[%s", GetRegisterName(AddressCalculation.RegisterA));
        if (AddressCalculation.RegisterCount == 2) {
          printf(" + %s", GetRegisterName(AddressCalculation.RegisterB));
        }
        if (AddressCalculation.Displacement > 0) {
          printf(" + %u", AddressCalculation.Displacement);
        }

        printf("]");
      }

    } break;
    }
  }
};

void PrintInstruction(const char *mneumonic, Operand dest, Operand source) {
  printf("%s ", mneumonic);
  dest.Print();
  printf(", ");
  source.Print();
  printf("\n");
}

u16 SignExtend(u8 byte) {
  if (byte & 0b1000'0000) {
    return u16(0xFF'00) | u16(byte);
  }

  return byte;
}

void MoveToFrom(u8 opcodeByte, SimulatorState &state) {
  constexpr u8 DirectionMask = 0b0000'0010;
  constexpr u8 WordMask = 0b0000'0001;

  constexpr u8 ModeMask = 0b1100'0000;
  constexpr u8 ModeShift = 6;
  constexpr u8 RegMask = 0b0011'1000;
  constexpr u8 RegShift = 3;
  constexpr u8 RMMask = 0b0000'0111;
  constexpr u8 RMShift = 0;

  u8 byte2 = state.AdvanceInstructionByte();

  u8 direction = (opcodeByte & DirectionMask) == DirectionMask;
  bool word = (opcodeByte & WordMask) == WordMask;

  u8 mode = (byte2 & ModeMask) >> ModeShift;
  u8 reg = (byte2 & RegMask) >> RegShift;
  u8 rm = (byte2 & RMMask) >> RMShift;

  Operand dest{};
  dest.Type = OperandType::RegisterValue;
  dest.RegisterValue = RegisterFromFlags(reg, word);

  Operand source{};
  if (Mode(mode) == Mode::Register) {
    source.Type = OperandType::RegisterValue;
    source.RegisterValue = RegisterFromFlags(rm, word);

  } else {
    source.Type = OperandType::AddressCalculation;
    source.AddressCalculation.RegisterCount = 0;
    source.AddressCalculation.Displacement = 0;

    if (Mode(mode) == Mode::Memory8BitDisplacement) {
      source.AddressCalculation.Displacement =
          SignExtend(state.AdvanceInstructionByte());
    } else if (Mode(mode) == Mode::Memory16BitDisplacement ||
               (Mode(mode) == Mode::NoDisplacement && rm == 0b110)) {
      source.AddressCalculation.Displacement = state.AdvanceInstructionWord();
    }

    switch (rm) {
    case 0b000:
      source.AddressCalculation.RegisterA = Register::BX;
      source.AddressCalculation.RegisterB = Register::SI;
      source.AddressCalculation.RegisterCount = 2;
      break;
    case 0b001:
      source.AddressCalculation.RegisterA = Register::BX;
      source.AddressCalculation.RegisterB = Register::DI;
      source.AddressCalculation.RegisterCount = 2;
      break;
    case 0b010:
      source.AddressCalculation.RegisterA = Register::BP;
      source.AddressCalculation.RegisterB = Register::SI;
      source.AddressCalculation.RegisterCount = 2;
      break;
    case 0b011:
      source.AddressCalculation.RegisterA = Register::BP;
      source.AddressCalculation.RegisterB = Register::DI;
      source.AddressCalculation.RegisterCount = 2;
      break;
    case 0b100:
      source.AddressCalculation.RegisterA = Register::SI;
      source.AddressCalculation.RegisterCount = 1;
      break;
    case 0b101:
      source.AddressCalculation.RegisterA = Register::DI;
      source.AddressCalculation.RegisterCount = 1;
      break;
    case 0b110:
      if (Mode(mode) != Mode::NoDisplacement) {
        source.AddressCalculation.RegisterA = Register::BP;
        source.AddressCalculation.RegisterCount = 1;
      }
      break;
    case 0b111:
      source.AddressCalculation.RegisterA = Register::BX;
      source.AddressCalculation.RegisterCount = 1;
      break;
    }
  }

  if (direction == 0) {
    std::swap(dest, source);
  }

  PrintInstruction("mov", dest, source);
}

void MoveImmediateToRegister(u8 opcodeByte, SimulatorState &state) {
  constexpr u8 WordMask = 0b0000'1000;
  constexpr u8 RegMask = 0b0000'0111;
  bool word = (opcodeByte & WordMask) == WordMask;
  u8 reg = (opcodeByte & RegMask);

  Operand src{
      .Type = OperandType::Immediate,
  };
  if (word) {
    src.Immediate = state.AdvanceInstructionWord();
  } else {
    src.Immediate = state.AdvanceInstructionByte();
  }

  Operand dest{.Type = OperandType::RegisterValue,
               .RegisterValue = RegisterFromFlags(reg, word)};
  PrintInstruction("mov", dest, src);
}

const OpcodeMatcher OpcodeMatchers[2] = {
    {.Mask = 0b1111'1100, .Opcode = 0b1000'1000, .Func = MoveToFrom},
    {.Mask = 0b1111'0000,
     .Opcode = 0b1011'0000,
     .Func = MoveImmediateToRegister},
};

bool FindOperation(u8 opcodeByte, OpcodeMatcher *output) {
  for (const auto &matcher : OpcodeMatchers) {
    if ((opcodeByte & matcher.Mask) == matcher.Opcode) {
      *output = matcher;
      return true;
    }
  }

  return false;
}

} // namespace

void Simulator8086::Execute() {
  for (;;) {
    if (mState.PC >= mState.CodeLength) {
      break;
    }

    u8 opcodeByte = mState.AdvanceInstructionByte();

    OpcodeMatcher opcode{};
    if (!FindOperation(opcodeByte, &opcode)) {
      printf("ERROR Unexpected opcode pattern %hhx\n", opcodeByte);
      return;
    }

    opcode.Func(opcodeByte, mState);
    if (mState.Error) {
      printf("ERROR Unexpected opcode pattern %hhx\n", opcodeByte);
      return;
    }
  }
}

Simulator8086::Simulator8086(const std::span<uint8_t> &code) {
  mState.CodeLength = code.size();
  mState.Code = new u8[code.size()];
  mState.PC = 0;

  memcpy(mState.Code, code.data(), mState.CodeLength);
}

Simulator8086::~Simulator8086() {
  delete[] mState.Code;
  mState.CodeLength = 0;
}
