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

struct FlagMasks {
  u8 Word{};
  u8 SignExtend{};

  bool IsWord(u8 opcodeByte) { return (opcodeByte & Word) > 0; }
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

struct Immediate {
  u16 Value;
  u16 ByteCount;
};

struct Operand {
  OperandType Type{};
  union {
    Register RegisterValue;
    Register RegisterAddress;
    AddressCalculation AddressCalculation;
    Immediate Immediate;
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
      printf("%s %u", Immediate.ByteCount == 2 ? "word" : "byte",
             Immediate.Value);
      break;
    case OperandType::AddressCalculation: {
      if (AddressCalculation.RegisterCount == 0) {
        printf("[%u]", AddressCalculation.Displacement);
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

Operand GetMoveModeOperand(SimulatorState &state, u8 mode, u8 rm, bool word) {
  Operand operand{};
  if (Mode(mode) == Mode::Register) {
    operand.Type = OperandType::RegisterValue;
    operand.RegisterValue = RegisterFromFlags(rm, word);
  } else {
    operand.Type = OperandType::AddressCalculation;
    operand.AddressCalculation.RegisterCount = 0;
    operand.AddressCalculation.Displacement = 0;

    if (Mode(mode) == Mode::Memory8BitDisplacement) {
      operand.AddressCalculation.Displacement =
          SignExtend(state.AdvanceInstructionByte());
    } else if (Mode(mode) == Mode::Memory16BitDisplacement ||
               (Mode(mode) == Mode::NoDisplacement && rm == 0b110)) {
      operand.AddressCalculation.Displacement = state.AdvanceInstructionWord();
    }

    switch (rm) {
    case 0b000:
      operand.AddressCalculation.RegisterA = Register::BX;
      operand.AddressCalculation.RegisterB = Register::SI;
      operand.AddressCalculation.RegisterCount = 2;
      break;
    case 0b001:
      operand.AddressCalculation.RegisterA = Register::BX;
      operand.AddressCalculation.RegisterB = Register::DI;
      operand.AddressCalculation.RegisterCount = 2;
      break;
    case 0b010:
      operand.AddressCalculation.RegisterA = Register::BP;
      operand.AddressCalculation.RegisterB = Register::SI;
      operand.AddressCalculation.RegisterCount = 2;
      break;
    case 0b011:
      operand.AddressCalculation.RegisterA = Register::BP;
      operand.AddressCalculation.RegisterB = Register::DI;
      operand.AddressCalculation.RegisterCount = 2;
      break;
    case 0b100:
      operand.AddressCalculation.RegisterA = Register::SI;
      operand.AddressCalculation.RegisterCount = 1;
      break;
    case 0b101:
      operand.AddressCalculation.RegisterA = Register::DI;
      operand.AddressCalculation.RegisterCount = 1;
      break;
    case 0b110:
      if (Mode(mode) != Mode::NoDisplacement) {
        operand.AddressCalculation.RegisterA = Register::BP;
        operand.AddressCalculation.RegisterCount = 1;
      }
      break;
    case 0b111:
      operand.AddressCalculation.RegisterA = Register::BX;
      operand.AddressCalculation.RegisterCount = 1;
      break;
    }
  }

  return operand;
}

Operand DecodeImmediate(SimulatorState &state, u8 opcodeByte, FlagMasks masks) {
  bool word = masks.IsWord(opcodeByte);
  bool signExtend = (opcodeByte & masks.SignExtend) > 0;

  Operand operand{};
  operand.Type = OperandType::Immediate;
  if (word && signExtend) {
    operand.Immediate.ByteCount = 1;
    operand.Immediate.Value = SignExtend(state.AdvanceInstructionByte());
  } else if (word) {
    operand.Immediate.ByteCount = 2;
    operand.Immediate.Value = state.AdvanceInstructionWord();
  } else {
    operand.Immediate.ByteCount = 1;
    operand.Immediate.Value = state.AdvanceInstructionByte();
  }

  return operand;
}

struct MoveDecode {
  u8 Mode{};
  u8 Reg{};
  u8 RM{};
};
MoveDecode GetMoveModeAndRM(u8 data) {
  constexpr u8 ModeMask = 0b1100'0000;
  constexpr u8 ModeShift = 6;
  constexpr u8 RegMask = 0b0011'1000;
  constexpr u8 RegShift = 3;
  constexpr u8 RMMask = 0b0000'0111;
  constexpr u8 RMShift = 0;

  MoveDecode result{
      .Mode = static_cast<u8>((data & ModeMask) >> ModeShift),
      .Reg = static_cast<u8>((data & RegMask) >> RegShift),
      .RM = static_cast<u8>((data & RMMask) >> RMShift),
  };

  return result;
}

void OperationToFrom(const char *mneumonic, u8 opcodeByte,
                     SimulatorState &state) {
  constexpr u8 DirectionMask = 0b0000'0010;
  constexpr u8 WordMask = 0b0000'0001;

  u8 byte2 = state.AdvanceInstructionByte();

  u8 direction = (opcodeByte & DirectionMask) == DirectionMask;
  MoveDecode decode = GetMoveModeAndRM(byte2);

  bool word = (opcodeByte & WordMask) == WordMask;

  Operand dest{};
  dest.Type = OperandType::RegisterValue;
  dest.RegisterValue = RegisterFromFlags(decode.Reg, word);

  Operand source = GetMoveModeOperand(state, decode.Mode, decode.RM, word);

  if (direction == 0) {
    std::swap(dest, source);
  }

  PrintInstruction(mneumonic, dest, source);
}

void MoveToFrom(u8 opcodeByte, SimulatorState &state) {
  OperationToFrom("mov", opcodeByte, state);
}

void OperationImmediateToRegister(const char *mneumonic, u8 opcodeByte,
                                  SimulatorState &state, FlagMasks masks) {
  constexpr u8 RegMask = 0b0000'0111;
  bool word = (opcodeByte & masks.Word) > 0;
  u8 reg = (opcodeByte & RegMask);

  Operand src = DecodeImmediate(state, opcodeByte, masks);
  Operand dest{.Type = OperandType::RegisterValue,
               .RegisterValue = RegisterFromFlags(reg, word)};

  PrintInstruction(mneumonic, dest, src);
}

void MoveImmediateToRegister(u8 opcodeByte, SimulatorState &state) {
  FlagMasks masks{.Word = 1};
  OperationImmediateToRegister("mov", opcodeByte, state, masks);
}

void OperationBasedOnMode(const char *mneumonic, u8 opcodeByte,
                          SimulatorState &state, struct FlagMasks masks) {
  bool word = (opcodeByte & masks.Word) > 0;
  u8 byte2 = state.AdvanceInstructionByte();
  MoveDecode decode = GetMoveModeAndRM(byte2);

  Operand destination = GetMoveModeOperand(state, decode.Mode, decode.RM, word);
  Operand source = DecodeImmediate(state, opcodeByte, masks);
  PrintInstruction(mneumonic, destination, source);
}

void MoveBasedOnMode(u8 opcodeByte, SimulatorState &state) {
  FlagMasks masks{.Word = 1};
  OperationBasedOnMode("mov", opcodeByte, state, masks);
}

void OperationMemoryToFromAccumulator(const char *mneumonic, u8 opcodeByte,
                                      SimulatorState &state,
                                      bool toAccumulator) {
  bool word = (opcodeByte & 1) == 1;

  Operand dest{
      .Type = OperandType::RegisterValue,
      .RegisterValue = RegisterFromFlags(0, word),
  };

  Operand source{
      .Type = OperandType::AddressCalculation,
      .AddressCalculation =
          {
              .Displacement = state.AdvanceInstructionWord(),
              .RegisterCount = 0,
          },
  };

  if (!toAccumulator) {
    std::swap(dest, source);
  }

  PrintInstruction(mneumonic, dest, source);
}

void MoveMemoryToAccumulator(u8 opcodeByte, SimulatorState &state) {
  OperationMemoryToFromAccumulator("mov", opcodeByte, state, true);
}

void MoveAccumlatorToMemory(u8 opcodeByte, SimulatorState &state) {
  OperationMemoryToFromAccumulator("mov", opcodeByte, state, false);
}

void AddToFrom(u8 opcodeByte, SimulatorState &state) {
  OperationToFrom("add", opcodeByte, state);
}

void AddBasedOnMode(u8 opcodeByte, SimulatorState &state) {
  FlagMasks masks{.Word = 1, .SignExtend = 2};
  OperationBasedOnMode("add", opcodeByte, state, masks);
}

void AddImmediate(u8 opcodeByte, SimulatorState &state) {
  FlagMasks masks{.Word = 1};

  Operand dest{};
  dest.Type = OperandType::RegisterValue;
  dest.RegisterValue = masks.IsWord(opcodeByte) ? Register::AX : Register::AL;

  Operand source = DecodeImmediate(state, opcodeByte, masks);

  PrintInstruction("add", dest, source);
}

const OpcodeMatcher OpcodeMatchers[] = {
    {.Mask = 0b1111'1100, .Opcode = 0b1000'1000, .Func = MoveToFrom},
    {.Mask = 0b1111'0000,
     .Opcode = 0b1011'0000,
     .Func = MoveImmediateToRegister},
    {.Mask = 0b1111'1110, .Opcode = 0b1100'0110, .Func = MoveBasedOnMode},
    {.Mask = 0b1111'1110,
     .Opcode = 0b1010'0000,
     .Func = MoveMemoryToAccumulator},
    {.Mask = 0b1111'1110,
     .Opcode = 0b1010'0010,
     .Func = MoveAccumlatorToMemory},

    {.Mask = 0b1111'1100, .Opcode = 0b0000'0000, .Func = AddToFrom},
    {.Mask = 0b1111'1100, .Opcode = 0b1000'0000, .Func = AddBasedOnMode},
    {.Mask = 0b1111'1110, .Opcode = 0b0000'0100, .Func = AddImmediate},

    // TODO subtraction ops
    // {.Mask = 0b1111'1100, .Opcode = 0b0010'1000, .Func = AddToFrom},
    // {.Mask = 0b1111'1100, .Opcode = 0b1000'0000, .Func = AddBasedOnMode},
    // {.Mask = 0b1111'1110, .Opcode = 0b0010'1100, .Func = AddImmediate},
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
