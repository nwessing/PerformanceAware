#include "simulator_8086.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>

namespace {
struct InstructionVariables {
  bool Word{};
  bool SignExtend{};
  bool Direction{};
  u8 Reg{};
  u8 Mode{};
  u8 RM{};
};

typedef void (*OperationExecutor)(InstructionVariables variables,
                                  SimulatorState &state);

constexpr u16 InstructionVariableUnused = 16;
struct InstructionVariableOffsets {
  u16 Word{InstructionVariableUnused};
  u16 SignExtend{InstructionVariableUnused};
  u16 Direction{InstructionVariableUnused};
  u16 Reg{InstructionVariableUnused};
  u16 Mode{InstructionVariableUnused};
  u16 RM{InstructionVariableUnused};
};

enum class OpcodePartType { Constant, Variable };

struct OpcodePart {
  OpcodePartType Type{};
  u8 BitLength{};
  u8 ConstantPattern{};
};

struct OpcodeMatcher {
  std::array<OpcodePart, 5> Parts{};
  OperationExecutor Func{};
  InstructionVariableOffsets VariableOffsets{};
  u8 BitLength{};

  OpcodeMatcher(OperationExecutor func,
                InstructionVariableOffsets variableOffsets,
                std::initializer_list<OpcodePart> parts)
      : VariableOffsets(variableOffsets), Func(func) {

    assert(parts.size() <= 5);

    u32 partIndex = 0;
    for (const OpcodePart &part : parts) {
      Parts[partIndex] = part;
      BitLength += part.BitLength;
      ++partIndex;
    }
  }

  bool Matches(u16 opcode, u8 byteCount) const {
    if (BitLength > byteCount * 8) {
      return false;
    }

    u8 bitIndex = 0;
    for (const auto &part : Parts) {
      if (part.Type == OpcodePartType::Constant) {
        u16 pattern = u16(part.ConstantPattern);
        u16 mask = (u32(1u << part.BitLength) - 1u);
        if ((mask & (opcode >> (16u - bitIndex - part.BitLength))) != pattern) {
          return false;
        }
      }

      bitIndex += part.BitLength;
      if (bitIndex >= BitLength) {
        break;
      }
    }

    return true;
  }
};

OpcodePart ConstantPart(u8 bitLength, u8 pattern) {
  OpcodePart part{.Type = OpcodePartType::Constant,
                  .BitLength = bitLength,
                  .ConstantPattern = pattern};
  return part;
}
OpcodePart VariablePart(u8 bitLength) {
  OpcodePart part{.Type = OpcodePartType::Variable, .BitLength = bitLength};
  return part;
}

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

  bool IsSignExtension(u8 opcodeByte) { return (opcodeByte & SignExtend) > 0; }
};

Register RegisterFromFlags(u8 reg, bool word) {
  u8 code = (word << 3) | reg;
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

Operand DecodeImmediate(SimulatorState &state, InstructionVariables variables) {
  Operand operand{};
  operand.Type = OperandType::Immediate;
  if (variables.Word && variables.SignExtend) {
    operand.Immediate.ByteCount = 1;
    operand.Immediate.Value = SignExtend(state.AdvanceInstructionByte());
  } else if (variables.Word) {
    operand.Immediate.ByteCount = 2;
    operand.Immediate.Value = state.AdvanceInstructionWord();
  } else {
    operand.Immediate.ByteCount = 1;
    operand.Immediate.Value = state.AdvanceInstructionByte();
  }

  return operand;
}

void OperationToFrom(const char *mneumonic, InstructionVariables variables,
                     SimulatorState &state) {
  Operand dest{};
  dest.Type = OperandType::RegisterValue;
  dest.RegisterValue = RegisterFromFlags(variables.Reg, variables.Word);

  Operand source =
      GetMoveModeOperand(state, variables.Mode, variables.RM, variables.Word);

  if (!variables.Direction) {
    std::swap(dest, source);
  }

  PrintInstruction(mneumonic, dest, source);
}

void MoveToFrom(InstructionVariables variables, SimulatorState &state) {
  OperationToFrom("mov", variables, state);
}

void OperationImmediateToRegister(const char *mneumonic,
                                  InstructionVariables variables,
                                  SimulatorState &state) {

  Operand src = DecodeImmediate(state, variables);
  Operand dest{.Type = OperandType::RegisterValue,
               .RegisterValue =
                   RegisterFromFlags(variables.Reg, variables.Word)};

  PrintInstruction(mneumonic, dest, src);
}

void MoveImmediateToRegister(InstructionVariables variables,
                             SimulatorState &state) {
  OperationImmediateToRegister("mov", variables, state);
}

void OperationBasedOnMode(const char *mneumonic, InstructionVariables variables,
                          SimulatorState &state) {
  Operand destination =
      GetMoveModeOperand(state, variables.Mode, variables.RM, variables.Word);
  Operand source = DecodeImmediate(state, variables);
  PrintInstruction(mneumonic, destination, source);
}

void MoveImmediateToRegisterOrMemory(InstructionVariables InstructionVariables,
                                     SimulatorState &state) {
  OperationBasedOnMode("mov", InstructionVariables, state);
}

void OperationMemoryToFromAccumulator(const char *mneumonic,
                                      InstructionVariables variables,
                                      SimulatorState &state,
                                      bool toAccumulator) {
  Operand dest{
      .Type = OperandType::RegisterValue,
      .RegisterValue = RegisterFromFlags(0, variables.Word),
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

void MoveMemoryToAccumulator(InstructionVariables variables,
                             SimulatorState &state) {
  OperationMemoryToFromAccumulator("mov", variables, state, true);
}

void MoveAccumlatorToMemory(InstructionVariables variables,
                            SimulatorState &state) {
  OperationMemoryToFromAccumulator("mov", variables, state, false);
}

void AddToFrom(InstructionVariables variables, SimulatorState &state) {
  OperationToFrom("add", variables, state);
}

void AddBasedOnMode(InstructionVariables variables, SimulatorState &state) {
  OperationBasedOnMode("add", variables, state);
}

void AddImmediate(InstructionVariables variables, SimulatorState &state) {

  Operand dest{};
  dest.Type = OperandType::RegisterValue;
  dest.RegisterValue = variables.Word ? Register::AX : Register::AL;

  Operand source = DecodeImmediate(state, variables);

  PrintInstruction("add", dest, source);
}

constexpr InstructionVariableOffsets RegisterOrMemoryToRegisterOrMemoryOffsets =
    {.Word = 8, .Direction = 9, .Reg = 3, .Mode = 6, .RM = 0};
constexpr InstructionVariableOffsets ImmediateToRegisterOrMemoryOffsets = {
    .Word = 8, .Mode = 6, .RM = 0};
constexpr InstructionVariableOffsets
    ImmediateToRegisterOrMemoryArithmeticOffsets = {
        .Word = 8, .SignExtend = 9, .Mode = 6, .RM = 0};

constexpr InstructionVariableOffsets WordOnlyOffsets = {.Word = 8};
constexpr InstructionVariableOffsets ImmediateToRegisterOffsets = {.Word = 11,
                                                                   .Reg = 8};

const OpcodeMatcher OpcodeMatchers[] = {
    OpcodeMatcher(MoveToFrom, RegisterOrMemoryToRegisterOrMemoryOffsets,
                  {ConstantPart(6, 0b100010), VariablePart(10)}),
    OpcodeMatcher(MoveImmediateToRegister, ImmediateToRegisterOffsets,
                  {ConstantPart(4, 0b1011), VariablePart(4)}),
    OpcodeMatcher(MoveImmediateToRegisterOrMemory,
                  ImmediateToRegisterOrMemoryOffsets,
                  {ConstantPart(7, 0b1100011), VariablePart(3),
                   ConstantPart(3, 0b000), VariablePart(3)}),
    OpcodeMatcher(MoveMemoryToAccumulator, WordOnlyOffsets,
                  {ConstantPart(7, 0b1010000), VariablePart(1)}),
    OpcodeMatcher(MoveAccumlatorToMemory, WordOnlyOffsets,
                  {ConstantPart(7, 0b1010001), VariablePart(1)}),

    OpcodeMatcher(AddToFrom, RegisterOrMemoryToRegisterOrMemoryOffsets,
                  {ConstantPart(6, 0b000000), VariablePart(10)}),
    OpcodeMatcher(AddBasedOnMode, ImmediateToRegisterOrMemoryArithmeticOffsets,
                  {ConstantPart(6, 0b100000), VariablePart(4),
                   ConstantPart(3, 0b000), VariablePart(3)}),
    OpcodeMatcher(AddImmediate, WordOnlyOffsets,
                  {ConstantPart(7, 0b0000010), VariablePart(1)}),

    // TODO subtraction ops
    // {.Mask = 0b1111'1100, .Opcode = 0b0010'1000, .Func = AddToFrom},
    // {.Mask = 0b1111'1100, .Opcode = 0b1000'0000, .Func = AddBasedOnMode},
    // {.Mask = 0b1111'1110, .Opcode = 0b0010'1100, .Func = AddImmediate},
}; // namespace

bool FindOperation(u16 opcode, u8 byteLength, const OpcodeMatcher **output) {
  for (const auto &matcher : OpcodeMatchers) {
    if (matcher.Matches(opcode, byteLength)) {
      *output = &matcher;
      return true;
    }
  }

  return false;
}

InstructionVariables
ExtractInstructionVariables(const InstructionVariableOffsets &offsets,
                            u16 opcode) {
  InstructionVariables result{};
  result.Word = ((opcode >> offsets.Word) & 0b1) == 1;
  result.Direction = ((opcode >> offsets.Direction) & 0b1) == 1;
  result.SignExtend = ((opcode >> offsets.SignExtend) & 0b1) == 1;
  result.RM = (opcode >> offsets.RM) & 0b111;
  result.Mode = (opcode >> offsets.Mode) & 0b11;
  result.Reg = (opcode >> offsets.Reg) & 0b111;
  return result;
}

} // namespace

void Simulator8086::Execute() {
  for (;;) {
    if (mState.PC >= mState.CodeLength) {
      break;
    }

    u8 opcodeByte = mState.AdvanceInstructionByte();
    u8 readLength = mState.PC < mState.CodeLength ? 2 : 1;
    u8 nextOpcodeByte = readLength == 2 ? mState.ReadByte(mState.PC) : 0;
    u16 opcode = (u16(opcodeByte) << 8) | nextOpcodeByte;

    const OpcodeMatcher *opcodeMatcher{};
    if (!FindOperation(opcode, readLength, &opcodeMatcher)) {
      printf("ERROR Unexpected opcode pattern %hhx\n", opcodeByte);
      return;
    }

    if (opcodeMatcher->BitLength > 8) {
      mState.AdvanceInstructionByte();
    }

    InstructionVariables variables =
        ExtractInstructionVariables(opcodeMatcher->VariableOffsets, opcode);
    opcodeMatcher->Func(variables, mState);
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
