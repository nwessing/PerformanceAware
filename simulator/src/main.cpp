#include "simulator_8086.h"
#include <stdio.h>
#include <vector>

bool ReadFileToVector(const char *filename, std::vector<uint8_t> &fileData) {
  FILE *file = fopen(filename, "rb");
  if (file == nullptr) {
    printf("Error opening file %s\n", filename);
    return false;
  }

  fseek(file, 0, SEEK_END);
  size_t fileLength = ftell(file);

  fileData.resize(fileLength);

  fseek(file, 0, SEEK_SET);
  size_t bytesRead = fread(fileData.data(), 1, fileLength, file);
  if (bytesRead != fileLength) {
    printf("Error reading full file %s Expected %u bytes, read %u bytes\n",
           filename, (uint32_t)fileLength, (uint32_t)bytesRead);
    return false;
  }

  return true;
}

int main(int argc, const char *const *argv) {
  if (argc < 2) {
    printf("Expected file name\n");
    return 1;
  }

  const char *filename = argv[1];
  std::vector<uint8_t> fileData{};
  if (!ReadFileToVector(filename, fileData)) {
    return 1;
  }

  Simulator8086 simulator{fileData};
  simulator.Execute();

  return 0;
}
