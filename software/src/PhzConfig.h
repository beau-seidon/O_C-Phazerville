#pragma once

#include <unordered_map>

namespace PhzConfig {
  enum ConfigKeys : uint16_t {
    POWER_CYCLE_COUNT,
    AUDIO_CONFIG,
  };

  using KEY = uint16_t;
  using VALUE = uint64_t;
  using ConfigMap = std::unordered_map<KEY, VALUE>;

  const char * const CONFIG_FILENAME = "PEWPEW.CFG";

  // Forward Decl
  void listFiles();
  bool load_config(const char* filename = CONFIG_FILENAME);
  void save_config(const char* filename = CONFIG_FILENAME);
  void clear_config();

  void setValue(KEY key, VALUE value);
  bool getValue(KEY key, VALUE &value);

  void printDirectory(FS &fs);
  void printDirectory(File dir, int numSpaces);
  void printSpaces(int num);
  void setup();
  void eraseFiles();

}
