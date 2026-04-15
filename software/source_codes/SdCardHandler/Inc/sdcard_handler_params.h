#ifndef __SDCARD_HANDLER_PARAMS__H_
#define __SDCARD_HANDLER_PARAMS__H_

#include <cstdint>

namespace SdCardArgs {
  #pragma pack(push)
  #pragma pack(1)

  #pragma pack(pop)
  enum e_statusType {
    OK = 0,
    SD_INIT_FAILED,
    SD_DEINIT_FAILED,
    MOUNT_FAILED,
    UNMOUNT_FAILED,
    FILE_DOES_NOT_EXIST,
    FILE_CREATED_FAILED,
    WRITE_FAILED,
    PLUG_DETECTED_FAILED,
    WRITE_DISABLED
  };
} // namespace SdCardArgs

#endif // __SDCARD_HANDLER_PARAMS__H_
