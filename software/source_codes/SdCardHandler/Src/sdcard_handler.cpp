#include "sdcard_handler.h"
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <chrono>
#include "Settings.h"
#include "mbed.h"

extern "C" {
    int fileno(FILE *stream);
    int fsync(int fd);
}

// Forward declarations for rotation helpers used below
static size_t get_file_size_bytes(const char *path);
static void rotate_if_needed(SdCardHandler *self);

static void flush_and_sync(FILE *f)
{
  if (!f) return;
  fflush(f);
  int fd = fileno(f);
  if (fd >= 0) fsync(fd);
}

SdCardHandler::SdCardHandler(ISerial& _serial)
    : m_sdCardDetPin(SD_DET_PIN),
      m_sd(SD_MOSI_PIN, SD_MISO_PIN, SD_SCLK_PIN, SD_CS_PIN), m_fs("sd"),
      m_cPtractiveFileName(nullptr),
      statusType(SdCardStatus::OK),
      m_allowWrites(true),
      m_serial(_serial) {}

// #############################################################################################//

SdCardHandler::~SdCardHandler() {}

// #############################################################################################//

bool SdCardHandler::sdCardPluggedDetected(void) {
  static bool pluggedDetected = false;
  pluggedDetected = m_sdCardDetPin.read();
  return !pluggedDetected;
}

// #############################################################################################//

void SdCardHandler::init(void) {
  if (m_sd.init() == HAL_OK) {
    if (m_fs.mount(&m_sd) == HAL_OK) {
    } else {
      statusType = SdCardStatus::MOUNT_FAILED;
    }
  } else {
    statusType = SdCardStatus::SD_INIT_FAILED;
  }
}

// #############################################################################################//

void SdCardHandler::deinit() {
  if (m_sd.deinit() == HAL_OK) {
    if (m_fs.unmount() == HAL_OK) {
    } else {
      statusType = SdCardStatus::UNMOUNT_FAILED;
    }
  } else {
    statusType = SdCardStatus::SD_DEINIT_FAILED;
  }
}

// #############################################################################################//

void SdCardHandler::safeEject()
{
  // Best-effort unmount and block device deinit. This helps ensure that
  // directory entries and allocation chains are committed before power-off.
  // It is safe to call even if already unmounted; errors are recorded.
  m_sdMutex.lock();
  m_allowWrites = false;
  if (!sdCardPluggedDetected()) {
    statusType = SdCardStatus::PLUG_DETECTED_FAILED;
    m_sdMutex.unlock();
    return;
  }
  // Try unmount first, then deinit the block device
  bool ok = true;
  if (m_fs.unmount() != HAL_OK) ok = false;
  if (m_sd.deinit() != HAL_OK) ok = false;
  statusType = ok ? SdCardStatus::WRITE_DISABLED : SdCardStatus::UNMOUNT_FAILED;
  // Optional user feedback over serial
  m_serial.serialPrint("SD safe eject: %s\r\n", ok ? "OK" : "partial");
  m_sdMutex.unlock();
}

bool SdCardHandler::beginAccess()
{
  m_sdMutex.lock();
  if (!m_allowWrites)
  {
    statusType = SdCardStatus::WRITE_DISABLED;
    m_sdMutex.unlock();
    return false;
  }
  return true;
}

void SdCardHandler::endAccess()
{
  m_sdMutex.unlock();
}

// #############################################################################################//

void SdCardHandler::isFilenameExist(void) {
  FILE *file = fopen(m_cPtractiveFileName, "r");
  if (file != NULL) {
    fclose(file);
  }
  else {
  statusType = SdCardStatus::FILE_DOES_NOT_EXIST;
  }
  
}

// #############################################################################################//

void SdCardHandler::readActiveFile() 
{
      FILE* file = fopen(m_cPtractiveFileName, "r");
    if (file == NULL) 
    {
        statusType = SdCardStatus::FILE_DOES_NOT_EXIST;
        return;
    }

    int c = getc(file);
    // read till end of file
    while (c != EOF) 
    {   
        // to get cursor back into original position of stream
        ungetc(c, file);
        m_serial.serialPrint("%c", getc(file));
        c = getc(file);
    }
    m_serial.serialPrint("\n");
    fclose(file);
}

// #############################################################################################//

void SdCardHandler::writeErrorLogToActiveFile(st_errorSdCardData* errorSDcardData)
{
    rotate_if_needed(this);
    if (get_file_size_bytes(m_cPtractiveFileName) == 0) { addErrorHeaderName(); }
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", errorSDcardData->errorType, errorSDcardData->operationState, 
                                          errorSDcardData->datetimeInfo.u8SysSecond, errorSDcardData->datetimeInfo.u8SysMinute,
                                          errorSDcardData->datetimeInfo.u8SysHour, errorSDcardData->datetimeInfo.u8SysDay,
                                          errorSDcardData->datetimeInfo.u8SysMonth, errorSDcardData->datetimeInfo.u8SysYear,
                                          errorSDcardData->u16SysVoltage, errorSDcardData->i32SysCurrent,
                                          errorSDcardData->i32PcbTemperature, errorSDcardData->i32CellTemperatureAverage,
                                          errorSDcardData->u16CellVoltageAverage, errorSDcardData->u16CellVoltageMin,
                                          errorSDcardData->u16CellVoltageMax, errorSDcardData->i32TemperatureMin,
                                          errorSDcardData->i32TemperatureMax
    );
    flush_and_sync(file);
    fclose(file);
}

// #############################################################################################//

void SdCardHandler::writeCellVoltageLogToActiveFile(uint32_t time, uint8_t module_num, uint16_t* cellInfo) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", time, module_num, 
                                          cellInfo[0], cellInfo[1], cellInfo[2], cellInfo[3],  
                                          cellInfo[4], cellInfo[5], cellInfo[6], cellInfo[7],
                                          cellInfo[8], cellInfo[9], cellInfo[10], cellInfo[11],
                                          cellInfo[12], cellInfo[13], cellInfo[14], cellInfo[15]
    );
    flush_and_sync(file);
    fclose(file); 
}

// #############################################################################################//

void SdCardHandler::writePackTemperatureLogToActiveFile(uint32_t time, uint8_t module_num, int32_t* tempInfo) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", time, module_num,  
                                          tempInfo[0], tempInfo[1], tempInfo[2], tempInfo[3],  
                                          tempInfo[4], tempInfo[5], tempInfo[6], tempInfo[7]
    );
    flush_and_sync(file);
    fclose(file);   
}

// #############################################################################################//

void SdCardHandler::writeSystemInformationLogToActiveFile(uint32_t time, st_powerElecPackInfoConfig* powerElecPackInfoConfig) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", time,
                                          powerElecPackInfoConfig->u32ModuleVoltage, powerElecPackInfoConfig->u32ModuleLoadVoltage, powerElecPackInfoConfig->i32ModuleCurrent, powerElecPackInfoConfig->u16ModuleCellVoltageMax,  
                                          powerElecPackInfoConfig->u16ModuleCellVoltageMin, powerElecPackInfoConfig->u16ModuleCellVoltageAverage, powerElecPackInfoConfig->u16ModuleCellVoltageMisMatch, powerElecPackInfoConfig->u16SupplyVoltage,
                                          powerElecPackInfoConfig->u16ModuleSoc, powerElecPackInfoConfig->i32ModuleTemperatureMax, powerElecPackInfoConfig->i32ModuleTemperatureMin, powerElecPackInfoConfig->i32ModuleTemperatureAverage,
                                          powerElecPackInfoConfig->bmsState, powerElecPackInfoConfig->u8operationState, powerElecPackInfoConfig->packBatteryWarErrState, powerElecPackInfoConfig->packOperationCellState
    );
    flush_and_sync(file);
    fclose(file);   
}

// #############################################################################################//

void SdCardHandler::setActiveFile(const char *path) {
  /* if memory has previously been allocated to activeFileName, free it to
   * prevent memory leakage */
  if (m_cPtractiveFileName != NULL)
    free(m_cPtractiveFileName);

  // concatenate the path to rootpath and set as activeFileName
  const char *rootPath = "/sd/";
  m_cPtractiveFileName =
      static_cast<char *>(malloc(1 + strlen(rootPath) + strlen(path)));
  strcpy(m_cPtractiveFileName, rootPath);
  strcat(m_cPtractiveFileName, path);
}

// #############################################################################################//

const char *SdCardHandler::getActiveFileName() { 
  return m_cPtractiveFileName; 
}

// ---- helpers for rotation ----
static size_t get_file_size_bytes(const char *path)
{
  FILE *f = fopen(path, "rb");
  if (!f) return 0;
  fseek(f, 0, SEEK_END);
  long pos = ftell(f);
  fclose(f);
  return (pos < 0) ? 0 : (size_t)pos;
}

static bool file_exists(const char *path)
{
  FILE *f = fopen(path, "rb");
  if (!f) return false; fclose(f); return true;
}

static void build_rolled_name(const char *basePath, char *out, size_t outsz)
{
  // basePath like "/sd/cellVoltageLog.csv" -> "/sd/cellVoltageLog_0001.csv"
  const char *slash = strrchr(basePath, '/');
  const char *name = slash ? slash + 1 : basePath;
  char dir[64] = {0};
  if (slash) { size_t n = (size_t)(slash - basePath + 1); n = (n < sizeof(dir)-1 ? n : sizeof(dir)-1); memcpy(dir, basePath, n); dir[n] = '\0'; }
  else { strcpy(dir, ""); }
  const char *dot = strrchr(name, '.');
  char prefix[80]; char ext[16];
  if (dot) { size_t pn = (size_t)(dot - name); pn = (pn < sizeof(prefix)-1 ? pn : sizeof(prefix)-1); memcpy(prefix, name, pn); prefix[pn] = '\0'; strncpy(ext, dot, sizeof(ext)-1); ext[sizeof(ext)-1] = '\0'; }
  else { strncpy(prefix, name, sizeof(prefix)-1); prefix[sizeof(prefix)-1] = '\0'; strcpy(ext, ""); }
  for (unsigned i = 1; i < 10000; ++i)
  {
    snprintf(out, outsz, "%s%s_%04u%s", dir, prefix, i, ext);
    if (!file_exists(out)) return; // found available name
  }
  // fallback if too many: overwrite last index
  snprintf(out, outsz, "%s%s_%04u%s", dir, prefix, 9999U, ext);
}

static void build_rolled_name_index(const char *basePath, unsigned idx, char *out, size_t outsz)
{
  const char *slash = strrchr(basePath, '/');
  const char *name = slash ? slash + 1 : basePath;
  char dir[64] = {0};
  if (slash) { size_t n = (size_t)(slash - basePath + 1); n = (n < sizeof(dir)-1 ? n : sizeof(dir)-1); memcpy(dir, basePath, n); dir[n] = '\0'; }
  else { strcpy(dir, ""); }
  const char *dot = strrchr(name, '.');
  char prefix[80]; char ext[16];
  if (dot) { size_t pn = (size_t)(dot - name); pn = (pn < sizeof(prefix)-1 ? pn : sizeof(prefix)-1); memcpy(prefix, name, pn); prefix[pn] = '\0'; strncpy(ext, dot, sizeof(ext)-1); ext[sizeof(ext)-1] = '\0'; }
  else { strncpy(prefix, name, sizeof(prefix)-1); prefix[sizeof(prefix)-1] = '\0'; strcpy(ext, ""); }
  snprintf(out, outsz, "%s%s_%04u%s", dir, prefix, idx, ext);
}

static unsigned count_segments_and_oldest(const char *basePath, unsigned *out_oldest)
{
  unsigned count = 0; unsigned oldest = 0;
  char path[128];
  for (unsigned i = 1; i < 10000; ++i) {
    build_rolled_name_index(basePath, i, path, sizeof(path));
    if (file_exists(path)) { count++; if (oldest==0) oldest=i; }
  }
  if (out_oldest) *out_oldest = oldest;
  return count;
}

// Rotate current active file if it exceeded size; keep writing to base name.
static void rotate_if_needed(SdCardHandler *self)
{
  const char *base = self->getActiveFileName();
  if (!base) return;
  const size_t sz = get_file_size_bytes(base);
  if (sz < (size_t)SD_LOG_MAX_BYTES) return;
  // Enforce a maximum number of segments: delete oldest if necessary
  unsigned oldest = 0; unsigned cnt = count_segments_and_oldest(base, &oldest);
  if (cnt >= (unsigned)SD_LOG_MAX_SEGMENTS && oldest > 0) {
    char oldestPath[128];
    build_rolled_name_index(base, oldest, oldestPath, sizeof(oldestPath));
    remove(oldestPath);
  }
  char rolled[128];
  build_rolled_name(base, rolled, sizeof(rolled));
  // Rename current base to rolled name.
  int rn = rename(base, rolled);
  if (rn != 0) {
    // If rename failed, keep appending to current file
    return;
  }
  // Create a fresh base and write header
  FILE *nf = fopen(base, "w");
  if (nf) { fclose(nf); }
}

void SdCardHandler::createFilenameToSdcard(){
   
    FILE *file = fopen(m_cPtractiveFileName, "w");
  
    if (file != NULL) {
      flush_and_sync(file);
      fclose(file);
    }
    else {
      statusType = SdCardStatus::FILE_CREATED_FAILED;
    }

}

// #############################################################################################//

void SdCardHandler::resetSdCardStatus(void){
  statusType = SdCardStatus::OK;
}

// #############################################################################################//

uint8_t SdCardHandler::getSdCardStatus(void) {
  return static_cast<uint8_t>(statusType); 
}

void SdCardHandler::addErrorHeaderName(void) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", 
                                          "errorType", "operationState", "sysSecond", "sysMinute","sysHour", "sysDay", 
                                          "sysMonth", "sysYear", "sysVoltage", "sysCurrent", "pcbTemperature", 
                                          "cellTemperatureAverage", "cellVoltageAverage", "cellVoltageMin",
                                          "cellVoltageMax", "temperatureMin", "temperatureMax"
    );
    flush_and_sync(file);
    fclose(file);
}

// #############################################################################################//

void SdCardHandler::addCellVoltageHeaderName(void) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", 
                                          "timeLogging", "module",
                                          "cellVoltage0", "cellVoltage1", "cellVoltage2","cellVoltage3", 
                                          "cellVoltage4", "cellVoltage5", "cellVoltage6","cellVoltage7", 
                                          "cellVoltage8", "cellVoltage9", "cellVoltage10","cellVoltage11",
                                          "cellVoltage12", "cellVoltage13", "cellVoltage14","cellVoltage15"
    );
    flush_and_sync(file);
    fclose(file);
}

// #############################################################################################//

void SdCardHandler::addPackTemperatureHeaderName(void) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", 
                                          "timeLogging", "module",
                                          "temperature1", "temperature2", "temperature3","temperature4", 
                                          "temperature5", "temperature6", "temperature7","temperature8"
    );
    flush_and_sync(file);
    fclose(file);
}

// #############################################################################################//

// #############################################################################################//

void SdCardHandler::addSystemInformationHeaderName(void) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL) 
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", 
                                          "timeLogging", 
                                          "ModuleVoltage", "LoadVoltage", "ModuleCurrent","MaxCellVoltage", 
                                          "MinCellVoltage", "AvgCellVoltage", "CellVoltageMismatch","SupplyVoltage",
                                          "SOC","MaxTemperature","MinTemperature","AvgTemperature",
                                          "BmsState","SystemMod","ErrorType","CellOperationState"
    );
    flush_and_sync(file);
    fclose(file);
}

// #############################################################################################//

void SdCardHandler::addEventHeaderName(void) {
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL)
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    fprintf(file, "%s,%s\n",
            "timeLogging",
            "event");
    flush_and_sync(file);
    fclose(file);
}

void SdCardHandler::writeEventLine(uint32_t time, const char *text) {
    rotate_if_needed(this);
    if (get_file_size_bytes(m_cPtractiveFileName) == 0) { addEventHeaderName(); }
    FILE* file = fopen(m_cPtractiveFileName,"a");
    if (file == NULL)
    {
        statusType = SdCardStatus::WRITE_FAILED;
        return;
    }
    // Escape commas minimally by replacing with semicolons
    // Copy to a small buffer (truncate if too long)
    char buf[96];
    size_t n = 0;
    for (const char *p = text; *p && n < sizeof(buf)-1; ++p) {
        char c = *p;
        if (c == ',') c = ';';
        buf[n++] = c;
    }
    buf[n] = '\0';
    fprintf(file, "%lu,%s\n", (unsigned long)time, buf);
    flush_and_sync(file);
    fclose(file);
}

void SdCardHandler::logEvent(const char *fmt, ...)
{
  if (!beginAccess())
  {
    return;
  }
  if (!sdCardPluggedDetected())
  {
    statusType = SdCardStatus::PLUG_DETECTED_FAILED;
    endAccess();
    return;
  }

  setActiveFile("EVENTS_LOG.csv");
  init();
  if (getSdCardStatus() != HAL_OK)
  {
    endAccess();
    return;
  }

  isFilenameExist();
  if (getSdCardStatus() != HAL_OK)
  {
    createFilenameToSdcard();
    addEventHeaderName();
    resetSdCardStatus();
  }
  else {
    // Existing file: rotate if needed before we append; ensure header if new file created
    rotate_if_needed(this);
    if (get_file_size_bytes(m_cPtractiveFileName) == 0) { addEventHeaderName(); }
  }

  // Format message
  char msg[128];
  va_list vl;
  va_start(vl, fmt);
  vsnprintf(msg, sizeof(msg), fmt, vl);
  va_end(vl);

  const auto now = Kernel::Clock::now().time_since_epoch();
  uint32_t now_ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  writeEventLine(now_ms, msg);

  if (getSdCardStatus() != HAL_OK)
  {
    endAccess();
    return;
  }
  deinit();
  if (getSdCardStatus() != HAL_OK)
  {
    endAccess();
    return;
  }
  endAccess();
}

// #############################################################################################//

uint8_t SdCardHandler::sdCardErrorCallback(st_errorSdCardData* errorSDcardData) {

  if (!beginAccess())
  {
    return 1;
  }
  if(sdCardPluggedDetected())
  {
    setActiveFile(ERRORLOG_FILENAME);
    init();
    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    isFilenameExist();

    if(getSdCardStatus() != HAL_OK)
    {
      createFilenameToSdcard();
      addErrorHeaderName();
      resetSdCardStatus();
    }

    writeErrorLogToActiveFile(errorSDcardData);

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    deinit();

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

  }

  else {
    statusType = SdCardStatus::PLUG_DETECTED_FAILED;
    endAccess();
    return 1;
  }
  endAccess();
  return 255;
}

// #############################################################################################//

uint8_t SdCardHandler::cellVoltageLogging(uint32_t time, uint8_t module_num, uint16_t* cellInfo) {
  if (!beginAccess())
  {
    return 1;
  }
  if(sdCardPluggedDetected())
  {
    setActiveFile(CELL_VOLTAGE_LOG_FILENAME);
    init();
    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    isFilenameExist();

    if(getSdCardStatus() != HAL_OK)
    {
      createFilenameToSdcard();
      addCellVoltageHeaderName();
      resetSdCardStatus();
    }
    else {
      rotate_if_needed(this);
      if (get_file_size_bytes(m_cPtractiveFileName) == 0) { addCellVoltageHeaderName(); }
    }

    writeCellVoltageLogToActiveFile(time, module_num, cellInfo);

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    deinit();

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

  }

  else {
    statusType = SdCardStatus::PLUG_DETECTED_FAILED;
    endAccess();
    return 1;
  }
  endAccess();
  return 255;
}

uint8_t SdCardHandler::packTemperatureLogging(uint32_t time, uint8_t module_num, int32_t* tempInfo) {

  if (!beginAccess())
  {
    return 1;
  }
  if(sdCardPluggedDetected()) 
  {
    setActiveFile(PACK_TEMPERATURE_LOG_FILENAME);
    init();
    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    isFilenameExist();

    if(getSdCardStatus() != HAL_OK)
    {
      createFilenameToSdcard();
      addPackTemperatureHeaderName();
      resetSdCardStatus();
    }
    else {
      rotate_if_needed(this);
      if (get_file_size_bytes(m_cPtractiveFileName) == 0) { addPackTemperatureHeaderName(); }
    }

    writePackTemperatureLogToActiveFile(time, module_num, tempInfo);

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    deinit();

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }
  } else {
    statusType = SdCardStatus::PLUG_DETECTED_FAILED;
    endAccess();
    return 1;
  }
  endAccess();
  return 255;
}

// #############################################################################################//

uint8_t SdCardHandler::systemInformationLogging(uint32_t time, st_powerElecPackInfoConfig* powerElecPackInfoConfig) {
  if (!beginAccess())
  {
    return 1;
  }
  if(sdCardPluggedDetected()) 
  {
    setActiveFile(SYSTEM_INFORMATION_LOG_FILENAME);
    init();
    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    isFilenameExist();

    if(getSdCardStatus() != HAL_OK)
    {
      createFilenameToSdcard();
      addSystemInformationHeaderName();
      resetSdCardStatus();
    }
    else {
      rotate_if_needed(this);
      if (get_file_size_bytes(m_cPtractiveFileName) == 0) { addSystemInformationHeaderName(); }
    }

    writeSystemInformationLogToActiveFile(time, powerElecPackInfoConfig);

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }

    deinit();

    if(getSdCardStatus() != HAL_OK)
    {
      endAccess();
      return 1;
    }
  } else {
    statusType = SdCardStatus::PLUG_DETECTED_FAILED;
    endAccess();
    return 1;
  }
  endAccess();
  return 255;
}

/*
  sdCard.setActiveFile(ERRORLOG_FILENAME);
  sdCard.init();
  serialPort.logMessage("SdCardStatus : %d", sdCard.getSdCardStatus());
  if(sdCard.getSdCardStatus() != HAL_OK)
  {
    sdCard.resetSdCardStatus();
  }


  sdCard.isFilenameExist();
  serialPort.logMessage("SdCardStatus : %d", sdCard.getSdCardStatus());
  if(sdCard.getSdCardStatus() != HAL_OK)
  {
    sdCard.createFilenameToSdcard();
    sdCard.resetSdCardStatus();
  }

  uint8_t _u8errorType = 2;
  uint8_t _u8year = 24;
  uint8_t _u8mounth = 24;
  uint8_t _u8day = 24;
  uint8_t _u8hour = 24;
  uint8_t _u8minute = 24;

  sdCard.writeErrorLogToActiveFile(_u8errorType, _u8year, _u8mounth, _u8day, _u8hour, _u8minute);
  serialPort.logMessage("SdCardStatus : %d", sdCard.getSdCardStatus());
  if(sdCard.getSdCardStatus() != HAL_OK)
  {
    sdCard.resetSdCardStatus();
  }

  sdCard.readActiveFile();
*/
