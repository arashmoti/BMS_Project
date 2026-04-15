#ifndef __BUFFER__H_
#define __BUFFER__H_

#include <stdint.h>

void     bufferAppend_int8(uint8_t* buffer, int8_t number, int32_t *index);
void     bufferAppend_uint8(uint8_t* buffer, uint8_t number, int32_t *index);
void     bufferAppend_int16(uint8_t* buffer, int16_t number, int32_t *index);
void     bufferAppend_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void     bufferAppend_int32(uint8_t* buffer, int32_t number, int32_t *index);
void     bufferAppend_uint32(uint8_t* buffer, uint32_t number, int32_t *index);

void     bufferAppend_int16_LSBFirst(uint8_t* buffer, int16_t number, int32_t *index);
void     bufferAppend_uint16_LSBFirst(uint8_t* buffer, uint16_t number, int32_t *index);
void     bufferAppend_int32_LSBFirst(uint8_t* buffer, int32_t number, int32_t *index);
void     bufferAppend_uint32_LSBFirst(uint8_t* buffer, uint32_t number, int32_t *index);

void     bufferAppend_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void     bufferAppend_float32(uint8_t* buffer, float number, float scale, int32_t *index);
int8_t   bufferGet_int8(const uint8_t *buffer, int32_t *index);
uint8_t  bufferGet_uint8(const uint8_t *buffer, int32_t *index);
int16_t  bufferGet_int16(const uint8_t *buffer, int32_t *index);
uint16_t bufferGet_uint16(const uint8_t *buffer, int32_t *index);
int32_t  bufferGet_int32(const uint8_t *buffer, int32_t *index);
uint32_t bufferGet_uint32(const uint8_t *buffer, int32_t *index);
float    bufferGet_float16(const uint8_t *buffer, float scale, int32_t *index);
float    bufferGet_float32(const uint8_t *buffer, float scale, int32_t *index);

#endif /* __BUFFER__H_ */
