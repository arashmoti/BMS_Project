#include "lib_buffer.h"

// ######################################################################################################################

void bufferAppend_int8(uint8_t* buffer, int8_t number, int32_t *index) {
	buffer[(*index)++] = number;
}

// ######################################################################################################################

void bufferAppend_uint8(uint8_t* buffer, uint8_t number, int32_t *index) {
	buffer[(*index)++] = number;
}

// ######################################################################################################################

void bufferAppend_int16(uint8_t* buffer, int16_t number, int32_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// ######################################################################################################################

void bufferAppend_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// ######################################################################################################################

void bufferAppend_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// ######################################################################################################################

void bufferAppend_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// ######################################################################################################################

void bufferAppend_int16_LSBFirst(uint8_t* buffer, int16_t number, int32_t *index) {
	buffer[(*index)++] = number;
	buffer[(*index)++] = number >> 8;
}

// ######################################################################################################################

void bufferAppend_uint16_LSBFirst(uint8_t* buffer, uint16_t number, int32_t *index) {
	buffer[(*index)++] = number;
	buffer[(*index)++] = number >> 8;
}

// ######################################################################################################################

void bufferAppend_int32_LSBFirst(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 24;
}

// ######################################################################################################################

void bufferAppend_uint32_LSBFirst(uint8_t* buffer, uint32_t number, int32_t *index) {
	buffer[(*index)++] = number;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 24;
}

// ######################################################################################################################

void bufferAppend_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    bufferAppend_int16(buffer, (int16_t)(number * scale), index);
}

// ######################################################################################################################

void bufferAppend_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    bufferAppend_int32(buffer, (int32_t)(number * scale), index);
}

// ######################################################################################################################

int8_t bufferGet_int8(const uint8_t *buffer, int32_t *index) {
	int8_t res =	((uint8_t) buffer[*index]);
	*index += 1;
	return res;
}

// ######################################################################################################################

uint8_t bufferGet_uint8(const uint8_t *buffer, int32_t *index) {
	uint8_t res = 	((uint8_t) buffer[*index]);
	*index += 1;
	return res;
}

// ######################################################################################################################

int16_t bufferGet_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

// ######################################################################################################################

uint16_t bufferGet_uint16(const uint8_t *buffer, int32_t *index) {
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

// ######################################################################################################################

int32_t bufferGet_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

// ######################################################################################################################

uint32_t bufferGet_uint32(const uint8_t *buffer, int32_t *index) {
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

// ######################################################################################################################

float bufferGet_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)bufferGet_int16(buffer, index) / scale;
}

// ######################################################################################################################

float bufferGet_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)bufferGet_int32(buffer, index) / scale;
}

// ######################################################################################################################
