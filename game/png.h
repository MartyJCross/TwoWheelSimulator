// Minimal PNG writer (uncompressed deflate) for screenshots. No dependencies.
#pragma once
#include <cstdint>

// rgb: width*height*3 bytes, top row first. Returns false if the file could not be written.
bool writePng(const char* path, int width, int height, const uint8_t* rgb);
