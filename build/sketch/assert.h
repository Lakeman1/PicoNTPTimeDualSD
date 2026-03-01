#line 1 "/home/nor/PicoNTPTimeDualSD/PicoNTPTimeDualSD/assert.h"
// assert.h
#pragma once

void panic(const char *file, int line);

#ifdef DEBUG
  #define ASSERT(x) do { if(!(x)) panic(__FILE__, __LINE__); } while(0)
#else
  #define ASSERT(x) ((void)0)
#endif
