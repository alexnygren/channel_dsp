#ifndef __AVERAGE_H__
#define __AVERAGE_H__ 1

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#define TARGET_SIZE float


struct average_structure {
  TARGET_SIZE *arr_numbers;
  TARGET_SIZE last_average;
  TARGET_SIZE avg_size;
  uint32_t avg_pos;
  TARGET_SIZE avg_sum;
  TARGET_SIZE value;
  TARGET_SIZE locked;
  bool is_locked;
  TARGET_SIZE range;
};

typedef struct average_structure average;

TARGET_SIZE moving_average(average *avg, TARGET_SIZE next_val, bool log);

average *initialize_average(unsigned int average_size);

average **initialize_averages(unsigned int number_to_alloc, unsigned int average_size);

#endif



