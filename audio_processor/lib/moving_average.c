#include "moving_average.h"

TARGET_SIZE moving_average(average *avg, TARGET_SIZE next_val, bool log) {

  TARGET_SIZE nextNum = (TARGET_SIZE) next_val;

  //Subtract the oldest number from the prev sum, add the new number
  avg->avg_sum = avg->avg_sum - avg->arr_numbers[avg->avg_pos] + nextNum;
  //Assign the nextNum to the position in the array
  avg->arr_numbers[avg->avg_pos] = nextNum;
  avg->last_average = avg->value;
  avg->value = (TARGET_SIZE) (avg->avg_sum / avg->avg_size);
  avg->avg_pos++;
  if (avg->avg_pos >= avg->avg_size) {
    avg->avg_pos = 0;
  }
  return avg->value;
}

average *initialize_average(unsigned int average_size) {
  average *avg = (average *) calloc(1, (sizeof (average)));
  avg->arr_numbers = (TARGET_SIZE *) calloc(average_size,sizeof(TARGET_SIZE));
  avg->avg_size = average_size;
  avg->avg_sum = 0;
  avg->avg_pos = 0;
  avg->value = 0;
  avg->last_average = 0;
  avg->locked = 0;

  return avg;
}

average **initialize_averages(unsigned int number_to_alloc, unsigned int average_size) {
  // initialize the average values
  average **averages = (average **) calloc(number_to_alloc+1,sizeof(average*));
  for (int offset = 0; offset < number_to_alloc+1; offset++) {
    averages[offset] = initialize_average(average_size);
  }
  return averages;
}

