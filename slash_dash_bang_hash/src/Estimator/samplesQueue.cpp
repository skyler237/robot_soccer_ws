#include "Estimator/samplesQueue.h"
#include "Estimator/estimator.h"
#include "Utilities/utilities.h"
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>


samplesQueue::samplesQueue() {
  size_ = MAX_QUEUE_SIZE;
  index_ = 0;
  sample_cnt_ = 0;
}

samplesQueue::samplesQueue(int queue_size) {
  size_ = queue_size;
  index_ = 0;
  sample_cnt_ = 0;
}

void samplesQueue::addSample(State sample) {
  samples_[index_] = sample;
  index_ = (index_ + 1)%size_; // update index and wrap around
  sample_cnt_ = saturate(sample_cnt_ + 1, 0, size_); // Saturate sample count at the size of the queue
}


// TODO: fix this here -- add estimator correction!
State samplesQueue::updateSamples(State update, int samples_old, double dt) {
  samples_old = saturate(samples_old, 0, sample_cnt_); // Don't try to update past the data we have stored
  int update_index = ((index_ - samples_old) + size_) % size_; // Go back the right number of samples, but with wrap around

  // Correct state here
  samples_[update_index] = Estimator::correctStateWithMeasurementsOnly(update);

  int curr_index = update_index;
  int next_index;
  for(int i = 0; i < samples_old; i++) {
    next_index = (curr_index + 1)%size_; // Get next index with wrap around
    samples_[next_index] = Estimator::predictState(samples_[curr_index], dt);
    curr_index = next_index;
  }
  return samples_[curr_index];
}
