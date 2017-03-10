#pragma once

#include "slash_dash_bang_hash/State.h"

using namespace slash_dash_bang_hash;

#define MAX_QUEUE_SIZE 30

class samplesQueue
{
  // Constructors
public:
  samplesQueue();
  samplesQueue(int queue_size);

  void addSample(State sample);
  State updateSamples(State update, int samples_old, double dt);

private:
  State samples_[MAX_QUEUE_SIZE]; // Array of States to keep track of samples
  int size_;
  int index_;
  int sample_cnt_;
};
