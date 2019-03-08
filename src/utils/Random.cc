/*
 * File: Random.cc
 * Project: GLib library
 * Author: gcj
 * Date: 2019/3/8
 * Description: 常用随机数。
 * License: see the LICENSE.txt file
 * reference: 修改版本的 DLib 库的 Random.cc。下面是 DLib 库的 license:
 */
/*DLib. Copyright (c) 2015 Dorian Galvez-Lopez. http://doriangalvez.com
  All rights reserved.
*/

#include "Random.h"

#include <ctime> // time(NULL)

namespace glib {

using namespace std;
bool Random::already_seeded_ = false;

void Random::SeedRand(){
	srand(time(NULL));
}

void Random::SeedRandOnce()
{
  if(!already_seeded_)
  {
    Random::SeedRand();
    already_seeded_ = true;
  }
}

void Random::SeedRand(int seed)
{
	srand(seed);
}

void Random::SeedRandOnce(int seed)
{
  if(!already_seeded_)
  {
    Random::SeedRand(seed);
    already_seeded_ = true;
  }
}

int Random::RandomInt(int min, int max){
	int d = max - min + 1;
	return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}

void Random::RandomVector(vector<size_t> &vec,
                          size_t min, size_t max) {
    for (unsigned int i = 0; i < vec.size(); ++i) {
        int randi = Random::RandomInt(min, max);
        vec[i] = randi;
    }
}

// ---------------------------------------------------------------------------
// 内嵌不重复随机数类定义

Random::UnrepeatedRandomizer::UnrepeatedRandomizer(int min, int max)
{
  if(min <= max)
  {
    min_ = min;
    max_ = max;
  }
  else
  {
    min_ = max;
    max_ = min;
  }

  createValues();
}

// ---------------------------------------------------------------------------

Random::UnrepeatedRandomizer::UnrepeatedRandomizer
  (const Random::UnrepeatedRandomizer& rnd)
{
  *this = rnd;
}

// ---------------------------------------------------------------------------

int Random::UnrepeatedRandomizer::get()
{
  if(empty()) createValues();

  Random::SeedRandOnce();

  int k = Random::RandomInt(0, values_.size()-1);
  int ret = values_[k];
  values_[k] = values_.back();
  values_.pop_back();

  return ret;
}

// ---------------------------------------------------------------------------

void Random::UnrepeatedRandomizer::createValues()
{
  int n = max_ - min_ + 1;

  values_.resize(n);
  for(int i = 0; i < n; ++i) values_[i] = min_ + i;
}

// ---------------------------------------------------------------------------

void Random::UnrepeatedRandomizer::reset()
{
  if((int)values_.size() != max_ - min_ + 1) createValues();
}

// ---------------------------------------------------------------------------

Random::UnrepeatedRandomizer&
Random::UnrepeatedRandomizer::operator=
  (const Random::UnrepeatedRandomizer& rnd)
{
  if(this != &rnd)
  {
    this->min_ = rnd.min_;
    this->max_ = rnd.max_;
    this->values_ = rnd.values_;
  }
  return *this;
}

// ---------------------------------------------------------------------------
} // namespace glib
