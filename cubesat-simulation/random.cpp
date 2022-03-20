// normal_distribution
#include <iostream>
#include <string>
#include <random>

int main(void)
{
  const int nrolls=10000;  // number of experiments
  const int nstars=100;    // maximum number of stars to distribute

  std::default_random_engine generator;
  std::normal_distribution<float> distribution(0.0,5e-2);

  for (int i=0; i<nrolls; ++i) {
    float number = distribution(generator);
    std::cout << number << "\n";
  }
  return 0;
}