#include <vector>
#include <cmath>

template<int T>
std::vector<float> calc_Quintic(int64_t a0, int64_t a1, int64_t a2, int64_t a3, int64_t a4, int64_t a5, int64_t t)
{ 
  std::vector<float> output;
  float digits = static_cast<float>(pow(2,T));
  int64_t third_derivation = 24*a4+std::round(static_cast<float>(60*a5*t)/digits);
  third_derivation = 6*a3+std::round(static_cast<float>(t*third_derivation)/digits);
  output.push_back(third_derivation/digits);
  int64_t second_derivation=0;
  if(third_derivation>=0)
  {
    second_derivation = 12*a4+std::ceil(static_cast<float>(20*a5*t)/digits);
    second_derivation = 6*a3+std::ceil(static_cast<float>(t*second_derivation)/digits);
    second_derivation = 2*a2+std::ceil(static_cast<float>(t*second_derivation)/digits);
  }
  else
  {
    second_derivation = 12*a4+std::floor(static_cast<float>(20*a5*t)/digits);
    second_derivation = 6*a3+std::floor(static_cast<float>(t*second_derivation)/digits);
    second_derivation = 2*a2+std::floor(static_cast<float>(t*second_derivation)/digits);
  }
  output.push_back(second_derivation/digits);
  
  int64_t first_derivation=0;
  if(second_derivation>=0)
  {
    first_derivation = 4*a4+std::ceil(static_cast<float>(5*a5*t)/digits);
    first_derivation = 3*a3+std::ceil(static_cast<float>(t*first_derivation)/digits);
    first_derivation = 2*a2+std::ceil(static_cast<float>(t*first_derivation)/digits);
    first_derivation = a1+std::ceil(static_cast<float>(t*first_derivation)/digits);
  }
  else
  {
    first_derivation = 4*a4+std::floor(static_cast<float>(5*a5*t)/digits);
    first_derivation = 3*a3+std::floor(static_cast<float>(t*first_derivation)/digits);
    first_derivation = 2*a2+std::floor(static_cast<float>(t*first_derivation)/digits);
    first_derivation = a1+std::floor(static_cast<float>(t*first_derivation)/digits);
  }
  output.push_back(first_derivation/digits);

  int64_t position=0;
  if(first_derivation>=0)
  {
    position = a4+std::ceil(static_cast<float>(t*a5)/digits);
    position = a3+std::ceil(static_cast<float>(t*position)/digits);
    position = a2+std::ceil(static_cast<float>(t*position)/digits);
    position = a1+std::ceil(static_cast<float>(t*position)/digits);
    position = a0+std::ceil(static_cast<float>(t*position)/digits);
  }
  else
  {
    position = a4+std::floor(static_cast<float>(t*a5)/digits);
    position = a3+std::floor(static_cast<float>(t*position)/digits);
    position = a2+std::floor(static_cast<float>(t*position)/digits);
    position = a1+std::floor(static_cast<float>(t*position)/digits);
    position = a0+std::floor(static_cast<float>(t*position)/digits);
  }
  output.push_back(position/digits);

  return output;
}
