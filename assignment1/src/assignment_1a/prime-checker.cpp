#include "prime-checker.hpp"

bool PrimeChecker::isPrime(uint16_t n) {
  bool retVal{true};
  if (n < 2 || (n != 2 && n%2 == 0)) {
    retVal = false;
  }
  else {
    for(uint16_t i{3}; (i*i) <= n; i += 2) {
      if (n%i == 0) {
        retVal = false;
        break;
      }
    }
  }
  return retVal;
}
