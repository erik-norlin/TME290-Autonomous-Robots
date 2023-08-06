#define CATCH_CONFIG_MAIN	

#include "catch.hpp"
#include "prime-checker.hpp"

TEST_CASE("Test PrimeChecker 1.") {
  PrimeChecker pc;
  REQUIRE(pc.isPrime(5));
 }
 
 TEST_CASE("Test PrimeChecker 2.") {
  PrimeChecker pc;
  REQUIRE(!pc.isPrime(4));
 }
 
 TEST_CASE("Test PrimeChecker 3.") {
  PrimeChecker pc;
  REQUIRE(pc.isPrime(3));
 }
 
 TEST_CASE("Test PrimeChecker 4.") {
  PrimeChecker pc;
  REQUIRE(pc.isPrime(2));
 }
