#include <iostream>

#include "serial.hpp"

bool test_nonce() {
  return true;
}

int main() {
  std::cout << "Start test_nonce..." << std::endl;
  bool nonce = test_nonce();
  std::cout << "test_nonce: " << nonce ? "Succeeded": "Failed" << std::endl;
}
