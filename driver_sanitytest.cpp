#include <iostream>

extern "C" void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode );

int main() {
  void *returnVal = HmdDriverFactory("pascetti", 0);
  std::cout << "fuck";
  return 0;
}
