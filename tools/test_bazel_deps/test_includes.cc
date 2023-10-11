#include <iostream>
#include

namespace elevation_mapping {

int do_main() {
  std::cout << "Success" << std::endl;
  return 0;
}

}

int main(int argc, char** argv) {
  return elevation_mapping::do_main();
}