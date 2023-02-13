#include <string>
#ifndef PARAMETER_HPP
#define PARAMETER_HPP
using namespace std;
namespace wit {
class Parameter {
 public:
  Parameter() : wit_node_port_("/dev/ttyUSB0"), baut_rate_(9600), ns("wit") {}

  string wit_node_port_;
  int baut_rate_;
  string ns;
};
}
#endif  // PARAMETER_HPP
