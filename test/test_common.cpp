/**
* @Function: Test RTKLIB common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include <iostream>
#include <Eigen/Core>

#include "gici/utility/transform.h"
#include "gici/estimate/common_parameter_block.h"

using namespace std;
using namespace gici;

int main(void)
{
  CommonParameterBlock<1, CommonParameterBlockType::Ionosphere> parameter_block;
  std::cout << parameter_block.typeInfo() << std::endl;
  parameter_block.setEstimate(Eigen::VectorXd::Ones(1) * 1.4);
  std::cout << parameter_block.estimate() << std::endl;
  return 1;
}