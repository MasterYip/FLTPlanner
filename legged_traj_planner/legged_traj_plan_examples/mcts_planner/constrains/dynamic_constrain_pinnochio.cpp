#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own modeldirectory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;
  const std::string robots_model_path = PINOCCHIO_MODEL_DIR;
  
  // You should change here to set up your own URDF file
  const std::string urdf_filename = robots_model_path + std::string("/example-robot-data/robots/talos_data/robots/talos_reduced.urdf");
  // You should change here to set up your own SRDF file
  const std::string srdf_filename = robots_model_path + std::string("/example-robot-data/robots/talos_data/srdf/talos.srdf");
  
  // Load the URDF model contained in urdf_filename
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  
  

  
  return 0;
}
