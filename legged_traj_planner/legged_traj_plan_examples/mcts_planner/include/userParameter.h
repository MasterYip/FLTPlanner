#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <string>
#include <vector>
#include <map>
#include <myDataType.h>
#include <grid_map_core/GridMap.hpp>
#include "constrains/util.hh"

class Parameters {
public:
    Parameters(grid_map::GridMap &mapData_);

    // GridMap parameters
    double MapLength;
    double MapWidth;
    double GridLength;
    
    std::string MapFilePath;
    std::string MapFileType;
    std::string elevationLayerName;
    std::string normalVector_x_Name;
    std::string normalVector_y_Name;
    std::string normalVector_z_Name;

    
    std::string foothold_select_type;

    // Robot parameters
    double workspaceReductionDis;
    double staticStabilityMarginThreshold;
    double norminalFootStepLengthInWorkspace;
    double kinematicMargin;
    double norminalTrunkHeight;
    double nominal_y_shift;
    float onlyRotateThreshold = 0.05;

    const double Mass = 30; // 机器人质量

    // Simulation parameters
    float SEARCH_TIME_LIMIT;
    int JOB_FACTOR;
    int GaitMode;
    const int simStepNum = 10;

    // Constraint flags
    bool COLLISION_CHECK;
    bool IsMaxForceConstraint;
    bool IsToruqeLimitConstraint;
    bool IsVirtualLoss;
    bool IsBestBP;
    bool USE_POWER_MEAN;

    bool IsTest;

    // Constants
    int max_depth = 50;
    std::string key_element= "&abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^*()1234567890[]{}<>";
    std::string availableFootholdLayerName = "elevation";

    // URDF Frame Names
    std::vector<std::string> FootName;
    std::vector<std::string> ShankName;
    std::vector<std::string> ThighName;

    // Pinocchio model
    pinocchio::Model HexMini_PinoModel;
    pinocchio::Data HexMini_PinoModel_Data;

    // GridMap
    grid_map::GridMap mapData;

    // Config file parameters
    std::map<std::string, std::string> configMap;

    void printParameters() const;

    // 初始化默认落足点位置
    MDT::POINT norminalFoothold_B[6];

    // 约束凸包
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini12;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini3;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini45;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini6;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini12_reduced;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini3_reduced;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini45_reduced;
    std::pair<MatrixXX, VectorX>  Ab_Ji_foot_forHexMini6_reduced;


    void init_grid_map(grid_map::GridMap &mapData);


private:
    void loadFromConfigFile();
    std::string trim(const std::string& str);
    std::map<std::string, std::string> readConfigParameters();
    void areaSingleNormalComputation(grid_map::GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix, const grid_map::Index& index);
    void computeWithAreaSerial(grid_map::GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);
    
    MDT::POINT init_defaultFoothold(const int &num);

    MatrixX3 getRawLegWorkspaceVertices();
    void setReducedB(VectorX& bb);
    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot12(bool isReduce);
    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot3(bool isReduce);
    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot45(bool isReduce);
    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot6(bool isReduce);
};

#endif // PARAMETERS_HPP