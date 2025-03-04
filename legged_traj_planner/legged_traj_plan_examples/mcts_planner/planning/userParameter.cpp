#include "grid_map_core/iterators/CircleIterator.hpp"
#include <eigen3/Eigen/Dense>
#include <HexapodParameter.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm> // 包含 std::remove_if 和 std::isspace 函数
#include <iostream>

#include <sstream> //字符串操作,搭配fstream读取文件数据

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <map>
#include "constrains/my_cdd.hh"
#include <userParameter.h>
#include "config.h"

// 构造函数
Parameters::Parameters(grid_map::GridMap &mapData_) : mapData(mapData_)
{
    // 读取文件参数
    this->loadFromConfigFile();

    // HexMini的URDF FrameName
    this->FootName = {"RF_FOOT", "RM_FOOT", "RB_FOOT", "LF_FOOT", "LM_FOOT", "LB_FOOT"};
    this->ShankName = {"RF_SHANK", "RM_SHANK", "RB_SHANK", "LF_SHANK", "LM_SHANK", "LB_SHANK"};
    this->ThighName = {"RF_THIGH", "RM_THIGH", "RB_THIGH", "LF_THIGH", "LM_THIGH", "LB_THIGH"};

    // 初始化足端运动学凸包参数
    Ab_Ji_foot_forHexMini12 = this->initHexMini_Ab_Ji_foot12(false);
    Ab_Ji_foot_forHexMini3 = this->initHexMini_Ab_Ji_foot3(false);
    Ab_Ji_foot_forHexMini45 = this->initHexMini_Ab_Ji_foot45(false);
    Ab_Ji_foot_forHexMini6 = this->initHexMini_Ab_Ji_foot6(false);
    Ab_Ji_foot_forHexMini12_reduced = this->initHexMini_Ab_Ji_foot12(true);
    Ab_Ji_foot_forHexMini3_reduced = this->initHexMini_Ab_Ji_foot3(true);
    Ab_Ji_foot_forHexMini45_reduced = this->initHexMini_Ab_Ji_foot45(true);
    Ab_Ji_foot_forHexMini6_reduced = this->initHexMini_Ab_Ji_foot6(true);

    // 设置默认落足点
    for (size_t i = 0; i < 6; i++)
    {
        this->norminalFoothold_B[i] = init_defaultFoothold(i);
    }

    // 设定地图数据
    this->init_grid_map(this->mapData);

    // 初始化机器人模型
    pinocchio::urdf::buildModel(URDF_FILE_PATH, this->HexMini_PinoModel);
    this->HexMini_PinoModel_Data = pinocchio::Data(this->HexMini_PinoModel);
}

void Parameters::loadFromConfigFile()
{
    this->configMap = this->readConfigParameters();

    this->MapLength = std::stof(this->configMap.at("MapLength"));
    this->MapWidth = std::stof(this->configMap.at("MapWidth"));
    this->GridLength = std::stof(this->configMap.at("GridLength"));
    this->MapFilePath = this->configMap.at("MapFilePath");
    this->MapFileType = this->configMap.at("MapFileType");
    this->elevationLayerName = this->configMap.at("ElevationLayerName");
    this->normalVector_x_Name = this->configMap.at("NormalVector_x_Name");
    this->normalVector_y_Name = this->configMap.at("NormalVector_y_Name");
    this->normalVector_z_Name = this->configMap.at("NormalVector_z_Name");

    this->foothold_select_type = this->configMap.at("foothold_select_type");

    this->workspaceReductionDis = std::stof(this->configMap.at("workspaceReductionDis_"));
    this->staticStabilityMarginThreshold = std::stof(this->configMap.at("staticStabilityMarginThreshold"));
    this->norminalFootStepLengthInWorkspace = std::stof(this->configMap.at("norminalFootStepLengthInWorkspace"));
    this->kinematicMargin = std::stof(this->configMap.at("kinematicMargin"));

    this->norminalTrunkHeight = std::stof(this->configMap.at("norminalTrunkHeight"));
    this->nominal_y_shift = std::stof(this->configMap.at("nominal_y_shift"));

    this->SEARCH_TIME_LIMIT = std::stof(this->configMap.at("MPI_run_time"));
    this->JOB_FACTOR = std::stoi(this->configMap.at("JOB_FACTOR"));
    this->GaitMode = std::stoi(this->configMap.at("GaitMode"));

    this->COLLISION_CHECK = this->configMap.at("COLLISION_CHECK") == "true";
    this->IsMaxForceConstraint = this->configMap.at("IsMaxForceConstraint") == "true";
    this->IsToruqeLimitConstraint = this->configMap.at("IsToruqeLimitConstraint") == "true";
    this->IsVirtualLoss = this->configMap.at("IsVirtualLoss") == "true";
    this->IsBestBP = this->configMap.at("IsBestBP") == "true";
    this->USE_POWER_MEAN = this->configMap.at("USE_POWER_MEAN") == "true";
    this->IsTest = this->configMap.at("IsTest") == "true";
}

// 打印参数
void Parameters::printParameters() const
{
    std::cout << "GridMap Parameters:" << std::endl;
    std::cout << "MapLength: " << MapLength << std::endl;
    std::cout << "MapWidth: " << MapWidth << std::endl;
    std::cout << "GridLength: " << GridLength << std::endl;
    std::cout << "MapFilePath: " << MapFilePath << std::endl;
    std::cout << "MapFileType: " << MapFileType << std::endl;
    std::cout << "ElevationLayerName: " << elevationLayerName << std::endl;
    std::cout << "normalVector_x_Name: " << normalVector_x_Name << std::endl;
    std::cout << "normalVector_y_Name: " << normalVector_y_Name << std::endl;
    std::cout << "normalVector_z_Name: " << normalVector_z_Name << std::endl;

    std::cout << "\nRobot Parameters:" << std::endl;
    std::cout << "workspaceReductionDis: " << workspaceReductionDis << std::endl;
    std::cout << "staticStabilityMarginThreshold: " << staticStabilityMarginThreshold << std::endl;
    std::cout << "norminalFootStepLengthInWorkspace: " << norminalFootStepLengthInWorkspace << std::endl;
    std::cout << "kinematicMargin: " << kinematicMargin << std::endl;
    std::cout << "norminalTrunkHeight: " << norminalTrunkHeight << std::endl;
    std::cout << "nominal_y_shift: " << nominal_y_shift << std::endl;
    std::cout << "onlyRotateThreshold: " << onlyRotateThreshold << std::endl;

    std::cout << "\nSimulation Parameters:" << std::endl;
    std::cout << "SEARCH_TIME_LIMIT: " << SEARCH_TIME_LIMIT << std::endl;
    std::cout << "JOB_FACTOR: " << JOB_FACTOR << std::endl;
    std::cout << "GaitMode: " << GaitMode << std::endl;
    std::cout << "simStepNum: " << simStepNum << std::endl;

    std::cout << "\nConstraint Flags:" << std::endl;
    std::cout << "COLLISION_CHECK: " << (COLLISION_CHECK ? "true" : "false") << std::endl;
    std::cout << "IsMaxForceConstraint: " << (IsMaxForceConstraint ? "true" : "false") << std::endl;
    std::cout << "IsToruqeLimitConstraint: " << (IsToruqeLimitConstraint ? "true" : "false") << std::endl;
    std::cout << "IsVirtualLoss: " << (IsVirtualLoss ? "true" : "false") << std::endl;
    std::cout << "IsBestBP: " << (IsBestBP ? "true" : "false") << std::endl;

    std::cout << "\nConstants:" << std::endl;
    std::cout << "MAX_DEPTH: " << max_depth << std::endl;
    std::cout << "key_element: " << key_element << std::endl;
    std::cout << "availableFootholdLayerName: " << availableFootholdLayerName << std::endl;

    std::cout << "\nURDF Frame Names:" << std::endl;
    std::cout << "Foot Names: ";
    for (const auto &foot : FootName)
    {
        std::cout << foot << " ";
    }
    std::cout << std::endl;

    std::cout << "Shank Names: ";
    for (const auto &shank : ShankName)
    {
        std::cout << shank << " ";
    }
    std::cout << std::endl;

    std::cout << "Thigh Names: ";
    for (const auto &thigh : ThighName)
    {
        std::cout << thigh << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < 6; ++i)
    {
        std::cout << "norminalFoothold_B[" << i << "]: " << norminalFoothold_B[i].transpose() << std::endl;
    }

    // 如果需要打印 Pinocchio 模型和 GridMap 数据，可以在此处添加逻辑
}

// 函数用于删除字符串两端的空格
std::string Parameters::trim(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t"); // 查找第一个非空格字符
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t"); // 查找最后一个非空格字符
    return str.substr(first, (last - first + 1));
}

std::map<std::string, std::string> Parameters::readConfigParameters()
{
    std::ifstream configFile(configFilePath);

    std::map<std::string, std::string> configMap;

    if (configFile.is_open())
    {
        std::string line;
        while (std::getline(configFile, line))
        {
            // std::cout << "Read line: " << line << std::endl;

            // 忽略注释和空行
            if (!line.empty() && line[0] != '#')
            {
                size_t delimiterPos = line.find('=');
                if (delimiterPos != std::string::npos)
                {
                    std::string key = this->trim(line.substr(0, delimiterPos));
                    std::string value = this->trim(line.substr(delimiterPos + 1));
                    // std::cout << "Key: " << key << ", Value: " << value << std::endl;

                    configMap[key] = value;
                }
            }
        }
        configFile.close();
    }
    else
    {
        std::cerr << "Failed to open config file." << std::endl;
    }

    return configMap;
}

// 计算法向量子函数
void Parameters::areaSingleNormalComputation(grid_map::GridMap &map, const std::string &inputLayer, const std::string &outputLayersPrefix,
                                             const grid_map::Index &index)
{
    // Requested position (center) of circle in map.
    grid_map::Position center;
    map.getPosition(index, center);
    double estimationRadius_ = 0.3;
    // Prepare data computation. Check if area is bigger than cell.
    const double minAllowedEstimationRadius = 0.5 * map.getResolution();
    if (estimationRadius_ <= minAllowedEstimationRadius)
    {
        // ROS_WARN("Estimation radius is smaller than allowed by the map resolution (%f < %f)", estimationRadius_, minAllowedEstimationRadius);
    }

    // Gather surrounding data.
    size_t nPoints = 0;
    grid_map::Position3 sum = grid_map::Position3::Zero();
    Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();
    for (grid_map::CircleIterator circleIterator(map, center, estimationRadius_); !circleIterator.isPastEnd(); ++circleIterator)
    {
        grid_map::Position3 point;
        if (!map.getPosition3(inputLayer, *circleIterator, point))
        {
            continue;
        }
        nPoints++;
        sum += point;
        sumSquared.noalias() += point * point.transpose();
    }

    Eigen::Matrix<double, 3, 1> unitaryNormalVector = Eigen::Matrix<double, 3, 1>::Zero();
    if (nPoints < 3)
    {
        // ROS_DEBUG("Not enough points to establish normal direction (nPoints = %i)", static_cast<int>(nPoints));
        unitaryNormalVector = Eigen::Matrix<double, 3, 1>::UnitZ();
    }
    else
    {
        const grid_map::Position3 mean = sum / nPoints;
        const Eigen::Matrix3d covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();

        // Compute Eigenvectors.
        // Eigenvalues are ordered small to large.
        // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
        solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
        if (solver.eigenvalues()(1) > 1e-8)
        {
            unitaryNormalVector = solver.eigenvectors().col(0);
        }
        else
        { // If second eigenvalue is zero, the normal is not defined.
            // ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated.");
            // ROS_DEBUG("Expected cause: data is on a straight line (nPoints = %i)", static_cast<int>(nPoints));
            unitaryNormalVector = Eigen::Matrix<double, 3, 1>::UnitZ();
        }
    }
    Eigen::Vector3d normalVectorPositiveAxis_ = Eigen::Matrix<double, 3, 1>::UnitZ();
    // Check direction of the normal vector and flip the sign towards the user defined direction.
    if (unitaryNormalVector.dot(normalVectorPositiveAxis_) < 0.0)
    {
        unitaryNormalVector = -unitaryNormalVector;
    }

    map.at(outputLayersPrefix + "x", index) = unitaryNormalVector.x();
    map.at(outputLayersPrefix + "y", index) = unitaryNormalVector.y();
    map.at(outputLayersPrefix + "z", index) = unitaryNormalVector.z();
}

// 计算法向量子函数
// SVD Area based methods.
void Parameters::computeWithAreaSerial(grid_map::GridMap &map, const std::string &inputLayer, const std::string &outputLayersPrefix)
{
    // For each cell in submap.
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
    {
        // Check if this is an empty cell (hole in the map).
        if (map.isValid(*iterator, inputLayer))
        {
            const grid_map::Index index(*iterator);
            this->areaSingleNormalComputation(map, inputLayer, outputLayersPrefix, index);
        }
    }
}

void Parameters::init_grid_map(grid_map::GridMap &mapData)
{
    // 1.从文件读取地图数据,加入到elevation图层中
    std::ifstream fin(MapFilePath);
    std::string line_info, input_result;
    std::vector<float> vectorData;
    if (fin) // 有该文件
    {
        setlocale(LC_ALL, ""); // 可输出中文
        std::string MapFilePath = this->configMap.at("MapFilePath");

        std::string MapFileType = this->configMap.at("MapFileType");

        // grid_map对象,地图大小设置,坐标系设置
        float MapLength = std::stof(this->configMap.at("MapLength"));
        float MapWidth = std::stof(this->configMap.at("MapWidth"));
        float GridLength = std::stof(this->configMap.at("GridLength"));
        mapData.setGeometry(grid_map::Length(MapLength, MapWidth), GridLength);
        mapData.setFrameId("odom");

        // 添加图层
        mapData.add("elevation"); // 高程图
        // mapData.add("elevation",grid_map::Matrix::Zero(mapData.getSize()(0),mapData.getSize()(1)));

        // 添加法向量层
        std::string outputLayersPrefix_ = "normal_";
        mapData.add(outputLayersPrefix_ + "x");
        mapData.add(outputLayersPrefix_ + "y");
        mapData.add(outputLayersPrefix_ + "z");

        std::cout << "*************************************************" << std::endl;
        std::cout << "Find map file: " << MapFilePath << std::endl;
        std::cout << "*************************************************" << std::endl;
        while (getline(fin, line_info)) // line中不包括每行的换行符
        {
            std::stringstream inputDATA(line_info);
            while (inputDATA >> input_result)
            {
                vectorData.push_back(atof(input_result.c_str()));
            }
        }

        if (MapFileType == "3D")
        {
            for (int i = 0; i < (int)vectorData.size(); i += 3)
            {
                grid_map::Position gm_position;
                gm_position << vectorData[i], vectorData[i + 1];
                grid_map::Index gm_index;
                mapData.getIndex(gm_position, gm_index);
                mapData.at("elevation", gm_index) = vectorData[i + 2];
            }
        }
        else if (MapFileType == "2D")
        {
            for (int i = 0; i < (int)vectorData.size(); i += 2)
            {
                grid_map::Position gm_position;
                gm_position << vectorData[i], vectorData[i + 1];
                grid_map::Index gm_index;
                mapData.getIndex(gm_position, gm_index);
                mapData.at("elevation", gm_index) = 0; // vectorData[i + 2];
            }
        }
        else
        {
            std::cout << "MapFileType is wrong !" << std::endl;
            exit(1);
        }
        // 2.把机器人初始(默认)落足点加入到elevation图层中
        for (int i = 0; i < 6; ++i)
        {
            grid_map::Index gm_index;
            grid_map::Position gm_position;
            gm_position.x() = this->norminalFoothold_B[i].x();
            gm_position.y() = this->norminalFoothold_B[i].y();
            mapData.getIndex(gm_position, gm_index);
            mapData.at("elevation", gm_index) = 0;
        }

        // // 4.发布地图数据
        // ROS_INFO("创建的grid_map地图大小为 %d x %d m (%d x %d cells).", (int)mapData.getLength().x(), (int)mapData.getLength().y(), (int)mapData.getSize()(0), (int)mapData.getSize()(1));

        // 计算法向量
        this->computeWithAreaSerial(mapData, "elevation", outputLayersPrefix_);

        std::cout << " map is ready !" << std::endl;
    }
    else // 没有该文件
    {
        std::cout << "There is no map data. planFile" << std::endl;
    }
}

// 机体base坐标系下六个默认落足点
MDT::POINT Parameters::init_defaultFoothold(const int &num)
{
    MatrixXX initFeetPosition_B_Frame(6, 3);

    // initFeetPosition_B_Frame.row(0) << 0.35350, -0.27608, -0.13143; // RF
    // initFeetPosition_B_Frame.row(1) << 0.05350, -0.33608, -0.13143; // RM
    // initFeetPosition_B_Frame.row(2) << -0.35350, -0.27608, -0.13143;// RB
    // initFeetPosition_B_Frame.row(3) << 0.35350,  0.27608, -0.13143; // LF
    // initFeetPosition_B_Frame.row(4) << 0.05350,  0.33608, -0.13143; // LM
    // initFeetPosition_B_Frame.row(5) << -0.35350,  0.27608, -0.13143;// LB

    initFeetPosition_B_Frame.row(0) << 0.35, -0.23 - this->nominal_y_shift, -this->norminalTrunkHeight;  // RF
    initFeetPosition_B_Frame.row(1) << 0.05, -0.29 - this->nominal_y_shift, -this->norminalTrunkHeight;  // RM
    initFeetPosition_B_Frame.row(2) << -0.35, -0.23 - this->nominal_y_shift, -this->norminalTrunkHeight; // RB
    initFeetPosition_B_Frame.row(3) << 0.35, 0.23 + this->nominal_y_shift, -this->norminalTrunkHeight;   // LF
    initFeetPosition_B_Frame.row(4) << 0.05, 0.29 + this->nominal_y_shift, -this->norminalTrunkHeight;   // LM
    initFeetPosition_B_Frame.row(5) << -0.35, 0.23 + this->nominal_y_shift, -this->norminalTrunkHeight;  // LB

    if (this->IsTest)
    {
        initFeetPosition_B_Frame.row(0) << 0.35, -0.23 - this->nominal_y_shift - 0.06, -this->norminalTrunkHeight;  // RF
        initFeetPosition_B_Frame.row(1) << 0.05, -0.29 - this->nominal_y_shift - 0.06, -this->norminalTrunkHeight;  // RM
        initFeetPosition_B_Frame.row(2) << -0.35, -0.23 - this->nominal_y_shift - 0.06, -this->norminalTrunkHeight; // RB
    }

    MDT::POINT p;
    p.x() = initFeetPosition_B_Frame(num, 0);
    p.y() = initFeetPosition_B_Frame(num, 1);
    p.z() = initFeetPosition_B_Frame(num, 2);

    return p;
}

MatrixX3 Parameters::getRawLegWorkspaceVertices()
{
    MatrixX3 VV(10, 3);
    VV.setZero();
    VV.row(0) << 0.2412, -0.154, -0.1303;
    VV.row(1) << -0, -0.21, -0.1464;
    VV.row(2) << -0.0, -0.165, -0.3889;
    VV.row(3) << 0.2556, -0.1674, -0.3545;
    VV.row(4) << -0.3199, -0.3958, 0.006312;
    VV.row(5) << -0.2209, -0.2967, -0.3344;
    VV.row(6) << 0.3721, -0.2772, 0.02371;
    VV.row(7) << 0.3527, -0.2589, -0.2644;
    VV.row(8) << 0.05979, -0.4186, -0.2857;
    VV.row(9) << 0.06059, -0.472, 0.1195;
    return VV;
}

void Parameters::setReducedB(VectorX &bb)
{
    // 将每个元素减去epsilon
    if (this->workspaceReductionDis >= 0.0 && this->workspaceReductionDis <= 0.3)
    {
        bb.array() -= this->workspaceReductionDis;
    }
    else
    {
        std::cout << "Error: this->workspaceReductionDis >= 0.0 && this->workspaceReductionDis <= 0.3" << std::endl;
        exit(0);
    }
}

std::pair<MatrixXX, VectorX> Parameters::initHexMini_Ab_Ji_foot12(bool isReduce)
{
    MatrixXX AA;
    VectorX bb;
    MatrixX3 VV = getRawLegWorkspaceVertices();

    bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
    if (!flag)
    {
        std::cout << "fail and exit" << std::endl;
        exit(0);
    }

    if (isReduce)
    {
        // 将每个元素减去epsilon
        setReducedB(bb);
    }

    return std::make_pair(AA, bb);
}

std::pair<MatrixXX, VectorX> Parameters::initHexMini_Ab_Ji_foot3(bool isReduce)
{
    MatrixXX AA;
    VectorX bb;

    MatrixX3 VV = getRawLegWorkspaceVertices();

    for (int i = 0; i < VV.rows(); ++i)
    {
        VV(i, 0) = -VV(i, 0);
    }
    bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
    if (!flag)
    {
        std::cout << "fail and exit" << std::endl;
        exit(0);
    }

    if (isReduce)
    {
        // 将每个元素减去epsilon
        setReducedB(bb);
    }

    return std::make_pair(AA, bb);
}

std::pair<MatrixXX, VectorX> Parameters::initHexMini_Ab_Ji_foot45(bool isReduce)
{
    MatrixXX AA;
    VectorX bb;

    MatrixX3 VV = getRawLegWorkspaceVertices();

    for (int i = 0; i < VV.rows(); ++i)
    {
        VV(i, 1) = -VV(i, 1);
    }
    bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
    if (!flag)
    {
        std::cout << "fail and exit" << std::endl;
        exit(0);
    }

    if (isReduce)
    {
        // 将每个元素减去epsilon
        setReducedB(bb);
    }

    return std::make_pair(AA, bb);
}

std::pair<MatrixXX, VectorX> Parameters::initHexMini_Ab_Ji_foot6(bool isReduce)
{
    MatrixXX AA;
    VectorX bb;

    MatrixX3 VV = getRawLegWorkspaceVertices();

    for (int i = 0; i < VV.rows(); ++i)
    {
        VV(i, 1) = -VV(i, 1);
        VV(i, 0) = -VV(i, 0);
    }
    bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
    if (!flag)
    {
        std::cout << "fail and exit" << std::endl;
        exit(0);
    }

    if (isReduce)
    {
        // 将每个元素减去epsilon
        setReducedB(bb);
    }

    return std::make_pair(AA, bb);
}