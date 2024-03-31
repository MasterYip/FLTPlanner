#include <user.h>
#include "grid_map_core/iterators/CircleIterator.hpp"
#include <eigen3/Eigen/Dense>
#include <HexapodParameter.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm> // 包含 std::remove_if 和 std::isspace 函数




namespace USER
{
    void areaSingleNormalComputation(grid_map::GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix,
                                                        const grid_map::Index& index) {
        // Requested position (center) of circle in map.
        grid_map::Position center;
        map.getPosition(index, center);
        double estimationRadius_ = 0.3;
        // Prepare data computation. Check if area is bigger than cell.
        const double minAllowedEstimationRadius = 0.5 * map.getResolution();
        if (estimationRadius_ <= minAllowedEstimationRadius) {
            // ROS_WARN("Estimation radius is smaller than allowed by the map resolution (%f < %f)", estimationRadius_, minAllowedEstimationRadius);
        }

        // Gather surrounding data.
        size_t nPoints = 0;
        grid_map::Position3 sum = grid_map::Position3::Zero();
        Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();
        for (grid_map::CircleIterator circleIterator(map, center, estimationRadius_); !circleIterator.isPastEnd(); ++circleIterator) {
            grid_map::Position3 point;
            if (!map.getPosition3(inputLayer, *circleIterator, point)) {
            continue;
            }
            nPoints++;
            sum += point;
            sumSquared.noalias() += point * point.transpose();
        }

        Eigen::Matrix<double, 3, 1>  unitaryNormalVector = Eigen::Matrix<double, 3, 1> ::Zero();
        if (nPoints < 3) {
            // ROS_DEBUG("Not enough points to establish normal direction (nPoints = %i)", static_cast<int>(nPoints));
            unitaryNormalVector = Eigen::Matrix<double, 3, 1> ::UnitZ();
        } else {
            const grid_map::Position3 mean = sum / nPoints;
            const Eigen::Matrix3d covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();

            // Compute Eigenvectors.
            // Eigenvalues are ordered small to large.
            // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
            solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
            if (solver.eigenvalues()(1) > 1e-8) {
            unitaryNormalVector = solver.eigenvectors().col(0);
            } else {  // If second eigenvalue is zero, the normal is not defined.
            // ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated.");
            // ROS_DEBUG("Expected cause: data is on a straight line (nPoints = %i)", static_cast<int>(nPoints));
            unitaryNormalVector = Eigen::Matrix<double, 3, 1> ::UnitZ();
            }
        }
        Eigen::Vector3d normalVectorPositiveAxis_ = Eigen::Matrix<double, 3, 1>::UnitZ();
        // Check direction of the normal vector and flip the sign towards the user defined direction.
        if (unitaryNormalVector.dot(normalVectorPositiveAxis_) < 0.0) {
            unitaryNormalVector = -unitaryNormalVector;
        }

        map.at(outputLayersPrefix + "x", index) = unitaryNormalVector.x();
        map.at(outputLayersPrefix + "y", index) = unitaryNormalVector.y();
        map.at(outputLayersPrefix + "z", index) = unitaryNormalVector.z();
    }



    // SVD Area based methods.
    void computeWithAreaSerial(grid_map::GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix) {
        // For each cell in submap.
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
            // Check if this is an empty cell (hole in the map).
            if (map.isValid(*iterator, inputLayer)) {
            const grid_map::Index index(*iterator);
            areaSingleNormalComputation(map, inputLayer, outputLayersPrefix, index);
            }
        }
    }


    bool calculateNormalVector(const grid_map::GridMap& mapIn, grid_map::GridMap& mapOut, 
        std::string& inputLayer_, const std::string& outputLayersPrefix_) {

        std::vector<std::string> normalVectorsLayers;
        normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
        normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
        normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

        mapOut = mapIn;
        for (const auto& layer : normalVectorsLayers) {
            mapOut.add(layer);
        }
        computeWithAreaSerial(mapOut, inputLayer_, outputLayersPrefix_);

        return true;
    }


    // 函数用于删除字符串两端的空格
    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t"); // 查找第一个非空格字符
        if (std::string::npos == first) {
            return str;
        }
        size_t last = str.find_last_not_of(" \t"); // 查找最后一个非空格字符
        return str.substr(first, (last - first + 1));
    }


    std::map<std::string, std::string> readConfigParameters(void)
    {
        std::ifstream configFile(configFilePath);
        
        std::map<std::string, std::string> configMap;

        if (configFile.is_open()) {
            std::string line;
            while (std::getline(configFile, line)) {
                // std::cout << "Read line: " << line << std::endl;

                // 忽略注释和空行
                if (!line.empty() && line[0] != '#') {
                    size_t delimiterPos = line.find('=');
                    if (delimiterPos != std::string::npos) {
                        std::string key = trim(line.substr(0, delimiterPos));
                        std::string value = trim(line.substr(delimiterPos + 1));
                        // std::cout << "Key: " << key << ", Value: " << value << std::endl;

                        configMap[key] = value;
                    }
                }
            }
            configFile.close();

        } else {
            std::cerr << "Failed to open config file." << std::endl;
        }

        return configMap;
    }

    void initRobotPinocchoModel(void)
    {
        pinocchio::urdf::buildModel(USER::robotURDF_Path, USER::HexMini_PinoModel);
        USER::HexMini_PinoModel_Data = pinocchio::Data(USER::HexMini_PinoModel);
    }


    grid_map::GridMap init_grid_map(std::map<std::string, std::string> configMap)
    {
        setlocale(LC_ALL, ""); // 可输出中文
        std::string MapFilePath = configMap.at("MapFilePath");

        std::string MapFileType = configMap.at("MapFileType");

        // grid_map对象,地图大小设置,坐标系设置
        grid_map::GridMap mapData;
        float MapLength = std::stof(USER::configMap.at("MapLength"));
        float MapWidth = std::stof(USER::configMap.at("MapWidth"));
        float GridLength = std::stof(USER::configMap.at("GridLength"));
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


        // mapData.add("workspace"); // 工作空间
        // mapData.add("3D_feeling", grid_map::Matrix::Zero(mapData.getSize()(0), mapData.getSize()(1)));

        // mapData.add("availableFootholds"); // 可落足点
        // for (grid_map::GridMapIterator iterator(mapData); !iterator.isPastEnd(); ++iterator)
        // {
        //     mapData.at("3D_feeling", *iterator) = -0.6;
        // }




        // 1.从文件读取地图数据,加入到elevation图层中
        std::ifstream fin(MapFilePath);
        std::string line_info, input_result;
        std::vector<float> vectorData;
        if (fin) // 有该文件
        {
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

            if(MapFileType == "3D")
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
            else if(MapFileType == "2D")
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

        }
        else // 没有该文件
        {
            std::cout << "There is no map data. planFile" << std::endl;
            exit(1);
        }



        // 2.把机器人初始(默认)落足点加入到elevation图层中
        for (int i = 0; i < 6; ++i)
        {
            grid_map::Index gm_index;
            grid_map::Position gm_position;
            gm_position.x() = HexapodParameter::norminalFoothold_B[i].x();
            gm_position.y() = HexapodParameter::norminalFoothold_B[i].y();
            mapData.getIndex(gm_position, gm_index);
            mapData.at("elevation", gm_index) = 0;
        }



        // // 4.发布地图数据
        // ROS_INFO("创建的grid_map地图大小为 %d x %d m (%d x %d cells).", (int)mapData.getLength().x(), (int)mapData.getLength().y(), (int)mapData.getSize()(0), (int)mapData.getSize()(1));
        
        // 计算法向量
        computeWithAreaSerial(mapData, "elevation", outputLayersPrefix_);

        std::cout << " map is ready !" << std::endl;
        return mapData;
    }
    



	std::map<std::string, std::string> configMap = readConfigParameters();
    grid_map::GridMap mapData;
    const std::string key_element = "&abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^*()1234567890[]{}<>"; // HashKey 子元素 !@#$%^*()1234567890[]{}<>
    const int max_depth = MAX_DEPTH; // 最大深度

    const bool COLLISION_CHECK = configMap.at("COLLISION_CHECK") == "true" ? true : false; // 碰撞检测
    const bool IsMaxForceConstraint = configMap.at("IsMaxForceConstraint") == "true" ? true : false; 
    const bool IsToruqeLimitConstraint = configMap.at("IsToruqeLimitConstraint") == "true" ? true : false; 
    const bool IsVirtualLoss = configMap.at("IsVirtualLoss") == "true" ? true : false; 
    const bool IsBestBP = configMap.at("IsBestBP") == "true" ? true : false; 


    const std::string elevationLayerName =  USER::configMap.at("ElevationLayerName");
    const std::string normalVector_x_Name = USER::configMap.at("NormalVector_x_Name");
    const std::string normalVector_y_Name = USER::configMap.at("NormalVector_y_Name");
    const std::string normalVector_z_Name = USER::configMap.at("NormalVector_z_Name");

    const std::string robotURDF_Path = USER::configMap.at("RobotURDF_Path");

    const float SEARCH_TIME_LIMIT =  std::stof(USER::configMap.at("MPI_run_time"));
    const int JOB_FACTOR = std::stoi(USER::configMap.at("JOB_FACTOR"));

    const int GaitMode = std::stoi(USER::configMap.at("GaitMode"));

    const int simStepNum = 10; // 模拟步数
    const std::string availableFootholdLayerName = "elevation";
    const float LegWorkspaceR = 0.86;

    const float norminalTrunkHeight = 0.25;

    pinocchio::Model HexMini_PinoModel;

    pinocchio::Data HexMini_PinoModel_Data;


    // HexMini的URDF FrameName
    std::vector<std::string> FootName = {"RF_FOOT", "RM_FOOT", "RB_FOOT",
                                        "LF_FOOT", "LM_FOOT", "LB_FOOT"
                                        };

    std::vector<std::string> ShankName = {"RF_SHANK", "RM_SHANK", "RB_SHANK",
                                        "LF_SHANK", "LM_SHANK", "LB_SHANK"
                                        };

    std::vector<std::string> ThighName = {"RF_THIGH", "RM_THIGH", "RB_THIGH",
                                        "LF_THIGH", "LM_THIGH", "LB_THIGH"
                                        };



}

