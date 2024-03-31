
/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <iostream>
#include <fstream>

/* external project header files */
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
/* internal project header files */

grid_map::GridMap init_grid_map(std::string MapFilePath)
{
    setlocale(LC_ALL, ""); // 可输出中文

    // grid_map setup (pose, size, frame, resolution)
    grid_map::GridMap mapData;
    mapData.setGeometry(grid_map::Length(15, 7), 0.1);
    mapData.setPosition(grid_map::Position(4, 0));
    mapData.setFrameId("odom");

    // 添加图层
    mapData.add("elevation"); // 高程图

    // 添加法向量层
    std::string outputLayersPrefix_ = "normal_";
    mapData.add(outputLayersPrefix_ + "x");
    mapData.add(outputLayersPrefix_ + "y");
    mapData.add(outputLayersPrefix_ + "z");

    // mapData.add("workspace"); // 工作空间
    mapData.add("3D_feeling", grid_map::Matrix::Zero(mapData.getSize()(0), mapData.getSize()(1)));

    // mapData.add("availableFootholds"); // 可落足点
    for (grid_map::GridMapIterator iterator(mapData); !iterator.isPastEnd(); ++iterator)
    {
        mapData.at("3D_feeling", *iterator) = -0.6;
    }

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
        for (int i = 0; i < (int)vectorData.size(); i += 3)
        {
            grid_map::Position gm_position;
            gm_position << vectorData[i], vectorData[i + 1];
            grid_map::Index gm_index;
            mapData.getIndex(gm_position, gm_index);
            mapData.at("elevation", gm_index) = vectorData[i + 2];
            mapData.at("3D_feeling", gm_index) = vectorData[i + 2];

            // debug
            // std::cout << vectorData[i] << "," << vectorData[i + 1] << std::endl; // 输出地形坐标
        }
    }
    else // 没有该文件
    {
        std::cout << "There is no map data. static_information.cpp" << std::endl;
        exit(1);
    }

    // 2.把机器人初始(默认)落足点加入到elevation图层中
    // for (int i = 0; i < 6; ++i)
    // {
    //     grid_map::Index gm_index;
    //     grid_map::Position gm_position;
    //     gm_position.x() = Hexapod_defaultFoothold[i].x();
    //     gm_position.y() = Hexapod_defaultFoothold[i].y();
    //     mapData.getIndex(gm_position, gm_index);
    //     mapData.at("elevation", gm_index) = 0;
    // }

    // // 4.发布地图数据
    // ROS_INFO("创建的grid_map地图大小为 %d x %d m (%d x %d cells).", (int)mapData.getLength().x(), (int)mapData.getLength().y(), (int)mapData.getSize()(0), (int)mapData.getSize()(1));

    // 计算法向量
    // computeWithAreaSerial(mapData, "elevation", outputLayersPrefix_);

    return mapData;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "grid_map_pub");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10.0);

    std::string MapFilePath;
    nh.getParam("terrain_file", MapFilePath);

    //------静态地图相关------
    ros::Publisher gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true); // 地图锁存住
    const grid_map::GridMap mapData = init_grid_map(MapFilePath);
    ROS_INFO("话题发布的的grid_map地图大小为 %d x %d m (%d x %d cells).",
             (int)mapData.getLength().x(), (int)mapData.getLength().y(),
             (int)mapData.getSize()(0), (int)mapData.getSize()(1));

    grid_map_msgs::GridMap gm_message;                             // 创建grid_map消息
    grid_map::GridMapRosConverter::toMessage(mapData, gm_message); // 把grid_map地图转换为ros消息格式

    while (ros::ok())
    {
        gridMapPub.publish(gm_message);
        loop_rate.sleep();
    }

    //------ros回头------
    // ros::spin();

    return 0;
}
