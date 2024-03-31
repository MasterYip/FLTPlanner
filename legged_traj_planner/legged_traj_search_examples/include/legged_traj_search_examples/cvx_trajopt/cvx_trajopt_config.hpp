#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <string>
#include <vector>
/* external project header files */
#include <Eigen/Eigen>
#include <ros/ros.h>
/* internal project header files */


struct CVX_TrajOpt_Config
{
    std::string mapTopic;
    // std::string meshTopic;
    // std::string edgeTopic;
    // std::string vertexTopic;
    // std::string interiorTopic;
    // int randomTries;
    // double randomScale;
    // int redundantTryH;
    double testRate;

    inline void loadParameters(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        // nh_priv.getParam("MeshTopic", meshTopic);
        // nh_priv.getParam("EdgeTopic", edgeTopic);
        // nh_priv.getParam("VertexTopic", vertexTopic);
        // nh_priv.getParam("InteriorTopic", interiorTopic);
        // nh_priv.getParam("RandomTries", randomTries);
        // nh_priv.getParam("RandomScale", randomScale);
        // nh_priv.getParam("RedundantTryH", redundantTryH);
        nh_priv.getParam("TestRate", testRate);
        return;
    }
};



