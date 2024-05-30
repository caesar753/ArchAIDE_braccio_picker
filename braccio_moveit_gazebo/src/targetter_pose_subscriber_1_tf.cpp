#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "custom_msgs/matrix.h"
#include "custom_msgs/target.h"
#include "auto_targetter.h"
#include "pose_goal_targetter.hpp"
#include "vision_utils.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "targetter_pose_subscriber");
    ros::NodeHandle nh;

    auto_targetter::BraccioObjectTargetInterface targetter(nh);
    pose_goal_targetter::BraccioPoseGoal pose_targetter(nh);

    targetter.load_calibrate();

    ros::Rate rate(100);

    custom_msgs::matrix sherds;
    sherds = targetter.centers;

    try
    {
        for (size_t i = 0; i < sherds.size(); ++i)
        {
            auto chosen = sherds[i].center;
            Eigen::Vector3d chosen_vec(chosen[0], chosen[1], chosen[2]);
            chosen_vec /= 1000.0;
            std::cout << typeid(chosen_vec[0]).name() << std::endl;
            std::cout << chosen_vec.size() << std::endl;

            Eigen::Vector4d new_chos;
            new_chos << chosen_vec, 0.00;

            auto chosen_name = sherds[i].sherd;
            std::cout << chosen_name << std::endl;

            auto chosen_bowl = sherds[i].home;
            std::cout << chosen_bowl << std::endl;

            auto chosen_dimension = sherds[i].dimension;

            targetter.go_to_target("top", chosen_name, chosen_dimension);

            pose_targetter.create_tf(new_chos, -0.39, 0.0, 2.32);
            bool success = pose_targetter.go_to_pos(pose_targetter.arm_target_pose);
            targetter.go_to_j(0.0, 0.0, 0.0, 1.50);

            if (success)
            {
                targetter.transform_home(chosen_bowl, chosen_dimension);
            }
        }

        targetter.go_start_position();
    }
    catch (const std::exception& e)
    {
        targetter.go_start_position();
        std::cout << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
