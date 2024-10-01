/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <mbf_costmap_core/costmap_planner.h>
#include <mbf_msgs/GetPathResult.h>

using std::string;

#ifndef CABLE_GPP
#define CABLE_GPP

namespace cable_gpp
{
    class cable_gpp: public mbf_costmap_core::CostmapPlanner {
    public:
        cable_gpp();
        cable_gpp(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    }

    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);

}
#endif
