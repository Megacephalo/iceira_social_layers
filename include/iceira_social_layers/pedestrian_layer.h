#ifndef ICEIRA_PEDESTRIAN_LAYER_H_
#define ICEIRA_PEDESTRIAN_LAYER_H_

// ROS
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>


/// data
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedGroup.h>
#include <spencer_tracking_msgs/TrackedGroups.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <behavior_utils/entities.h>
#include <behavior_utils/geometry.h>
#include <behavior_utils/helpers.h>

//#include <behavior_functions/social_compliance_cost.h>
//#include <social_compliance_layer/SocialComplianceLayerConfig.h>

#include <pluginlib/class_list_macros.h>

// TODO: include libraries capable to update pedestrians poses
// TODO Integrate spencer tracking into the class
namespace iceira_pedestrian_layer {
  
using TPersons = spencer_tracking_msgs::TrackedPersons;
using TGroups = spencer_tracking_msgs::TrackedGroups;
//using LayerConfig = social_compliance_layer::SocialComplianceLayerConfig;

  class PedestrianLayer : public costmap_2d::Layer
  {
  public:
     PedestrianLayer();
     ~PedestrianLayer() ;
     
     // overriding costmap layer methods
     virtual void onInitialize(costmap_2d::Costmap2DROS *static_map,
                  unsigned int max_timesteps, ros::Duration time_resolution) ;
     
     // subscriber callbacks
    void callbackTrackedPersons(const TPersons::ConstPtr& msg);

    void callbackTrackedGroups(const TGroups::ConstPtr& msg);

    void callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void callbackGlobalPath(const nav_msgs::Path::ConstPtr& path);
    
    /**
     * @brief update the dynamic layers according to the most recent of observations in the environment. 
     * 
     * update is called by the planner right before a new plan is computed. The way the dynamic representation
     * is only updated when needed, not every time a new observation
     * comes in.
     */ 
    void layerUpdate() ;
  private:
    
    void computeLayerCostmap(const double& min_i,
          const double& min_j,
	  const double& max_i,
	  const double& max_j) ;
	  
    void updateRobotPose() ;
    
    /**
     * @brief publish the costmap layer that represents the requested time
     *        for visualization
     *
     * @param time requested time
     */
    void visualizeCostmapLayer(ros::Time time);    
    
    void calcGaussian(double pos_x,
            double pos_y,
	    double origin_x,
	    double origin_y,
	    double amplitude,
	    double variance_x,
	    double variance_y,
	    double skew) ;
    
    /**
     * @brief calculate the distance to the origin of the Gaussian from which the
     *        values are lower than the cutoff_value
     *
     * @param cutoff_value Gaussian value bound
     * @param amplitude amplitude of Gaussian
     * @param variance variance
     * @return cutoff distance from origin
     */
    double calcCutoffRadius(double cutoff_value, double amplitude, double variance);

      /**
   * @brief find the (unbounded) coordinates in the cost map that correspond to
   *        a given pose in an arbitrary reference frame
   *
   * @param pose requested pose
   * @param[out] cell_in_costmap_x corresponding map cell x coordinate
   * @param[out] cell_in_costmap_y corresponding map cell y coordinate
   * @param[out] angle_in_costmap corresponding angle in map frame
   * @return whether the transformation into map frame was successfull
   */
  bool getCostmapCoordinates(geometry_msgs::PoseStamped *pose, int &cell_in_costmap_x,
                             int &cell_in_costmap_y, double &angle_in_costmap);

  
  /**
   * FIXME: change the costmap format.
   * @brief update the relevant grid cells in the cost map according to the given
   *        pose of a person
   *
   * @param human_in_costmap_x map cell x coordinate of human
   * @param human_in_costmap_y map cell y coordinate of human
   * @param angle angle of human in map
   * @param costmap costmap to update
   */
  void markHumanInCostmap(int human_in_costmap_x, int human_in_costmap_y, double angle,
                          lattice_planner::TimedCostmap *costmap);

  
  
    ros::Subscriber sub_persons_;
    ros::Subscriber sub_groups_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_global_path_;
    
     
  } ; // class PedestrianLayer
} // namespace iceira_pedistrian_layer

#endif