#ifndef ICEIRA_PEDESTRIAN_LAYER_H_
#define ICEIRA_PEDESTRIAN_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

// TODO: include libraries capable to update pedestrians poses
// TODO Integrate spencer tracking into the class
namespace iceira_pedestrian_layer {
  class PedestrianLayer : public costmap_2d::Layer
  {
  public:
     PedestrianLayer();
     
     virtual void onInitialize() ;

     virtual void updateBounds(double, double, double, double*, double*, double*, double*);
     virtual void updateCosts(costmap_2d::Costmap2D&, int, int, int, int);
  private:
     void reconfigureCB(costmap_2d::GenericPluginConfig&, uint32_t);
     //void getTargetEstimations(const PTrackingBridge::TargetEstimations::ConstPtr&);
     double mark_x_, mark_y_;
     dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;  
     ros::Subscriber sub_;
     //TargetsCostUpdater tcu;
     
  } ; // class PedestrianLayer
} // namespace iceira_pedistrian_layer

#endif