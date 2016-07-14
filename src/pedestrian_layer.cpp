#include "iceira_social_layers/pedestrian_layer.h"
#include "../include/iceira_social_layers/pedestrian_layer.h"


using costmap_2d::LETHAL_OBSTACLE ;

namespace iceira_pedestrian_layer
{
  PedestrianLayer::PedestrianLayer() {} 
  
  void PedestrianLayer::onInitialize()
  {
  }

  void PedestrianLayer::reconfigureCB(costmap_2d::GenericPluginConfig&, uint32_t)
  {
  }

  void PedestrianLayer::updateBounds(double, double, double, double*, double*, double*, double*)
  {
  }

  void PedestrianLayer::updateCosts(costmap_2d::Costmap2D&, int, int, int, int)
  {
  }






}  // namepsace iceira_pedestrian_layer









# include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iceira_pedestrian_layer::PedestrianLayer, costmap_2d::Layer)