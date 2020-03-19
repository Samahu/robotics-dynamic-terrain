#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
    class DynamicTerrain : public VisualPlugin
    {
    public:
        DynamicTerrain() : VisualPlugin()
        {
        }

    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
        {
            // Make sure the ROS node for Gazebo has already been initialized                                                                                    
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            ROS_INFO("DynamicTerrain plugin successfully loaded!");
        }
    };

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrain)
}