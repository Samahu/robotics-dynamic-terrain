#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector2.hh>

namespace gazebo
{
    class DynamicTerrainModel : public ModelPlugin
    {
        public: void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            ROS_INFO("DynamicTerrainModel plugin: successfully loaded!");

            this->model_ = model;
            this->post_world_update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DynamicTerrainModel::OnUpdate, this));

            hole_drilled_ = false;
            timer_.Start();
        }

        void modifyTerrain(physics::HeightmapShapePtr heightmap_shape,
            ignition::math::Vector2d heightmap_position,
            double outside_radius, double inside_radius,
            double weight, const std::string& op)
        {
            auto size = static_cast<int>(heightmap_shape->VertexCount().X());
            ROS_INFO_STREAM("DynamicTerrainModel: Terrain size " << size);

            auto left = std::max(int((heightmap_position.X() - outside_radius)), 0);
            auto top = std::max((int(heightmap_position.Y() - outside_radius)), 0);
            auto right = std::min(int((heightmap_position.X() + outside_radius)), size);
            auto bottom = std::min(int((heightmap_position.Y() + outside_radius)), size);

            auto average_height = 0.0;

            ROS_INFO_STREAM("DynamicTerrainModel: bounds " << left << ", " << top << ", " << right << ", " << bottom);

            /*
            if (op == "flatten" || op == "smooth")
                average_height = heightmap->AvgHeight(rendering::Conversions::ConvertIgn(heightmap_position), outside_radius);
            */

            for (auto y = 0; y <= size; ++y)
                for (auto x = 0; x <= size; ++x)
                {
                    auto ts_x_dist = (x / static_cast<double>(size)) - heightmap_position.X();
                    auto ts_y_dist = (y / static_cast<double>(size))  - heightmap_position.Y();
                    auto dist = sqrt(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

                    auto inner_weight = 1.0;
                    if (dist > inside_radius)
                    {
                        inner_weight = ignition::math::clamp(dist / outside_radius, 0.0, 1.0);
                        inner_weight = 1.0 - (inner_weight * inner_weight);
                    }

                    float added_height = inner_weight * weight;
                    float new_height = heightmap_shape->GetHeight(x, y);

                    if (op == "raise")
                        new_height += added_height;
                    else if (op == "lower")
                        new_height -= added_height;
                    else if (op == "flatten")
                    {
                        if (new_height < average_height)
                            new_height += added_height;
                        else
                            new_height -= added_height;
                    }
                    else if (op == "smooth")
                    {
                        if (new_height < average_height)
                            new_height += added_height;
                        else
                            new_height -= added_height;
                    }
                    else
                        ROS_ERROR_STREAM("Unknown terrain operation[" << op << "]");

                    heightmap_shape->SetHeight(x, y, heightmap_shape->GetHeight(x, y) - 1);
                }
        }

        physics::HeightmapShapePtr getHeightmapShape()
        {
            if (model_ == nullptr)
            {
                ROS_ERROR("DynamicTerrainModel plugin: Couldn't acquire heightmap model!");
                return nullptr;
            }

            auto collision = model_->GetLink("terrain-link")->GetCollision("collision");
            if (collision == nullptr)
            {
                ROS_ERROR("DynamicTerrainModel plugin: Couldn't acquire heightmap model collision!");
                return nullptr;
            }
            
            auto shape = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
            if (shape == nullptr)
            {
                ROS_ERROR("DynamicTerrainModel plugin: Couldn't acquire heightmap model collision!");
                return nullptr;
            }            

            ROS_INFO_STREAM("DynamicTerrainModel plugin: heightmap shape ["
                << shape->VertexCount().X() << ", " << shape->VertexCount().Y() << "]");

            return shape;
        }

        void OnUpdate()
        {
            // Only continue if a second has elapsed
            if (timer_.GetElapsed().Double() < 10.0 || hole_drilled_)
                return;

            auto drill_location = ignition::math::Vector2d(300, -275);
            auto heightmapShape = getHeightmapShape();
            if (heightmapShape == nullptr)
            {
                ROS_INFO("DynamicTerrainModel: Couldn't acquire heightmap shape!");
                return;
            }

            modifyTerrain(heightmapShape, drill_location, 10, 5, 2.0, "lower");
            hole_drilled_ = true;
            ROS_INFO_STREAM("DynamicTerrainModel: A hole has been drilled at (" << drill_location.X() << ", " << drill_location.Y() << ")");

        }

    private:
        physics::ModelPtr model_;
        event::ConnectionPtr post_world_update_connection_;
        common::Timer timer_;
        bool hole_drilled_;
    };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DynamicTerrainModel)
}