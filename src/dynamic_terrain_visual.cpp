#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include<gazebo/common/common.hh>
#include <OgreVector2.h>
#include <OgreVector3.h>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/rendering/Conversions.hh>

namespace gazebo
{
    class DynamicTerrainVisual : public VisualPlugin
    {
    public:
        void Load(rendering::VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            ROS_INFO("DynamicTerrainVisual: successfully loaded!");

            this->post_render_update_connection_ = event::Events::ConnectPostRender(
                std::bind(&DynamicTerrainVisual::onUpdate, this));

            hole_drilled_ = false;
            timer_.Start();
        }

    private:
        void modifyTerrain(rendering::Heightmap* heightmap, physics::HeightmapShapePtr heightmap_shape,
            Ogre::Vector3 terrain_position, double outside_radius,
            double inside_radius, double weight, const std::string& op)
        {
            auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

            if (!terrain)
            {
                ROS_ERROR("DynamicTerrainVisual: Invalid heightmap");
                return;
            }

            auto size = static_cast<int>(terrain->getSize());
            ROS_INFO_STREAM("DynamicTerrainVisual: Terrain size " << size);

            Ogre::Vector3 heightmap_position;
            ROS_INFO_STREAM("DynamicTerrainVisual: terrain position  " << terrain_position.x << ", " << terrain_position.y << ", " << terrain_position.z);
            terrain->getTerrainPosition(terrain_position, &heightmap_position);
            ROS_INFO_STREAM("DynamicTerrainVisual: terrain position  " << heightmap_position.x << ", " << heightmap_position.y << ", " << heightmap_position.z);

            auto left = std::max(int((heightmap_position.x - outside_radius) * size), 0);
            auto top = std::max((int(heightmap_position.y - outside_radius) * size), 0);
            auto right = std::min(int((heightmap_position.x + outside_radius) * size), size);
            auto bottom = std::min(int((heightmap_position.y + outside_radius) * size), size);

            ROS_INFO_STREAM("DynamicTerrainVisual: computed bounds " << left << ", " << top << ", " << right << ", " << bottom);
            auto average_height = 0.0;

            if (op == "flatten" || op == "smooth")
                average_height = heightmap->AvgHeight(rendering::Conversions::ConvertIgn(heightmap_position), outside_radius);

            for (auto y = top; y <= bottom; ++y)
                for (auto x = left; x <= right; ++x)
                {
                    auto ts_x_dist = (x / static_cast<double>(size)) - heightmap_position.x;
                    auto ts_y_dist = (y / static_cast<double>(size))  - heightmap_position.y;
                    auto dist = sqrt(ts_y_dist * ts_y_dist + ts_x_dist * ts_x_dist);

                    auto inner_weight = 1.0;
                    if (dist > inside_radius)
                    {
                        inner_weight = ignition::math::clamp(dist / outside_radius, 0.0, 1.0);
                        inner_weight = 1.0 - (inner_weight * inner_weight);
                    }

                    float added_height = inner_weight * weight;
                    float new_height = terrain->getHeightAtPoint(x, y);
        

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

                    terrain->setHeightAtPoint(x, y, new_height);
                }

            terrain->dirty();
            terrain->update();
        }

        rendering::Heightmap* getHeightmap()
        {
            auto scene = rendering::get_scene();
            if (!scene)
            {
                ROS_ERROR("DynamicTerrainVisual: Couldn't acquire scene!");
                return nullptr;
            }

            auto heightmap = scene->GetHeightmap();
            if (heightmap == nullptr)
            {
                ROS_ERROR("DynamicTerrainVisual: scene has no heightmap!");
                return nullptr;
            }

            auto terrain_group = heightmap->OgreTerrain();

            if (terrain_group == nullptr)
            {
                ROS_ERROR("DynamicTerrainVisual: terrain_group is null!");
                return nullptr;
            }

            return heightmap;
        }

        void drillTerrainAt(double x, double y)
        {
            auto heightmap = getHeightmap();
            if (heightmap == nullptr)
            {
                ROS_ERROR("DynamicTerrainVisual: Couldn't acquire heightmap!");
                return;
            }

            /*
            auto heightmap_shape = getHeightmapShape();
            if (heightmap_shape == nullptr)
            {
                ROS_ERROR("DynamicTerrainVisual: Couldn't acquire heightmap shape!");
                return;
            }
            */

            auto position_xy = Ogre::Vector3(x, y, 0);
            modifyTerrain(heightmap, nullptr, position_xy, 0.003, 0.002, 1.0, "lower");
            hole_drilled_ = true;
            ROS_INFO_STREAM("DynamicTerrainVisual: A hole has been drilled at (" << position_xy.x << ", " << position_xy.y << ")");
        }

        void onUpdate()
        {
            // Only continue if a second has elapsed
            if (timer_.GetElapsed().Double() < 10.0 || hole_drilled_)
                return;

            drillTerrainAt(310.0, -275.0);
        }

    private:
        event::ConnectionPtr post_render_update_connection_;
        common::Timer timer_;
        bool hole_drilled_;
    };

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)
}