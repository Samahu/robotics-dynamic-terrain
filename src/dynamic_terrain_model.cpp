#include <gazebo/gazebo.hh>

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/rendering/Conversions.hh>

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class DynamicTerrainModel : public ModelPlugin
    {
        public: void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            // if (!ros::isInitialized())
            // {
            //     ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            //         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            //     return;
            // }

            gzlog << "DynamicTerrainModel: successfully loaded!" << std::endl;

            this->model_ = model;

            this->on_update_connection_ = event::Events::ConnectPostRender(
                std::bind(&DynamicTerrainModel::onUpdate, this));

            hole_drilled_ = false;
            timer_.Start();
        }

        void modifyTerrain(rendering::Heightmap* heightmap, physics::HeightmapShapePtr heightmap_shape,
            Ogre::Vector3 terrain_position, double outside_radius,
            double inside_radius, double weight, const std::string& op)
        {
            auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

            if (!terrain)
            {
                gzerr << "DynamicTerrainModel: Invalid heightmap" << std::endl;
                return;
            }

            auto size = static_cast<int>(terrain->getSize());
            gzlog << "DynamicTerrainModel: Terrain size " << size << std::endl;

            Ogre::Vector3 heightmap_position;
            terrain->getTerrainPosition(terrain_position, &heightmap_position);

            gzlog << "DynamicTerrainModel: terrain position  "
                << terrain_position.x << ", " << terrain_position.y << ", " << terrain_position.z << std::endl;
            gzlog << "DynamicTerrainModel: terrain position  "
                << heightmap_position.x << ", " << heightmap_position.y << ", " << heightmap_position.z << std::endl;

            auto left = std::max(int((heightmap_position.x - outside_radius) * size), 0);
            auto top = std::max(int((heightmap_position.y - outside_radius) * size), 0);
            auto right = std::min(int((heightmap_position.x + outside_radius) * size), size);
            auto bottom = std::min(int((heightmap_position.y + outside_radius) * size), size);

            gzlog << "DynamicTerrainModel: computed bounds "
                << left << ", " << top << ", " << right << ", " << bottom << std::endl;

            auto average_height = 0.0;

            if (op == "flatten" || op == "smooth")
                average_height = heightmap->AvgHeight(rendering::Conversions::ConvertIgn(heightmap_position), outside_radius);

            std::string original;
            std::string updated;

            for (auto y = top; y <= bottom; ++y)
            {
                for (auto x = left; x <= right; ++x)
                {
                    auto xx = x;
                    auto yy = heightmap_shape->VertexCount().Y() - y - 1;

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
                    float new_height = heightmap_shape->GetHeight(xx, yy);
        

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
                        gzerr << "Unknown terrain operation[" << op << "]" << std::endl;

                    // gzlog << "DynamicTerrainModel: p(" << (x-left) << ", " << (y-top) << "): "
                    //    << heightmap_shape->GetHeight(xx, yy) << " -> " << new_height << std::endl;
                    original += std::to_string(heightmap_shape->GetHeight(xx, yy));
                    original += " ";

                    updated += std::to_string(new_height);
                    updated += " ";

                    heightmap_shape->SetHeight(xx, yy, new_height);
                }

                original += "\n";
                updated += "\n";
            }

            gzlog << "DynamicTerrainModel: ORIGINAL\n" << original << std::endl;
            gzlog << "DynamicTerrainModel: UPDATED\n" << updated << std::endl;
        }

        rendering::Heightmap* getHeightmap()
        {
            auto scene = rendering::get_scene();
            if (!scene)
            {
                gzerr << "DynamicTerrainModel: Couldn't acquire scene!" << std::endl;
                return nullptr;
            }

            auto heightmap = scene->GetHeightmap();
            if (heightmap == nullptr)
            {
                gzerr << "DynamicTerrainModel: scene has no heightmap!" << std::endl;
                return nullptr;
            }

            auto terrain_group = heightmap->OgreTerrain();

            if (terrain_group == nullptr)
            {
                gzerr << "DynamicTerrainModel: terrain_group is null!" << std::endl;
                return nullptr;
            }

            return heightmap;
        }

        physics::HeightmapShapePtr getHeightmapShape()
        {
            if (model_ == nullptr)
            {
                gzerr << "DynamicTerrainModel plugin: Couldn't acquire heightmap model!" << std::endl;
                return nullptr;
            }

            auto collision = model_->GetLink("terrain-link")->GetCollision("collision");
            if (collision == nullptr)
            {
                gzerr << "DynamicTerrainModel plugin: Couldn't acquire heightmap model collision!" << std::endl;
                return nullptr;
            }
            
            auto shape = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
            if (shape == nullptr)
            {
                gzerr << "DynamicTerrainModel plugin: Couldn't acquire heightmap model collision!" << std::endl;
                return nullptr;
            }            

            gzlog << "DynamicTerrainModel plugin: heightmap shape ["
                << shape->VertexCount().X() << ", " << shape->VertexCount().Y() << "]" << std::endl;

            return shape;
        }

        void drillTerrainAt(double x, double y)
        {
            auto heightmap = getHeightmap();
            if (heightmap == nullptr)
            {
                gzerr << "DynamicTerrainModel: Couldn't acquire heightmap!" << std::endl;
                return;
            }

            auto heightmap_shape = getHeightmapShape();
            if (heightmap_shape == nullptr)
            {
                gzerr << "DynamicTerrainModel: Couldn't acquire heightmap shape!" << std::endl;
                return;
            }

            auto position_xy = Ogre::Vector3(x, y, 0);
            modifyTerrain(heightmap, heightmap_shape, position_xy, 0.003, 0.002, 1.0, "lower");
            hole_drilled_ = true;
            gzlog << "DynamicTerrainModel: A hole has been drilled at ("
                << position_xy.x << ", " << position_xy.y << ")" << std::endl;
        }

        void onUpdate()
        {
            // Only continue if a second has elapsed
            if (timer_.GetElapsed().Double() < 10.0 || hole_drilled_)
                return;

            drillTerrainAt(310.0, -275.0);
        }

    private:
        physics::ModelPtr model_;
        event::ConnectionPtr on_update_connection_;
        common::Timer timer_;
        bool hole_drilled_;
    };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DynamicTerrainModel)
}