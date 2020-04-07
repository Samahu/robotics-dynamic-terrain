#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "modify_terrain.h"

namespace gazebo
{
    class DynamicTerrainModel : public ModelPlugin
    {
        public: void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/)
        {
            gzlog << "DynamicTerrainModel: successfully loaded!" << std::endl;

            this->model_ = model;

            this->on_update_connection_ = event::Events::ConnectPostRender(
                std::bind(&DynamicTerrainModel::onUpdate, this));

            hole_drilled_ = false;
            timer_.Start();
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
            
            ModifyTerrain::modify(heightmap, position_xy, 0.003, 0.002, 1.0, "lower",
                [&heightmap_shape](int x, int y) { return heightmap_shape->GetHeight(x, heightmap_shape->VertexCount().Y() - y - 1); },
                [&heightmap_shape](int x, int y, float value) { heightmap_shape->SetHeight(x, heightmap_shape->VertexCount().Y() - y - 1, value); }
            );

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