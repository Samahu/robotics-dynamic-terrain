#include <gazebo/common/common.hh>
#include "modify_terrain.h"

using namespace gazebo;

class DynamicTerrainVisual : public VisualPlugin
{
public:
    void Load(rendering::VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/)
    {
        gzlog << "DynamicTerrainVisual: successfully loaded!" << std::endl;

        this->on_update_connection_ = event::Events::ConnectPostRender(
            std::bind(&DynamicTerrainVisual::onUpdate, this));

        hole_drilled_ = false;
        timer_.Start();
    }

private:

    rendering::Heightmap* getHeightmap()
    {
        auto scene = rendering::get_scene();
        if (!scene)
        {
            gzerr << "DynamicTerrainVisual: Couldn't acquire scene!" << std::endl;
            return nullptr;
        }

        auto heightmap = scene->GetHeightmap();
        if (heightmap == nullptr)
        {
            gzerr << "DynamicTerrainVisual: scene has no heightmap!" << std::endl;
            return nullptr;
        }

        return heightmap;
    }

    void drillTerrainAt(double x, double y)
    {
        auto heightmap = getHeightmap();
        if (heightmap == nullptr)
        {
            gzerr << "DynamicTerrainVisual: Couldn't acquire heightmap!" << std::endl;
            return;
        }

        auto position_xy = Ogre::Vector3(x, y, 0);
        
        auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
        ModifyTerrain::modify(heightmap, position_xy, 0.003, 0.002, 1.0, "lower",
            [&terrain](long x, long y) { return terrain->getHeightAtPoint(x, y); },
            [&terrain](long x, long y, float value) { terrain->setHeightAtPoint(x, y, value); }
        );

        terrain->updateGeometry();
        terrain->updateDerivedData(false,
            Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);

        hole_drilled_ = true;
        gzlog << "DynamicTerrainVisual: A hole has been drilled at ("
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
    event::ConnectionPtr on_update_connection_;
    common::Timer timer_;
    bool hole_drilled_;
};

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)