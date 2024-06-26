#include "include/HybridAStar.h"
#include "include/py_cpp_struct.h"

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

extern "C" {
    void ApplyHybridAStar(HybridAStarInitialConditions *hastar_ic,
        HybridAStarHyperparameters *hastar_hp, HybridAStarReturnValues *hastar_rv) {
        MapInfo *map_info = new MapInfo(hastar_ic, hastar_hp);
        HybridAStar hastar (map_info, hastar_hp);
        vector<Pose> path = hastar.runHybridAStar();

        // Reconstruct Hybrid AStar path
        int index = 0;
        for (auto p : path) {
            hastar_rv->x_path[index] = p[0];
            hastar_rv->y_path[index] = p[1];
            hastar_rv->yaw_path[index] = p[2];
            index += 1;
        }
        hastar_rv->x_path[index] = NAN;
        hastar_rv->y_path[index] = NAN;
        hastar_rv->yaw_path[index] = NAN;

        index = 0;
        for (int i=0; i<map_info->obstacle_path.size(); i++) {
            for (int j=0; j<map_info->obstacle_path[0].size(); j++) {
                hastar_rv->obstacle_Path[index++] = map_info->obstacle_path[i][j];
            }
        }
        hastar_rv->obstacle_Path[index] = NAN;

        // Only map_info contains pointers that need to be freed
        delete map_info;
        hastar_rv->success = !path.empty();
    }
}
