// The M1 test area: open hardpack to ride around on, with a few features to try the bike on. (The proper demo map
// with checkpoints comes in M5.) Spawn at the origin facing +Z.
#pragma once
#include "terrain.h"

namespace tws {

struct TestAreaInfo {
    Vec3 spawn{0, 0, 0};
    double spawn_heading_deg = 0;
};

TestAreaInfo buildTestArea(Terrain& t);

}  // namespace tws
