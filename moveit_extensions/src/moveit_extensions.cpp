#include <moveit_extensions/moveit_extensions.h>

namespace moveit_extensions {
  double InterpolatedPropagationDistanceField::getInterpolatedDistance(double x, double y, double z) const {
    double rx = (x - origin_x_) / resolution_;
    double ry = (y - origin_y_) / resolution_;
    double rz = (z - origin_z_) / resolution_;
    // 小さい側のcell
    int ix = rx;
    int iy = ry;
    int iz = rz;
    if (ix < 0 || iy < 0 || iz < 0 || ix >= getXNumCells() - 1 || iy >= getYNumCells() - 1 || iz >= getZNumCells() - 1){
      return getDistance(x,y,z);
    }
    // 小さい側のcellからの比率
    rx -= ix;
    ry -= iy;
    rz -= iz;
    return getDistance(ix, iy, iz) * (1.0 - rx) * (1.0 - ry) * (1.0 - rz)
      + getDistance(ix + 1, iy, iz) * rx * (1.0 - ry) * (1.0 - rz)
      + getDistance(ix, iy + 1, iz) * (1.0 - rx) * ry * (1.0 - rz)
      + getDistance(ix + 1, iy + 1, iz) * rx * ry * (1.0 - rz)
      + getDistance(ix, iy, iz + 1) * (1.0 - rx) * (1.0 - ry) * rz
      + getDistance(ix + 1, iy, iz + 1) * rx * (1.0 - ry) * rz
      + getDistance(ix, iy + 1, iz + 1) * (1.0 - rx) * ry * rz
      + getDistance(ix + 1, iy + 1, iz + 1) * rx * ry * rz;
  }
};
