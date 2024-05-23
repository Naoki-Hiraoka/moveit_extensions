#include <moveit_extensions/moveit_extensions.h>

namespace moveit_extensions {
  double InterpolatedPropagationDistanceField::getDistance(double x, double y, double z) const {
    double rx = (x - origin_x_) / resolution_;
    double ry = (y - origin_y_) / resolution_;
    double rz = (z - origin_z_) / resolution_;
    // 小さい側のcell. worldToGridを使うと近いcellになってしまう
    int ix = std::floor(rx);
    int iy = std::floor(ry);
    int iz = std::floor(rz);
    if (ix < 0 || iy < 0 || iz < 0 || ix >= getXNumCells() - 1 || iy >= getYNumCells() - 1 || iz >= getZNumCells() - 1){
      return PropagationDistanceField::getDistance(x,y,z);
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

  double InterpolatedPropagationDistanceField::getDistance(int x, int y, int z) const {
    return PropagationDistanceField::getDistance(x,y,z);
  }

  double InterpolatedPropagationDistanceField::getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y,
                                                                   double& gradient_z, bool& in_bounds) const
  {
    int gx, gy, gz;

    worldToGrid(x, y, z, gx, gy, gz);

    // if out of bounds, return max_distance, and 0 gradient
    // we need extra padding of 1 to get gradients
    if (gx < 1 || gy < 1 || gz < 1 || gx >= getXNumCells() - 1 || gy >= getYNumCells() - 1 || gz >= getZNumCells() - 1)
      {
        gradient_x = 0.0;
        gradient_y = 0.0;
        gradient_z = 0.0;
        in_bounds = false;
        return getUninitializedDistance();
      }

    gradient_x = (getDistance(gx + 1, gy, gz) - getDistance(gx - 1, gy, gz)) * inv_twice_resolution_;
    gradient_y = (getDistance(gx, gy + 1, gz) - getDistance(gx, gy - 1, gz)) * inv_twice_resolution_;
    gradient_z = (getDistance(gx, gy, gz + 1) - getDistance(gx, gy, gz - 1)) * inv_twice_resolution_;

    in_bounds = true;
    return getDistance(x, y, z); // ここだけDistanceField::getDistanceGradientと異なる
  }


};
