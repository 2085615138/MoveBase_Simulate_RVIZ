/*********************************************************************
 *
 *这个类继承自Layer类和Costmap2D类，它是地图插件（静态层和障碍层）的基类。它的类方法主要用于处理bound和用几种不同的策略合并子地图和主地图
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_LAYER_H_
#define COSTMAP_2D_COSTMAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace costmap_2d
{

class CostmapLayer : public Layer, public Costmap2D
{
public:
  /*四个数据成员extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_在构造函数中被初始化为1e6和-1e6；*/
  CostmapLayer() : has_extra_bounds_(false),
    extra_min_x_(1e6), extra_max_x_(-1e6),
    extra_min_y_(1e6), extra_max_y_(-1e6) {}

  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area=false);

  /**
   * If an external source changes values in the costmap,
   * it should call this method with the area that it changed
   * to ensure that the costmap includes this region as well.
   * @param mx0 Minimum x value of the bounding box
   * @param my0 Minimum y value of the bounding box
   * @param mx1 Maximum x value of the bounding box
   * @param my1 Maximum y value of the bounding box
   */
  void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:
  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * TrueOverwrite means every value from this layer
   * is written into the master grid.
   */
  void updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Overwrite means every valid value from this layer
   * is written into the master grid (does not copy NO_INFORMATION)
   */
  void updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the maximum of the master_grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten. If the layer's value is NO_INFORMATION,
   * the master value does not change.
   */
  void updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the sum of the master grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten with the layer's value. If the layer's value
   * is NO_INFORMATION, then the master value does not change.
   *
   * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
   * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
   */
  void updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /**
   * Updates the bounding box specified in the parameters to include
   * the location (x,y)
   *
   * @param x x-coordinate to include
   * @param y y-coordinate to include
   * @param min_x bounding box
   * @param min_y bounding box
   * @param max_x bounding box
   * @param max_y bounding box
   */
  void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

  /*
   * Updates the bounding box specified in the parameters
   * to include the bounding box from the addExtraBounds
   * call. If addExtraBounds was not called, the method will do nothing.
   *
   * Should be called at the beginning of the updateBounds method
   *
   * @param min_x bounding box (input and output)
   * @param min_y bounding box (input and output)
   * @param max_x bounding box (input and output)
   * @param max_y bounding box (input and output)
   */
  void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);
  bool has_extra_bounds_;

private:
  double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_LAYER_H_
