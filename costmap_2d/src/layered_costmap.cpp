/*********************************************************************
 * LayeredCostmap类是Costmap2DROS的成员，含有主地图，并能通过它操作各层子地图。这里关注一下两个被Costmap2DROS调用的函数。
 *********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (track_unknown)
    costmap_.setDefaultValue(NO_INFORMATION);
  else
    costmap_.setDefaultValue(FREE_SPACE);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

/*1. 地图尺寸设置 LayeredCostmap::resizeMap
这个函数在 Costmap2DROS 动态配置参数的回调函数 ReconfigureCB 中被调用，作用是在开启地图更新线程之前，调用 Costmap2D 的 resizeMap 函数，
用给定参数重新设置主地图的尺寸、原点、分辨率，再通过 plugin 指针调用各层地图的 matchSize ，使其以上参数和主地图匹配。
*/
void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  //调用costmap_的resizeMap方法
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  //然后根据plugin对每一层的地图调用其父类Costmap2D成员的initial方法，将plugin所指向的每一层地图的大小都设置为costmap_数据成员一样的空间大小
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
}

/*2. 地图更新 LayeredCostmap::updateMap
这个函数在Costmap2DROS的地图更新线程中被循环调用。它分为两步：第一步：更新bound，即确定地图更新的范围；
第二步：更新cost，更新每层地图cell对应的cost值后整合到主地图上。*/
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  /*rolling_window_默认为false，如果开启的话，地图是时刻跟随机器人中心移动的，这里需要根据机器人当前位置和地图大小计算出地图的新原点，设置给主地图。*/
  //如果我们使用窗口滚动，我们需要更新使用的机器人位置
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;
  /*接下来进行地图更新的第一步：更新bound
    设置好minx_、miny_、maxx_、maxy_的初始值，然后对每一层的子地图调用其updateBounds函数，传入minx_、miny_、maxx_、maxy_，函数将新的bound填充进去。
    updateBounds函数在Layer类中声明，在各层地图中被重载，第二步使用到的updateCosts函数也是如此。这两个函数的具体内容在各层地图部分详述。
  */
  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
  {
    if(!(*plugin)->isEnabled())
      continue;
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    //这个阶段会更新每个Layer的更新区域，这样在每个运行周期内减少了数据拷贝的操作时间。 
    //updateBounds 传入的是一个矩形范围
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }
  /*接下来调用 Costmap2D 类的 worldToMapEnforceBounds 函数，将得到的 bound 转换到地图坐标系。这个函数可以防止转换后的坐标超出地图范围。*/
  int x0, xn, y0, yn;
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
    return;
  /*接下来，调用resetMap，将主地图上bound范围内的cell的cost恢复为默认值（track_unknown：255 / 否则：0），再对每层子地图调用 updateCosts 函数。*/
  costmap_.resetMap(x0, y0, xn, yn);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if((*plugin)->isEnabled())
      (*plugin)->updateCosts(costmap_, x0, y0, xn, yn); //第二步，调用layer类的updatecosts
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    if((*plugin)->isEnabled())
      current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
