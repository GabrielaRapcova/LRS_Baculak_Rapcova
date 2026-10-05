#include <gtest/gtest.h>
#include "astar_planner/search.hpp"
using namespace astar_planner;

uav_navigation_msgs::msg::VoxelCostmap map()
{
  uav_navigation_msgs::msg::VoxelCostmap result;
  result.header.frame_id="map"; result.resolution=1;
  Block b; b.occupancy.fill(0); result.blocks.push_back(b);
  return result;
}
void set(uav_navigation_msgs::msg::VoxelCostmap & m,int x,int y,int z,uint8_t value)
{m.blocks[0].occupancy[x+8*(y+8*z)]=value;}

TEST(Search, OpenSpaceAndExactEndpoints)
{
  Grid grid(map());
  auto result=search(grid,{0.6,0.7,0.8},{6.6,5.7,4.8},Limits{});
  ASSERT_EQ(result.status,SUCCESS);
  ASSERT_EQ(result.path.size(),2u);
  EXPECT_DOUBLE_EQ(result.path.front().x,0.6);
  EXPECT_DOUBLE_EQ(result.path.back().z,4.8);
  EXPECT_GT(result.raw_count,result.path.size());
}
TEST(Search, AltitudeChangingDetour)
{
  auto m=map();
  for (int y=0;y<8;++y) {for (int z=0;z<3;++z) {set(m,3,y,z,254);}}
  Grid grid(m);
  auto result=search(grid,{1.5,3.5,1.5},{5.5,3.5,1.5},Limits{});
  ASSERT_EQ(result.status,SUCCESS);
  EXPECT_TRUE(std::any_of(result.path.begin(),result.path.end(),[](Point p){return p.z>=3.5;}));
  for (size_t i=1;i<result.path.size();++i) {EXPECT_TRUE(visible(grid,result.path[i-1],result.path[i],10000));}
}
TEST(Search, CannotCutCornersOrVoxelPlanes)
{
  auto m=map(); set(m,2,1,1,253);
  Grid grid(m);
  EXPECT_FALSE(visible(grid,{1.5,1.5,1.5},{2.5,2.5,1.5},10000));
  EXPECT_FALSE(visible(grid,{2.0,1.2,1.5},{2.0,1.8,1.5},10000));
  EXPECT_FALSE(visible(grid,{1.5,1.5,1.5},{2.0,1.5,1.5},10000));
  EXPECT_FALSE(visible(grid,{2.0,1.5,1.5},{1.5,1.5,1.5},10000));
}
TEST(Search, NoPathUnknownAndInvalidCoordinates)
{
  auto m=map();
  for (int y=0;y<8;++y) {for (int z=0;z<8;++z) {set(m,3,y,z,254);}}
  Grid grid(m);
  EXPECT_EQ(search(grid,{1.5,3.5,1.5},{5.5,3.5,1.5},Limits{}).status,NO_PATH);
  EXPECT_EQ(search(grid,{-1,0,0},{5,3,1},Limits{}).status,INVALID_START);
  EXPECT_EQ(search(grid,{1.5,1.5,1.5},{INFINITY,0,0},Limits{}).status,INVALID_GOAL);
}
TEST(Search, LimitsCancellationAndRepeatedRequests)
{
  Grid grid(map()); Limits limits;
  limits.expansions=1;
  EXPECT_EQ(search(grid,{0.5,0.5,0.5},{7.5,7.5,7.5},limits).status,SEARCH_LIMIT);
  EXPECT_EQ(search(grid,{0.5,0.5,0.5},{7.5,7.5,7.5},Limits{},[](){return true;}).status,CANCELLED);
  for (int i=0;i<2;++i) {EXPECT_EQ(search(grid,{1.5,1.5,1.5},{1.5,1.5,1.5},Limits{}).status,SUCCESS);}
}
TEST(Search, NegativeBlockIndicesAndGeometry)
{
  auto m=map(); m.blocks[0].x=-1; m.origin.x=10; m.resolution=0.5;
  Grid grid(m);
  EXPECT_EQ(grid.cost({-1,0,0}),0);
  EXPECT_EQ(search(grid,{6.25,0.25,0.25},{9.75,2.25,2.25},Limits{}).status,SUCCESS);
}

TEST(Search, ThreeAxisCornerAndUnknownDiagonal)
{
  auto m=map(); set(m,2,1,1,254);
  Grid grid(m);
  EXPECT_FALSE(visible(grid,{1.5,1.5,1.5},{2.5,2.5,2.5},10000));
  m.blocks[0].occupancy.fill(255);
  set(m,1,1,1,0); set(m,2,2,2,0);
  EXPECT_EQ(search(Grid(m),{1.5,1.5,1.5},{2.5,2.5,2.5},Limits{}).status,NO_PATH);
}
TEST(Search, NodeTimeAndTraversalLimits)
{
  Grid grid(map()); Limits limits;
  limits.nodes=1;
  EXPECT_EQ(search(grid,{1.5,1.5,1.5},{6.5,6.5,6.5},limits).status,SEARCH_LIMIT);
  limits=Limits{}; limits.seconds=1e-12;
  EXPECT_EQ(search(grid,{1.5,1.5,1.5},{6.5,6.5,6.5},limits).status,SEARCH_LIMIT);
  EXPECT_THROW(visible(grid,{1.5,1.5,1.5},{6.5,6.5,6.5},1),Interrupted);
}
