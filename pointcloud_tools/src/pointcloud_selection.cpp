#define BOOST_BIND_NO_PLACEHOLDERS 
#include <pcl/io/io.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/features/integral_image_normal.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <boost/graph/adjacency_matrix.hpp>
#include <queue>
#include <map>
#include <math.h>
#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_adjacency_container.h>
#include <pcl/octree/octree_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_pointcloud_adjacency.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <random>

using namespace std::chrono_literals;
using std::placeholders::_1;


bool compare_coordinates(const pcl::PointNormal point1, const pcl::PointNormal point2){
  return((point1.x == point2.x) && (point1.y == point2.y) && (point1.z == point2.z));
}


struct cmpPoints {
    bool operator()(const pcl::PointNormal a, const pcl::PointNormal b)
    const 
    {
        if (a.x < b.x) return true;
        if (a.x == b.x && a.y < b.y) return true;
        if (a.x == b.x && a.y == b.y && a.z < b.z) return true;
        return false;
    }
};



// A structure to hold the necessary parameters
struct node {
    pcl::PointNormal xyz;
    std::map<pcl::PointNormal, node, cmpPoints>::iterator parent_iterator;
    // f = g + h
    double f, g, h;
    bool closed = false;
    bool open = false;
    std::vector<pcl::PointNormal> neighbours;
    bool operator==(const node &nde) const {
      return compare_coordinates(nde.xyz, xyz);
    }
};

namespace pcl
{
  namespace octree
  {
    template <typename PointT,
          typename LeafContainerT = OctreePointCloudAdjacencyContainer<PointT>,
          typename BranchContainerT = OctreeContainerEmpty>
    class OctreePointCloudPathFinding : public OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>
    {
      public:
          OctreePointCloudPathFinding (const double resolution_arg) 
          : OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT> (resolution_arg)
          {

          }

          // BGL graph
          using OctreeBaseT = OctreeBase<LeafContainerT, BranchContainerT>;
          using VoxelAdjacencyList = boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, PointT, float>;
          using VoxelID = typename VoxelAdjacencyList::vertex_descriptor;
          using EdgeID = typename VoxelAdjacencyList::edge_descriptor;
          using OctreePointCloudT = OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeBaseT>;
          using LeafVectorT = std::vector<LeafContainerT *>;
          using NodeMap = std::map<pcl::PointNormal, node, cmpPoints>;

          //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          void computeVoxelAdjacencyGraph (VoxelAdjacencyList &voxel_adjacency_graph, NodeMap &m)
          {
            //TODO Change this to use leaf centers, not centroids!
            
            voxel_adjacency_graph.clear ();
            //Add a vertex for each voxel, store ids in map
            std::map <LeafContainerT*, VoxelID> leaf_vertex_id_map;
            for (auto leaf_itr = this->leaf_depth_begin () ; leaf_itr != this->leaf_depth_end (); ++leaf_itr)
            {
              OctreeKey leaf_key = leaf_itr.getCurrentOctreeKey ();
              PointT centroid_point;
              this->genLeafNodeCenterFromOctreeKey(leaf_key, centroid_point);
              VoxelID node_id = add_vertex (voxel_adjacency_graph);
              
              voxel_adjacency_graph[node_id] = centroid_point;
              LeafContainerT* leaf_container = &(leaf_itr.getLeafContainer ());
              leaf_vertex_id_map[leaf_container] = node_id;
            }
            //Iterate through and add edges to adjacency graph
            for (auto leaf_itr = this->begin(); leaf_itr != this->end(); ++leaf_itr)
            {
              VoxelID u = (leaf_vertex_id_map.find (*leaf_itr))->second;
              PointT p_u = voxel_adjacency_graph[u];
              // std::cout << p_u.x << std::endl;
              node graph_node;
              graph_node.xyz = p_u;
              for (auto neighbor_itr = (*leaf_itr)->cbegin (), neighbor_end = (*leaf_itr)->cend (); neighbor_itr != neighbor_end; ++neighbor_itr)
              {
                LeafContainerT* neighbor_container = *neighbor_itr;
                EdgeID edge;
                bool edge_added;
                VoxelID v = (leaf_vertex_id_map.find (neighbor_container))->second;
                boost::tie (edge, edge_added) = add_edge (u,v,voxel_adjacency_graph);
                PointT p_v = voxel_adjacency_graph[v];
                // if(!compare_coordinates(p_u,p_v)){
                //   //std::cout << "Iseenda naaber!" << std::endl;
                //   graph_node.neighbours.push_back(p_v);
                // }
                graph_node.neighbours.push_back(p_v);

                //float dist = (p_v.getVector3fMap () - p_u.getVector3fMap ()).norm ();
                //voxel_adjacency_graph[edge] = dist;
              }
              // std::cout << graph_node.neighbours.size() << std::endl;
              m.insert({p_u,graph_node});
                
            }
          }
  };
}
}

rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr status_publisher_;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr searchCloud (new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr searchResult (new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr object_pointcloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_subscription_;
std::vector<pcl::PointNormal> searchPointList;
pcl::octree::OctreePointCloudPathFinding<pcl::PointNormal> octree1 (0.0011f);
boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, pcl::PointNormal, float> adj_list;


//https://stackoverflow.com/questions/19467485/how-to-remove-element-not-at-top-from-priority-queue
template<typename T, class Container=std::vector<T>, class Compare=std::less<typename Container::value_type>> 
class custom_priority_queue : public std::priority_queue<T, Container, Compare>
{
  public:

      bool remove(const T& value) { //bool
        auto it = std::find(this->c.begin(), this->c.end(), value);
        if (it != this->c.end()) {
            this->c.erase(it);
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
            return true;
       }
       else {
        return false;
       }
 }
};



// this is an structure which implements the
// operator overloading
struct compare_lenght{
    bool operator()(node const& n1, node const& n2)
    {
        // return "true" if "n1" is ordered
        // before "n2", for example:
        return n1.f > n2.f;
    }
};


//To save the path to goal
std::vector<pcl::PointNormal> path_to_goal;
//Map with all nodes
std::map<pcl::PointNormal, node, cmpPoints> m;
//std::vector<node> all_nodes;
//Primary queue for nodes
custom_priority_queue<node, std::vector<node>, compare_lenght> open_set;

void publish_status(int status_number){
  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = status_number;
  status_publisher_->publish(msg);
}

void mapKeysToPointCloud(std::map<pcl::PointNormal, node, cmpPoints>& map, pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud){
  int i = 0;
  for(std::map<pcl::PointNormal, node, cmpPoints>::iterator it = map.begin(); it != map.end(); ++it) {
    (*pointCloud)[i].x = it->first.x;
    (*pointCloud)[i].y = it->first.y;
    (*pointCloud)[i].z = it->first.z;
    (*pointCloud)[i].normal_x = it->first.normal_x;
    (*pointCloud)[i].normal_y = it->first.normal_y;
    (*pointCloud)[i].normal_z = it->first.normal_z;
    ++i;
  }
}

void vectorToPointCloud(std::vector<pcl::PointNormal>& vector, pcl::PointCloud<pcl::PointNormal>::Ptr pointCloud){
  int i = 0;
  for(std::vector<pcl::PointNormal>::iterator it = vector.begin(); it != vector.end(); ++it) {
    (*pointCloud)[i].x = it->x;
    (*pointCloud)[i].y = it->y;
    (*pointCloud)[i].z = it->z;
    (*pointCloud)[i].normal_x = it->normal_x;
    (*pointCloud)[i].normal_y = it->normal_y;
    (*pointCloud)[i].normal_z = it->normal_z;
    ++i;
    //if(i == 100){break;}
  }
}

//Function to calculate distance
double distance(pcl::PointNormal point1, pcl::PointNormal point2){
    double d = sqrt(pow(point2.x - point1.x, 2) +
                pow(point2.y - point1.y, 2) +
                pow(point2.z - point1.z, 2) * 1.0);
    //std::cout << d << std::endl;
    return d;
}

void generatePath(node goal, node start){
  std::cout << "Generate path" << std::endl;

  node last_node = goal.parent_iterator->second;
  //   std::cout << "hhh" << std::endl;

  // node last_node2 = last_node.parent_iterator->second;
  //   std::cout << "hhh" << std::endl;

  // node last_node3 = last_node2.parent_iterator->second;
  //   std::cout << "hhh" << std::endl;

  // node last_node4 = last_node3.parent_iterator->second;
  // std::cout << "hhh" << std::endl;
  // std::cout << goal.xyz << last_node.xyz << last_node2.xyz << last_node3.xyz << last_node4.xyz << std::endl;
  path_to_goal.push_back(goal.xyz);
  while(!compare_coordinates(last_node.xyz,start.xyz)){
    //std::cout << last_node.xyz << std::endl;
    path_to_goal.push_back(last_node.xyz);
    last_node = last_node.parent_iterator->second;
  }
}

void search_path(std::map<pcl::PointNormal, node, cmpPoints>::iterator start, std::map<pcl::PointNormal, node, cmpPoints>::iterator goal)
{
  //set distance to starting point 0, because this is the starting point
  std::cout << "Beginning search!" << std::endl;
  start->second.g = 0;
  //set the pointer to the parent to itself, because this is the start
  start->second.parent_iterator = start;
  //calculate total distance cost
  start->second.f = start->second.g + distance(start->second.xyz, goal->second.xyz);
  //add to open_set to start exploring
  start->second.open = true;
  open_set.push(start->second);
  node current_node;
  //iterate as long as there are no more items in set
  try
  {  while(!open_set.empty()){
      //get the loweset cost
      current_node = open_set.top();
      open_set.pop();
      current_node.open = false;
      //path_to_goal.push_back(current_node.xyz);
      //std::cout << path_to_goal.size() << std::endl;
      //check if current node is the goal node
      if(compare_coordinates(current_node.xyz, goal->second.xyz)){
        std::cout << "Path found!" << std::endl;
        //std::cout << current_node.parent_iterator->second.xyz << std::endl;
        generatePath(current_node, start->second);
        break;
      }
      //mark node as explored/closed
      current_node.closed = true;
      //explore all neighbours
      for(auto neighbourPoint:current_node.neighbours){
        // std::cout << "neighbour" << std::endl;
        // std::cout << neighbourPoint << std::endl;
        std::map<pcl::PointNormal, node, cmpPoints>::iterator neigh_iterator = m.find(neighbourPoint);
        node neighbour = neigh_iterator->second;
        // std::cout << neighbour.xyz << std::endl;
        //check if neighbour is closed/explored
        if(neigh_iterator->second.closed == false){
          // std::cout << "not closed" << std::endl;
          //check if neighbour is in open_set
          //if not prepare for update
          if(neigh_iterator->second.open == false){
            // std::cout << "open" << std::endl;
            neigh_iterator->second.g = 100000; //TODO: change to infinity
            // neigh_iterator->second.parent_iterator = ;
          }
          //compare and update the values between nodes
          //update_vertex(current_node, neighbour, goal);
            // std::cout << "update vertex" << std::endl;
            double g_old = neigh_iterator->second.g;

          //compute cost
            if((current_node.g + distance(current_node.xyz, neigh_iterator->second.xyz)) < neigh_iterator->second.g){
              // std::cout << "if distance" << std::endl;
              neigh_iterator->second.parent_iterator = m.find(current_node.xyz);
              // std::cout <<current_node.g + distance(current_node.xyz, neighbour.xyz) <<std::endl;
              neigh_iterator->second.g = current_node.g + distance(current_node.xyz, neigh_iterator->second.xyz);
              // std::cout << "new " << neighbour.g << std::endl;
            }
            // std::cout << n2.g << std::endl;
            // compute_cost(n1, n2);
            // std::cout << "new " << n2.g << std::endl;

            // //std::cout << "old "<< g_old << std::endl;
            if(neigh_iterator->second.g < g_old){
            // std::cout << "L'ksin sisse" << std::endl;

              if(neigh_iterator->second.open){
                // std::cout << "remove" << std::endl;
                open_set.remove(neighbour);
              }
              //calculate total heuristic distance
              neigh_iterator->second.f = neigh_iterator->second.g + distance(neigh_iterator->second.xyz, goal->second.xyz);
              //add to open_set to start exploring
              // std::cout << "push" << std::endl;
              neigh_iterator->second.open = true;
              open_set.push(neigh_iterator->second);
            }
        }
      }
  }
        // No path found :(
      //std::cout << "No path found!" << std::endl;
  }
  catch (...) {
    std::cout << "ERROR!" << std::endl;;
  } 
  std::cout << "Search over!" << std::endl;
  generatePath(current_node, start->second);
}

void generate_normals_slow(pcl::PointCloud<pcl::PointNormal>::Ptr normals_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud){
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointNormal, pcl::Normal> ne;
  ne.setNumberOfThreads(8);
  ne.setInputCloud(point_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  std::cout<< "step1" <<std::endl;
  ne.setRadiusSearch(0.8);
  std::cout<< "step2" <<std::endl;

  // Compute the features
  ne.compute (*cloud_normals);
  std::cout<< "step3" <<std::endl;
  pcl::concatenateFields(*point_cloud, *cloud_normals, *normals_cloud);

  // cloud_normals->size () should have the same size as the input cloud->size ()*
}

void generate_normals(pcl::PointCloud<pcl::PointNormal>::Ptr normals_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud){
  // estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointNormal, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  pcl::concatenateFields(*point_cloud, *normals, *normals_cloud);

}

void generatePathFromPoints(){
  if(searchPointList.size() > 1){
    publish_status(5);
    pcl::octree::OctreePointCloudPathFinding<pcl::PointNormal> octree2 (0.0010f);//0.0031f
    m.clear();
    adj_list.clear();
    searchCloud->clear();
    octree2.setInputCloud(cloud);
    octree2.addPointsFromInputCloud();
    octree2.computeVoxelAdjacencyGraph(adj_list, m);
    searchCloud->width = cloud->size();
    searchCloud->height = 1;
    searchCloud->points.resize (cloud->width * cloud->height);
    mapKeysToPointCloud(m, searchCloud);
    std::cout << m.size() << std::endl;

    ////SEARCH FOR PATH
    auto searchite1 = m.begin();
    auto searchite2 = m.begin();//find_point

    double old_distance = 1000.0;

    //STUPID: TEE FUNKTSIOONIKS
    for ( auto it = m.begin(); it != m.end(); ++it  ){//auto elem : cloud_with_normals->points
      double new_distance = distance(searchPointList.front(), it->first);//*(searchPointList.end() - 1)
      if(new_distance<old_distance){
        searchite1 = it;
        old_distance = new_distance;
      }
    }
    std::cout << searchPointList.front() << std::endl;
    std::cout << searchite1->first << std::endl;

    //STUPID: TEE FUNKTSIOONIKS
    old_distance = 1000.0;
    for ( auto it = m.begin(); it != m.end(); ++it  ){//auto elem : cloud_with_normals->points
      double new_distance = distance(searchPointList.back(), it->first);
      if(new_distance<old_distance){
        searchite2 = it;
        old_distance = new_distance;
      }
    }
    std::cout << searchPointList.back() << std::endl;
    std::cout << searchite2->first << std::endl;

    std::cout << "Starting search." << std::endl;
    search_path(searchite2,searchite1);
    std::cout << "Search list is " << path_to_goal.size() << " elements." << std::endl;
    searchResult->width = path_to_goal.size();
    searchResult->height = 1;
    searchResult->points.resize (searchResult->width * searchResult->height);
    vectorToPointCloud(path_to_goal,searchResult);
    publish_status(6);
    path_to_goal.clear();
    searchPointList.clear();
  }
}

/*
Iterate over all elements of vector and print
them one by one, seperated by provided seperated.
*/
template <typename T>
void print_vector(const std::vector<T> & vec, std::string sep=" ")
{
    for(auto elem : vec)
    {
        std::cout<<"("<<elem.x<< "," << elem.y<< "," << elem.z<<")"<< sep;
    }
    std::cout<<std::endl;
}


class CommandSubscriber : public rclcpp::Node
{
  public:
    CommandSubscriber()
    : Node("panel_command_subscriber_pointcloud_selection")
    {
      status_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("software_status", 10);
      subscription_panel_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/panel_command", 10, std::bind(&CommandSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      switch (msg->buttons[0]){
        case 4:
          std::cout << "Generate random path!" << std::endl;
          generatePathFromPoints();
          break;
        default:
          break;
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_panel_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class PointSelection : public rclcpp::Node
{
  public:
    PointSelection()
    : Node("point_selection")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 10, std::bind(&PointSelection::topic_callback, this, _1));
    }
  private:
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "'%f' '%f' '%f'", msg->point.x,msg->point.y,msg->point.z);
        pcl::PointNormal searchPoint;
        searchPoint.x = msg->point.x;
        searchPoint.y = msg->point.y;
        searchPoint.z = msg->point.z;
        std::cout << searchPoint << std::endl;
        std::cout << "otsin punkti 2" << std::endl;
        searchPointList.push_back(searchPoint);
    }
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};


class PointCloud2Sub : public rclcpp::Node
{
  public:
    PointCloud2Sub()
    : Node("PointCloud2Sub")
    {
      pc2_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scanned_cloud", 10, std::bind(&PointCloud2Sub::pointCloud2Callback, this, _1));
    };
  private:
    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::fromROSMsg(*msg, *cloud);
    }
};

class searchCloudPublisher : public rclcpp::Node
{
  public:
    searchCloudPublisher()
    : Node("searchCloudPublisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("search_cloud", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&searchCloudPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      //std::cout << "points size "<< searchCloud->points.size() << std::endl;
      if(searchCloud->points.size()){
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*searchCloud, output);
        output.header.frame_id = "world";
        output.header.stamp = rclcpp::Clock().now();
        publisher_->publish(output);
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
};

class searchResultPublisher : public rclcpp::Node
{
  public:
    searchResultPublisher()
    : Node("searchResultPublisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("search_result", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&searchResultPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      if(searchResult->size()>0){
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*searchResult, output);
        output.header.frame_id = "world";
        output.header.stamp = rclcpp::Clock().now();
        publisher_->publish(output);
        //searchResult->clear();
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto node1 = std::make_shared<PointCloud2Sub>();
  auto node2 = std::make_shared<PointSelection>();
  auto node3 = std::make_shared<searchCloudPublisher>();
  auto node4 = std::make_shared<searchResultPublisher>();
  auto node5 = std::make_shared<CommandSubscriber>();
  exec.add_node(node1);
  exec.add_node(node2);
  exec.add_node(node3);
  exec.add_node(node4);
  exec.add_node(node5);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
