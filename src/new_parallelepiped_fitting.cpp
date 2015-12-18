
// Author(s): Luca Gemma

#include <string>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <ios>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/String.h"

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>
#include <pcl/io/ply_io.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>



#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/impl/angles.hpp>





#include <pcl/features/don.h>



#include "marker_generator.h"
#include "visual_perception/TabletopSegmentation.h"
#include "visual_perception/ParallelepipedFitting.h"
#include "visual_perception/Plane.h"
#include "visual_perception/Parallelepiped.h"
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


// pcl_viewer
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/time.h>

// PCL conditional euclidean clustering
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/point_types_conversion.h>

// New Pcl Difference of Normals Example for PCL Segmentation 
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

namespace visual_perception {

class ParallelepipedFitter
{ //typedef pcl::PointXYZ    Point;
  typedef pcl::PointXYZRGB    Point;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
  typedef pcl::PointNormal    PointN;

  //typedef pcl::PointXYZI PointTypeIO;
  //typedef pcl::PointXYZINormal PointTypeFull;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher parallelepipedfitting_pub_ ;
  //! Service server for object detection
  ros::ServiceServer fitting_parallelepiped_srv_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
   //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;
  bool flatten_plane_;
   //! How much the table gets padded in the horizontal direction
  double plane_padding_;

  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;

  //! A tf transform listener
  tf::TransformListener listener_;

    //! A tf transform broadcaster
  tf::TransformBroadcaster broadcaster_;

   



  //------------------ Callbacks -------------------

  
  //! Callback for service calls
 bool serviceCallback(ParallelepipedFitting::Request &request, ParallelepipedFitting::Response &response);
 
  
//------------------- Complete processing -----
// //! Publishes rviz markers for the given tabletop clusters

 //! Converts table convex hull into a triangle mesh to add to a Table messagetemplate <class PointCloudType>
 


 //template <class PointCloudType>
 //Plane ParallelepipedFitter::getPlane(std_msgs::Header cloud_header,const tf::Transform &table_plane_trans, const PointCloudType &table_points);



// //   template <class PointCloudType>
// //   void publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header);
//    //! Pull out and transform the convex hull points from a Table message
//   template <class PointCloudType>
//   bool tableMsgToPointCloud (Plane &plane, std::string frame_id, PointCloudType &plane_hull);
  

 public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
 ParallelepipedFitter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;
    parallelepipedfitting_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    fitting_parallelepiped_srv_ = nh_.advertiseService(nh_.resolveName("parallelepiped_fitting_srv"), &ParallelepipedFitter::serviceCallback, this);
    

   //initialize operational flags
   priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
   priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
   priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.005);// 0.003
   priv_nh_.param<double>("up_direction", up_direction_, -1.0);
    priv_nh_.param<bool>("flatten_plane", flatten_plane_, false);

  }


  //! Empty stub
  ~ParallelepipedFitter() {}
};



/*template <class PointCloudType>
void CylinderFitting::publishClusterMarkers(const std::vector<PointCloudType> &clusters, std_msgs::Header cloud_header)
{
  for (size_t i=0; i<clusters.size(); i++) 
  {
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    cloud_marker.header = cloud_header;
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    marker_pub_.publish(cloud_marker);
  }
}*/



/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction, bool flatten_plane)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  tf::Vector3 position(-a*d, -b*d, -c*d);
  tf::Vector3 z(a, b, c);

  //if we are flattening the plane, make z just be (0,0,up_direction)
  if(flatten_plane)
  {
    ROS_INFO("flattening plane");
    z[0] = z[1] = 0;
    z[2] = up_direction;
  }
  else
  {
    //make sure z points "up"
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
    {
      z = -1.0 * z;
      ROS_INFO("flipped z");
    }
  }
    
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  tf::Vector3 x(1, 0, 0);
  tf::Vector3 y1(0, 1, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
  if (fabs(z.dot(y1)) > 1.0 - 1.0e-4) x = tf::Vector3(1,0,0);
  tf::Vector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  tf::Matrix3x3 rotation;
  rotation[0] = x;  // x
  rotation[1] = y;  // y
  rotation[2] = z;  // z
  rotation = rotation.transpose();
  tf::Quaternion orientation;
  rotation.getRotation(orientation);
  ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
  ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  return tf::Transform(orientation, position);
}

template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
         const tf::Transform& table_plane_trans,
         sensor_msgs::PointCloud &table_points,std::string plane_name)
{
  // Prepare the output
  table_points.header = pcl_conversions::fromPCL(table.header);
  table_points.points.resize (table.points.size ());
  table_points.header.frame_id=plane_name.c_str();
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  // ros::Time table_stamp;
  // ros::Time now = ros::Time::now();
  // table_stamp.fromNSec(table.header.stamp*1000);
  // ROS_INFO("ROS TIME NOW %lu", now.toNSec()/1000);
  // ROS_INFO("TABLE HEADER STAMP %lu", table_stamp.toNSec()/1000);
  // table_stamp = pcl_conversions::fromPCL(table.header.stamp);
  tf::StampedTransform table_pose_frame(table_plane_trans,ros::Time::now(), "head_asus/camera_rgb_optical_frame", plane_name.c_str());
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform(plane_name.c_str(), "head_asus/camera_rgb_optical_frame", ros::Time(0), &error_msg))//table_points.header.stamp
  {
    ROS_ERROR("Can not transform point cloud from frame %s to %s; error %s", 
        "head_asus/camera_rgb_optical_frame", plane_name.c_str(),error_msg.c_str());
    return false;
  }
  else
    {ROS_INFO("Can transform point cloud from frame %s to %s ","head_asus/camera_rgb_optical_frame",plane_name.c_str()); }
  int current_try = 0, max_tries = 5;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformPointCloud(plane_name.c_str(), table_points, table_points);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into plane_frame; error %s", 
                  "head_asus/camera_rgb_optical_frame", ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  table_points.header.stamp = ros::Time::now();
  table_points.header.frame_id = plane_name.c_str();
  return true;
}




tf::Transform getParallelepipedTransform (pcl::ModelCoefficients coeffs)
{
  // ROS_INFO("Size of coeffs: %d", (int)coeffs.values.size());
  // ROS_ASSERT(coeffs.values.size() > 8);
  double x = coeffs.values[0], y = coeffs.values[1], z = coeffs.values[2], ax = coeffs.values[3], ay = coeffs.values[4], az = coeffs.values[5];
  // r = coeffs.values[6]; the radius is not used

  // The position is directly obtained from the coefficients, and will be corrected later
  tf::Vector3 position(x,y,z);
  
  // w is the axis of the cylinder which will be aligned with the z reference frame of the cylinder
  tf::Vector3 w(ax, ay, az);
  tf::Vector3 u(1, 0, 0);
  if ( fabs(w.dot(u)) > 1.0 - 1.0e-4) u = tf::Vector3(0, 1, 0);
  tf::Vector3 v = w.cross(u).normalized();
  u = v.cross(w).normalized();
  tf::Matrix3x3 rotation;
  rotation[0] = u;  // x
  rotation[1] = v;  // y
  rotation[2] = w;  // z
  rotation = rotation.transpose();
  tf::Quaternion orientation;
  rotation.getRotation(orientation);

  // Compose the transformation and return it
  return tf::Transform(orientation, position);
}



template <typename PointT> void
getClustersFromPointCloud2 (const pcl::PointCloud<PointT> &cloud_objects,           
          const std::vector<pcl::PointIndices> &clusters2, 
          std::vector<sensor_msgs::PointCloud> &clusters)
{
  clusters.resize (clusters2.size ());
  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    pcl::PointCloud<PointT> cloud_cluster;
    pcl::copyPointCloud(cloud_objects, clusters2[i], cloud_cluster);
   sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg( cloud_cluster, pc2 ); 
    sensor_msgs::convertPointCloud2ToPointCloud (pc2, clusters[i]);  
  }
}



bool ParallelepipedFitter::serviceCallback(ParallelepipedFitting::Request &request, ParallelepipedFitting::Response &response)
{  
  float height_1,width_1,depth_1,height_2,width_2,depth_2,height_3,width_3,depth_3;
  tf::Quaternion quaternion_1,quaternion_2,quaternion_3;
  tf::Vector3 origin_1,origin_2,origin_3;
  float I_1,I_2,I_3;
  int n_object;
  n_object=request.n_object;
  Table table;
  table=request.table;
  ROS_INFO("Parallelepiped Fitting start !!!");
  ROS_INFO("Table:%f,%f,%f,%f,%f,%f",table.x_min,table.x_max,table.y_min,table.y_max,table.z_min,table.z_max);
  pcl::PointCloud<PointN>::Ptr cloud_cluster_rem_cluster_1 (new pcl::PointCloud<PointN>);
  pcl::PointCloud<PointN>::Ptr cloud_cluster_rem_cluster_2 (new pcl::PointCloud<PointN>);
  pcl::PointCloud<PointN>::Ptr cloud_cluster_rem_cluster_3 (new pcl::PointCloud<PointN>);
  pcl::PointCloud<Point>::Ptr cloud_1 (new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr cloud_2 (new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr cloud_3 (new pcl::PointCloud<Point>);
  Eigen::VectorXf pl_coeff_1;
  Eigen::VectorXf pl_coeff_2;
  Eigen::VectorXf pl_coeff_3;
  ros::Time start_time = ros::Time::now();
  
  std::vector<sensor_msgs::PointCloud> cluster;
  cluster=request.cluster;
  

 sensor_msgs::PointCloud2 converted_cloud;
 sensor_msgs::convertPointCloudToPointCloud2 (cluster[n_object], converted_cloud);
 ROS_INFO("Step 1 conversion from ros2pcl DONE !!!");
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
 pcl::fromROSMsg(converted_cloud, *cloud_ptr);
 // sensor_msgs::PointCloud2 cluster ;
  // sensor_msgs::convertPointCloud2ToPointCloud (cluster,request.cluster);
  // ROS_INFO("Step 1 conversion from ros2pcl DONE !!!");
  // pcl::PointCloud<Point>::Ptr cloud_ptr (new pcl::PointCloud<Point>); 
  // pcl::fromROSMsg(cluster, *cloud_ptr);
  ROS_INFO("Step 2 cloud_ptr DONE !!!");
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
  pcl::PointCloud<Point>::Ptr plane_hull_1_ptr (new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr plane_hull_2_ptr (new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr plane_hull_3_ptr (new pcl::PointCloud<Point>);


// Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
// pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);



  ROS_INFO("Init Done!");
 
  ROS_INFO("Downsample the points"); 
  // ---[ Downsample the points
  pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>); 
  grid_objects_.setInputCloud (cloud_ptr);
  grid_objects_.filter (*cloud_objects_downsampled_ptr);
  ROS_INFO("Downsample the points DONE !!!"); 
  // Step 6: Split the objects into Euclidean clusters
  ROS_INFO ("WAYTING FOR: Split the objects into Euclidean clusters");
  std::vector<pcl::PointIndices> clusters2;
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

  // ---[ Convert clusters into the PointCloud message
  std::vector<sensor_msgs::PointCloud> clusters_c;
  getClustersFromPointCloud2<Point> (*cloud_objects_downsampled_ptr, clusters2, clusters_c);
  //getClustersFromPointCloud2<Point> (*cloud_ptr, clusters2, clusters_c);
  ROS_INFO("Clusters converted");


  // FINAL, pusblish the markers
  //publishClusterMarkers(clusters_c, cloud.header);
    
  if ((int)clusters2.size () == 0) 
    
    {
      ROS_INFO("NO object on the table");
      response.result = response.NO_PARALLELEPIPED;

      //return;
    }
  else
  { 
    tf::Transform base_2_camera;
    tf::StampedTransform base_2_camera_stamped;
            
    ros::Time now2 = ros::Time::now();
    listener_.waitForTransform("vito_anchor", "head_asus/camera_rgb_optical_frame", now2, ros::Duration(4));
    listener_.lookupTransform("vito_anchor", "head_asus/camera_rgb_optical_frame", now2, base_2_camera_stamped);
            
    base_2_camera.setOrigin(base_2_camera_stamped.getOrigin());
    base_2_camera.setBasis(base_2_camera_stamped.getBasis());


    ROS_INFO("Difference of normals segmentation: start !!");
    ///The smallest scale to use in the DoN filter.
    double scale1=0.01;

    ///The largest scale to use in the DoN filter.
    double scale2=0.02;

    ///The minimum DoN magnitude to threshold by
    double threshold=0.06; //0.05

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius=0.009;

    //cloud_objects_downsampled_ptr 

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<Point>::Ptr tree;
    if (cloud_objects_downsampled_ptr->isOrganized ())
    {
      tree.reset (new pcl::search::OrganizedNeighbor<Point> ());
    }
    else
    {
     tree.reset (new pcl::search::KdTree<Point> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud_objects_downsampled_ptr);

    if (scale1 >= scale2)
    {
     ROS_ERROR("Error: Large scale must be > small scale!");
     response.result = response.OTHER_ERROR;
    }
    else
    {
      // Compute normals using both small and large scales at each point
      pcl::NormalEstimationOMP<Point,PointN> ne;
      ne.setInputCloud(cloud_objects_downsampled_ptr);
      ne.setSearchMethod(tree);
      /**
      * NOTE: setting viewpoint is very important, so that we can ensure
      * normals are all pointed in the same direction!
      */
      ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

      // calculate normals with the small scale
      ROS_INFO("Calculating normals for scale: %f",(float)scale1);
      pcl::PointCloud<PointN>::Ptr normals_small_scale (new pcl::PointCloud<PointN>);

      ne.setRadiusSearch (scale1);
      ne.compute (*normals_small_scale);

      // calculate normals with the large scale
      ROS_INFO("Calculating normals for scale: %f",(float)scale2);
      pcl::PointCloud<PointN>::Ptr normals_large_scale (new pcl::PointCloud<PointN>);

      ne.setRadiusSearch (scale2);
      ne.compute (*normals_large_scale);

      // Create output cloud for DoN results
      pcl::PointCloud<PointN>::Ptr doncloud (new pcl::PointCloud<PointN>);
      pcl::copyPointCloud<Point, PointN>(*cloud_objects_downsampled_ptr, *doncloud);

      ROS_INFO("Calculating DoN");
      // Create DoN operator
      pcl::DifferenceOfNormalsEstimation<Point, PointN, PointN> don;
      don.setInputCloud (cloud_objects_downsampled_ptr);
      don.setNormalScaleLarge (normals_large_scale);
      don.setNormalScaleSmall (normals_small_scale);

      if (!don.initCompute ())
        {
          ROS_ERROR("Error: Could not intialize DoN feature operator");
          response.result = response.OTHER_ERROR;
        }

      // Compute DoN
      don.computeFeature (*doncloud);

      // Save DoN features
      // pcl::PCDWriter writer;
      // writer.write<pcl::PointN> ("don.pcd", *doncloud, false); 

      // Filter by magnitude
      ROS_INFO("Filtering out DoN mag <= %f",(float)threshold );

      // Build the condition for filtering
      pcl::ConditionOr<PointN>::Ptr range_cond (new pcl::ConditionOr<PointN> ());
      range_cond->addComparison (pcl::FieldComparison<PointN>::ConstPtr (new pcl::FieldComparison<PointN> ("curvature", pcl::ComparisonOps::GT, threshold)));
      
      // Build the filter
      pcl::ConditionalRemoval<PointN> condrem (range_cond,true);
      condrem.setInputCloud (doncloud);

      pcl::PointCloud<PointN>::Ptr doncloud_filtered (new pcl::PointCloud<PointN>);

      // Apply filter
      condrem.filter (*doncloud_filtered);
      pcl::PointIndices cluster_indices_rem;
      condrem.getRemovedIndices(cluster_indices_rem);

      //doncloud = doncloud_filtered;

      // Save filtered output
      ROS_INFO("Filtered Pointcloud data points %f: ",(float) doncloud->points.size ());

      //  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

        // Filter by magnitude
      ROS_INFO("Clustering using EuclideanClusterExtraction with tolerance <= %f", (float)segradius);

      pcl::search::KdTree<PointN>::Ptr segtree (new pcl::search::KdTree<PointN>);
      segtree->setInputCloud (doncloud);


      // pcl::PCDWriter writer;
      
        
          pcl::PointCloud<PointN>::Ptr cloud_cluster_rem (new pcl::PointCloud<PointN>);
          //for (std::vector<int>::const_iterator pit = cluster_indices_rem.begin (); pit != cluster_indices_rem.end (); ++pit)
          for(int i=0;i<cluster_indices_rem.indices.size();++i)
          {
            cloud_cluster_rem->push_back(doncloud->points[cluster_indices_rem.indices[i]]);
          }
          

          cloud_cluster_rem->width = int (cloud_cluster_rem->points.size ());
          cloud_cluster_rem->height = 1;
          cloud_cluster_rem->is_dense = true;

          //Save cluster
           ROS_INFO("PointCloud representing the Cluster: %d data points.", (int)cloud_cluster_rem->points.size());
           std::stringstream ss,ss2;
           ss << "/home/pacman/Projects/pcd_file/rem_cluster.pcd";
           ss2 << "/home/pacman/Projects/pcd_file/don_cluster.pcd";
            pcl::PCDWriter writer;
           writer.write<PointN> (ss.str (), *cloud_cluster_rem, false);
           writer.write<PointN> (ss2.str (), *doncloud_filtered, false);

           std::vector<pcl::PointIndices> cluster_indices;
           pcl::EuclideanClusterExtraction<PointN> ec;

          doncloud = cloud_cluster_rem;
          ec.setClusterTolerance (segradius);
          ec.setMinClusterSize (100);
          ec.setMaxClusterSize (10000);
          ec.setSearchMethod (segtree);
          ec.setInputCloud (doncloud);
          ec.extract (cluster_indices);
             
              int j = 0;



        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
        {
          pcl::PointCloud<PointN>::Ptr cloud_cluster_rem_cluster (new pcl::PointCloud<PointN>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          {
            cloud_cluster_rem_cluster->points.push_back (doncloud->points[*pit]);
            if(j==0){
              cloud_cluster_rem_cluster_1->points.push_back (doncloud->points[*pit]);
              cloud_cluster_rem_cluster_1->width = int (cloud_cluster_rem_cluster_1->points.size ());
              cloud_cluster_rem_cluster_1->height = 1;
              cloud_cluster_rem_cluster_1->is_dense = true;
            
              
              //ROS_INFO("cloud_cluster_rem_cluster_1 Loaded");
            }
            if(j==1){
              cloud_cluster_rem_cluster_2->points.push_back (doncloud->points[*pit]);
              cloud_cluster_rem_cluster_2->width = int (cloud_cluster_rem_cluster_2->points.size ());
              cloud_cluster_rem_cluster_2->height = 1;
              cloud_cluster_rem_cluster_2->is_dense = true;
            
              
              //ROS_INFO("cloud_cluster_rem_cluster_2 Loaded");
            }
            if(j==2){
              cloud_cluster_rem_cluster_3->points.push_back (doncloud->points[*pit]);
              cloud_cluster_rem_cluster_3->width = int (cloud_cluster_rem_cluster_3->points.size ());
              cloud_cluster_rem_cluster_3->height = 1;
              cloud_cluster_rem_cluster_3->is_dense = true;
           
              
              //ROS_INFO("cloud_cluster_rem_cluster_3 Loaded");

            }  
          }

          cloud_cluster_rem_cluster->width = int (cloud_cluster_rem_cluster->points.size ());
          cloud_cluster_rem_cluster->height = 1;
          cloud_cluster_rem_cluster->is_dense = true;

          //Save cluster
           ROS_INFO("PointCloud representing the Cluster: %d data points.", (int)cloud_cluster_rem_cluster->points.size());
           std::stringstream ss;
           ss << "/home/pacman/Projects/pcd_file/cloud_cluster_rem_cluster_" << j << ".pcd";
           writer.write<PointN> (ss.str (), *cloud_cluster_rem_cluster, false); 
         }
         pcl::copyPointCloud(*cloud_cluster_rem_cluster_1,*cloud_1);
         pcl::copyPointCloud(*cloud_cluster_rem_cluster_2,*cloud_2);
         pcl::copyPointCloud(*cloud_cluster_rem_cluster_3,*cloud_3);
         ROS_INFO("Fit Parallelepiped to each cluster in clusters2 IIIFFF the cluster is not empty");
         //  //Step 7: Fit Parallelepiped to each cluster in clusters2 IIIFFF the cluster is not empty
         //Additional PCL objects
         KdTreePtr normals_cluster_tree_;
         pcl::NormalEstimation<Point, pcl::Normal> n3d_cluster_1,n3d_cluster_2,n3d_cluster_3;  
         n3d_cluster_1.setKSearch (10);  
         n3d_cluster_1.setSearchMethod (normals_cluster_tree_);
         n3d_cluster_2.setKSearch (10);  
         n3d_cluster_2.setSearchMethod (normals_cluster_tree_);
         n3d_cluster_3.setKSearch (10);  
         n3d_cluster_3.setSearchMethod (normals_cluster_tree_);
         
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
         /////////// PLANE 1 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         
         if(j>=0){

         pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1_ptr (new pcl::PointCloud<pcl::Normal>); 
         n3d_cluster_1.setInputCloud(cloud_1);
         n3d_cluster_1.compute(*cloud_normals_1_ptr); 
         //pcl::SampleConsensusModelNormalParallelPlane<Point,PointN>::Ptr plane_model1_ (new pcl::SampleConsensusModelNormalParallelPlane<Point,PointN> (cloud_1,cloud_normals_1_ptr));
         pcl::SampleConsensusModelParallelPlane<Point>::Ptr plane_model1_ (new pcl::SampleConsensusModelParallelPlane<Point> (cloud_1));
         plane_model1_->setAxis(Eigen::Vector3f (1.0, 0.0, 0.0));
         plane_model1_->setEpsAngle(pcl::deg2rad (45.0));
         ROS_INFO("WAYTING FOR FITTING  plane 1!!!"); 
         pcl::RandomSampleConsensus<Point> ransac1 (plane_model1_,1);
         ransac1.computeModel();
         
         ransac1.getModelCoefficients (pl_coeff_1);
         if (pl_coeff_1.size () <=3) {
         ROS_INFO("Failed to fit a plane to the cluster");
         }
         else
         {
         ROS_INFO(" Now, try to do a better fit, get the inliers for plane 1");
         std::vector<int> plane_inliers_1;
         ransac1.getInliers(plane_inliers_1);

         ROS_INFO("Give the previous coeff as an initial guess and optimize to fit to the inliers");
         Eigen::VectorXf plane_coeff_1_optimized;
         plane_model1_->optimizeModelCoefficients(plane_inliers_1, pl_coeff_1, plane_coeff_1_optimized);

         ROS_INFO ("[ObjectFitter::input_callback] Plane Model coefficients optimized are: [%f %f %f %f].", plane_coeff_1_optimized[0], plane_coeff_1_optimized[1], plane_coeff_1_optimized[2], plane_coeff_1_optimized[3]);
         ROS_INFO("Step 3 Plane_1 done");

         //Project the inliers into the cylinder model to obtain the cylinder height and the center of the cylinder.
         // Hard code to convert from Eigen::VecctorXf to pcl::ModelCoefficients and pcl::PointIndices  
         pcl::PointIndices::Ptr plane_inliers_1_ptr (new pcl::PointIndices); 
         plane_inliers_1_ptr->indices.resize((int)plane_inliers_1.size());
               
         pcl::ModelCoefficients::Ptr plane_1_coefficients_ptr (new pcl::ModelCoefficients);
         plane_1_coefficients_ptr->values.resize(plane_coeff_1_optimized.size());
            
         for (int i = 0; i < plane_coeff_1_optimized.size(); i++)
         {   
           plane_1_coefficients_ptr->values[i] = plane_coeff_1_optimized[i];
         }
                
         for (int i = 0; i < (int)plane_inliers_1.size(); i++)
          {
            plane_inliers_1_ptr->indices[i] = plane_inliers_1[i];
                 
          }
         // Step 4 : Project the table inliers on the table
         pcl::PointCloud<Point>::Ptr plane_projected_1_ptr (new pcl::PointCloud<Point>); 
         plane_projected_1_ptr->header.frame_id="head_asus/camera_rgb_optical_frame";
         proj_.setInputCloud (cloud_1);
         proj_.setIndices(plane_inliers_1_ptr);
         proj_.setModelCoefficients(plane_1_coefficients_ptr);
         proj_.filter(*plane_projected_1_ptr);

         //          // Get Plane transform 
         tf::Transform plane_1_trans;
         tf::Transform plane_1_trans_flat;
         sensor_msgs::PointCloud plane_1_points;
         sensor_msgs::PointCloud plane_hull_1_points;


         plane_1_trans = getPlaneTransform(*plane_1_coefficients_ptr,up_direction_,false);
         hull_.setInputCloud(plane_projected_1_ptr);
         hull_.reconstruct(*plane_hull_1_ptr);
         plane_1_trans_flat = getPlaneTransform(*plane_1_coefficients_ptr, up_direction_, true);
         tf::Vector3 new_plane_pos_1;
         double avg_x_1, avg_y_1, avg_z_1;
         avg_x_1 = avg_y_1 = avg_z_1 = 0;
         for (size_t i=0; i<plane_projected_1_ptr->points.size(); i++)
         {
           avg_x_1 += plane_projected_1_ptr->points[i].x;
           avg_y_1 += plane_projected_1_ptr->points[i].y;
           avg_z_1 += plane_projected_1_ptr->points[i].z;
         }
         avg_x_1 /= plane_projected_1_ptr->points.size();
         avg_y_1 /= plane_projected_1_ptr->points.size();
         avg_z_1 /= plane_projected_1_ptr->points.size();
         ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x_1, avg_y_1, avg_z_1);

         // place the new plane frame in the center of the convex hull
         new_plane_pos_1[0] = avg_x_1;
         new_plane_pos_1[1] = avg_y_1;
         new_plane_pos_1[2] = avg_z_1;
         plane_1_trans.setOrigin(new_plane_pos_1);


         // shift the non-flat plane frame to the center of the convex hull as well
         plane_1_trans_flat.setOrigin(new_plane_pos_1);  
                
         ROS_INFO ("[PlaneFitter::input_callback] Success in computing the plane 1 transformation with 7 elements (pos, quat).");
         origin_1 = plane_1_trans.getOrigin();
         quaternion_1 = plane_1_trans.getRotation();
         ROS_INFO("Center of the plane at [%f %f %f]", origin_1[0], origin_1[1], origin_1[2]);
         


         //tf::StampedTransform paralle_2_twist_stamped;
                
         //listener_.waitForTransform("plane_1", "left_arm_7_link", now, ros::Duration(4));
         //listener_.lookupTransform("plane_1", "left_arm_7_link", now, paralle_2_twist_stamped);

         tf::Transform plane_1_in_base;
         //plane_1_in_base.frame_id="vito_anchor";
         
         plane_1_in_base = base_2_camera*plane_1_trans;
     
         sensor_msgs::PointCloud plane_points_1;
         plane_points_1.header.frame_id="plane_1";
         ROS_INFO("Trasform Calculated for plane 1");
         ros::Time now = ros::Time::now();
         // broadcaster_.sendTransform(tf::StampedTransform(plane_1_trans_flat, ros::Time::now(), "head_asus/camera_rgb_optical_frame", "plane_1_trans_flat"));
         broadcaster_.sendTransform(tf::StampedTransform(plane_1_trans, now, "head_asus/camera_rgb_optical_frame", "plane_1"));
          ROS_INFO("Trasform Sended");
         if (getPlanePoints<Point>(*plane_projected_1_ptr, plane_1_trans, plane_points_1,"plane_1"))
         {
           Plane plane_1;
           //get the extents of the table
           if (!plane_1_points.points.empty()) 
           {
            plane_1.x_min = plane_points_1.points[0].x;
            plane_1.x_max = plane_points_1.points[0].x;
            plane_1.y_min = plane_points_1.points[0].y;
            plane_1.y_max = plane_points_1.points[0].y;
            plane_1.z_min = plane_points_1.points[0].z;
            plane_1.z_max = plane_points_1.points[0].z;
           }  
           for (size_t i=1; i<plane_points_1.points.size(); ++i) 
           {
            if (plane_points_1.points[i].x<plane_1.x_min && plane_points_1.points[i].x>-0.21) plane_1.x_min = plane_points_1.points[i].x;
            if (plane_points_1.points[i].x>plane_1.x_max && plane_points_1.points[i].x< 0.21) plane_1.x_max = plane_points_1.points[i].x;
            if (plane_points_1.points[i].y<plane_1.y_min && plane_points_1.points[i].y>-0.09) plane_1.y_min = plane_points_1.points[i].y;
            if (plane_points_1.points[i].y>plane_1.y_max && plane_points_1.points[i].y< 0.09) plane_1.y_max = plane_points_1.points[i].y;
            if (plane_points_1.points[i].z<plane_1.z_min && plane_points_1.points[i].z>-0.01) plane_1.z_min = plane_points_1.points[i].z;
            if (plane_points_1.points[i].z>plane_1.z_max && plane_points_1.points[i].z< 0.01) plane_1.z_max = plane_points_1.points[i].z;
           }
      

           ROS_INFO ("[PlaneFitter::input_callback] Success for plane_1 in computing plane.x_min,plane.x_max,plane.y_min,plane.y_max,plane.z_min,plane.z_max: %f,%f,%f,%f,%f,%f.",plane_1.x_min,plane_1.x_max,plane_1.y_min,plane_1.y_max,plane_1.z_min,plane_1.z_max);
 
           geometry_msgs::Pose plane_pose_1,plane_pose_1_in_base;
           // pusblish the cylinder in the camera frame
           tf::poseTFToMsg(plane_1_trans, plane_pose_1);
           // pusblish the cylinder in the robot frame
           tf::poseTFToMsg(plane_1_in_base, plane_pose_1_in_base);
           plane_1.pose.pose = plane_pose_1_in_base;
 
           // plane_1.x_min=plane_1.x_min;
           // plane_1.x_max= plane_1.x_max;
           // plane_1.y_min=plane_1.y_min;
           // plane_1.y_max=plane_1.y_max;
           // plane_1.z_min=plane_1.z_min;
           // plane_1.z_max=plane_1.z_max;

           // tf::Vector3 point_A(plane_1.x_max,plane_1.y_min,plane_1.z_max);
           // tf::Vector3 point_B(plane_1.x_max,plane_1.y_max,plane_1.z_max);
           // tf::Vector3 point_C(plane_1.x_min,plane_1.y_max,plane_1.z_max);
           
            height_1 = sqrt((plane_1.x_max-plane_1.x_min)*(plane_1.x_max-plane_1.x_min)+(plane_1.y_max-plane_1.y_max)*(plane_1.y_max-plane_1.y_max)+(plane_1.z_max-plane_1.z_max)*(plane_1.z_max-plane_1.z_max));

            width_1  = sqrt((plane_1.x_max-plane_1.x_max)*(plane_1.x_max-plane_1.x_max)+(plane_1.y_max-plane_1.y_min)*(plane_1.y_max-plane_1.y_min)+(plane_1.z_max-plane_1.z_max)*(plane_1.z_max-plane_1.z_max));
 
            depth_1  = sqrt((plane_1.x_max-plane_1.x_max)*(plane_1.x_max-plane_1.x_max)+(plane_1.y_min-plane_1.y_min)*(plane_1.y_min-plane_1.y_min)+(plane_1.z_max-plane_1.z_min)*(plane_1.z_max-plane_1.z_min));
           
           //height_1 = sqrt((plane_1.x_max-plane_1.x_min)*(plane_1.x_max-plane_1.x_min));

           //width_1  = sqrt((plane_1.y_max-plane_1.y_min)*(plane_1.y_max-plane_1.y_min));
 
//           depth_1  = sqrt((plane_1.z_max-plane_1.z_min)*(plane_1.z_max-plane_1.z_min));
           ROS_INFO("Height_1: %f ",float(height_1));
           ROS_INFO("Width_1: %f ",float(width_1));
           ROS_INFO("Depth_1: %f ",float(depth_1));


           plane_1.height=height_1;
           plane_1.width=width_1;
           plane_1.depth=depth_1;

           ROS_INFO ("[ParallelepipedFitter::input_callback] Success for plane 1in computing height,width,depth: %f,%f,%f.",height_1,width_1,depth_1);
           if(height_1<1.0 & width_1<1.0 & depth_1<1.0)
           {
            response.plane_1 = plane_1;

            //Parallelepiped parallelepiped;

             uint32_t cube_shape = visualization_msgs::Marker::CUBE;
             visualization_msgs::Marker cube_marker;
             // Set the marker type to arrow
             cube_marker.type = cube_shape;

             // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
             cube_marker.header.frame_id = "/head_asus/camera_rgb_optical_frame";
             //cube_marker.header.frame_id = "/vito_anchor";
             //cube_marker.header.frame_id = "/plane_1";

             cube_marker.header.stamp = ros::Time::now();

             // // Set the namespace and id for this marker.  This serves to create a unique ID
             // // Any marker sent with the same namespace and id will overwrite the old one
             cube_marker.ns = "parallelepiped_fitting_node";
             cube_marker.id = n_object+1;

             // // Set the marker action.  Options are ADD and DELETE
             cube_marker.action = visualization_msgs::Marker::ADD;

             //tf::Vector3 origin_1 = plane_1_in_base.getOrigin();
             //tf::Quaternion quaternion_1 = plane_1_in_base.getRotation();
               
             cube_marker.pose.position.x = origin_1[0];
             cube_marker.pose.position.y = origin_1[1];
             cube_marker.pose.position.z = origin_1[2];
             cube_marker.pose.orientation.x = quaternion_1[0];
             cube_marker.pose.orientation.y = quaternion_1[1];
             cube_marker.pose.orientation.z = quaternion_1[2];
             cube_marker.pose.orientation.w = quaternion_1[3];
                
             //Controll plane fitting
             int n_inliers_1;
             n_inliers_1=plane_model1_->countWithinDistance(plane_coeff_1_optimized,(double)0.025);
             ROS_INFO("n_inliers del fitting %d",n_inliers_1);
             ROS_INFO("(int)cloud_ptr->size() %d",(int)cloud_1->size());

          
             //I= (float)plane_inliers_1.size()/(float)cloud_ptr->size();
             I_1= (float)n_inliers_1/(float)cloud_1->size();
             ROS_INFO("Indice di qualità del fitting %f",I_1);
           
             // // Set the scale of the marker 
             cube_marker.scale.x = fabs(width_1);
             cube_marker.scale.y = fabs(height_1); 
             cube_marker.scale.z = fabs(depth_1);

             // // Set the color -- be sure to set alpha to something non-zero!
             cube_marker.color.r = 1.0f-(double)I_1;;
             cube_marker.color.g = (double)I_1;
             cube_marker.color.b = 0.0f;
             cube_marker.color.a = 1.0;
             if((double)I_1>0.45)
             { 
               response.result=response.SUCCESS;
               cube_marker.lifetime = ros::Duration(10);
               ROS_INFO("FINAL, publish the markers");
                // // Publish the marker
               parallelepipedfitting_pub_.publish(cube_marker);
             }
             else
             {
               cube_marker.lifetime = ros::Duration(0.5); 
               ROS_INFO("FINAL, publish the markers");
               // // Publish the marker
               parallelepipedfitting_pub_.publish(cube_marker);
             }          
            }
         }  

        }

      

      }
         


         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
         /////////// PLANE 2 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         
         if(j>=1){
         pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2_ptr (new pcl::PointCloud<pcl::Normal>); 
         n3d_cluster_2.setInputCloud(cloud_2);
         n3d_cluster_2.compute(*cloud_normals_2_ptr); 
         //pcl::SampleConsensusModelNormalParallelPlane<Point,PointN>::Ptr plane_model2_ (new pcl::SampleConsensusModelNormalParallelPlane<Point,PointN> (cloud_2));
         pcl::SampleConsensusModelParallelPlane<Point>::Ptr plane_model2_ (new pcl::SampleConsensusModelParallelPlane<Point> (cloud_2));
         //pcl::SampleConsensusModelPlane<Point>::Ptr plane_model2_ (new pcl::SampleConsensusModelPlane<Point> (cloud_2));
         
         plane_model2_->setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
         plane_model2_->setEpsAngle (pcl::deg2rad (45.0));
         ROS_INFO("WAYTING FOR FITTING plane 2 !!!"); 
         pcl::RandomSampleConsensus<Point> ransac2 (plane_model2_,1);
         ransac2.computeModel();
         ransac2.getModelCoefficients (pl_coeff_2);
         if (pl_coeff_2.size () <=3) {
         ROS_INFO("Failed to fit a plane to the cluster");
         }
         else
         {
          ROS_INFO(" Now, try to do a better fit, get the inliers for plane 2");
          std::vector<int> plane_inliers_2;
          ransac2.getInliers(plane_inliers_2);

          ROS_INFO("Give the previous coeff as an initial guess and optimize to fit to the inliers");
          Eigen::VectorXf plane_coeff_2_optimized;
          plane_model2_-> optimizeModelCoefficients(plane_inliers_2, pl_coeff_2, plane_coeff_2_optimized);

          ROS_INFO ("[ObjectFitter::input_callback] Plane Model coefficients optimized are: [%f %f %f %f].", plane_coeff_2_optimized[0], plane_coeff_2_optimized[1], plane_coeff_2_optimized[2], plane_coeff_2_optimized[3]);
          ROS_INFO("Step 3 Plane_2 done");

          //Project the inliers into the cylinder model to obtain the cylinder height and the center of the cylinder.
          // Hard code to convert from Eigen::VecctorXf to pcl::ModelCoefficients and pcl::PointIndices  
          pcl::PointIndices::Ptr plane_inliers_2_ptr (new pcl::PointIndices); 
          plane_inliers_2_ptr->indices.resize((int)plane_inliers_2.size());
                
          pcl::ModelCoefficients::Ptr plane_2_coefficients_ptr (new pcl::ModelCoefficients);
          plane_2_coefficients_ptr->values.resize(plane_coeff_2_optimized.size());
             
          for (int i = 0; i < plane_coeff_2_optimized.size(); i++)
          {   
            plane_2_coefficients_ptr->values[i] = plane_coeff_2_optimized[i];
          }
                 
          for (int i = 0; i < (int)plane_inliers_2.size(); i++)
           {
             plane_inliers_2_ptr->indices[i] = plane_inliers_2[i];
                  
           }
          // Step 4 : Project the table inliers on the table
          pcl::PointCloud<Point>::Ptr plane_projected_2_ptr (new pcl::PointCloud<Point>); 
          proj_.setInputCloud(cloud_2);
          proj_.setIndices(plane_inliers_2_ptr);
          proj_.setModelCoefficients (plane_2_coefficients_ptr);
          proj_.filter (*plane_projected_2_ptr);

          //          // Get Plane transform 
          tf::Transform plane_2_trans;
          tf::Transform plane_2_trans_flat;
          sensor_msgs::PointCloud plane_2_points;
          sensor_msgs::PointCloud plane_hull_2_points;


          plane_2_trans = getPlaneTransform(*plane_2_coefficients_ptr,up_direction_,false);
          hull_.setInputCloud (plane_projected_2_ptr);
          hull_.reconstruct (*plane_hull_2_ptr);
          plane_2_trans_flat = getPlaneTransform (*plane_2_coefficients_ptr, up_direction_, true);
          tf::Vector3 new_plane_pos_2;
          double avg_x_2, avg_y_2, avg_z_2;
          avg_x_2 = avg_y_2 = avg_z_2 = 0;
          for (size_t i=0; i<plane_projected_2_ptr->points.size(); i++)
          {
            avg_x_2 += plane_projected_2_ptr->points[i].x;
            avg_y_2 += plane_projected_2_ptr->points[i].y;
            avg_z_2 += plane_projected_2_ptr->points[i].z;
          }
          avg_x_2 /= plane_projected_2_ptr->points.size();
          avg_y_2 /= plane_projected_2_ptr->points.size();
          avg_z_2 /= plane_projected_2_ptr->points.size();
          ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x_2, avg_y_2, avg_z_2);

          // place the new plane frame in the center of the convex hull
          new_plane_pos_2[0] = avg_x_2;
          new_plane_pos_2[1] = avg_y_2;
          new_plane_pos_2[2] = avg_z_2;
          plane_2_trans.setOrigin(new_plane_pos_2);


          // shift the non-flat plane frame to the center of the convex hull as well
          plane_2_trans_flat.setOrigin(new_plane_pos_2); 

          ROS_INFO ("[PlaneFitter::input_callback] Success in computing the plane 2 transformation with 7 elements (pos, quat).");
         origin_2 = plane_2_trans.getOrigin();
         quaternion_2 = plane_2_trans.getRotation();
         ROS_INFO("Center of the plane at [%f %f %f]", origin_2[0], origin_2[1], origin_2[2]);
         
         //tf::StampedTransform paralle_2_twist_stamped;
                
         //listener_.waitForTransform("plane_1", "left_arm_7_link", now, ros::Duration(4));
         //listener_.lookupTransform("plane_1", "left_arm_7_link", now, paralle_2_twist_stamped);

         tf::Transform plane_2_in_base;

         plane_2_in_base = base_2_camera*plane_2_trans;

         sensor_msgs::PointCloud plane_points_2;
         plane_points_2.header.frame_id="plane_2";

         ros::Time now = ros::Time::now();
         // broadcaster_.sendTransform(tf::StampedTransform(plane_1_trans_flat, ros::Time::now(), "head_asus/camera_rgb_optical_frame", "plane_1_trans_flat"));
         broadcaster_.sendTransform(tf::StampedTransform(plane_2_trans, now, "head_asus/camera_rgb_optical_frame", "plane_2"));
         ROS_INFO("Trasform Sended");
         
         ROS_INFO("Trasform in vito_anchor frame of reference calculated");
         if (getPlanePoints<Point> (*plane_projected_2_ptr, plane_2_trans, plane_points_2,"plane_2"))
         {
           Plane plane_2;
           //get the extents of the table
           if (!plane_points_2.points.empty()) 
           {
            plane_2.x_min = plane_points_2.points[0].x;
            plane_2.x_max = plane_points_2.points[0].x;
            plane_2.y_min = plane_points_2.points[0].y;
            plane_2.y_max = plane_points_2.points[0].y;
            plane_2.z_min = plane_points_2.points[0].z;
            plane_2.z_max = plane_points_2.points[0].z;
           }  
           for (size_t i=1; i<plane_points_2.points.size(); ++i) 
           {
            if (plane_points_2.points[i].x<plane_2.x_min && plane_points_2.points[i].x>-0.5) plane_2.x_min = plane_points_2.points[i].x;
            if (plane_points_2.points[i].x>plane_2.x_max && plane_points_2.points[i].x< 0.5) plane_2.x_max = plane_points_2.points[i].x;
            if (plane_points_2.points[i].y<plane_2.y_min && plane_points_2.points[i].y>-0.5) plane_2.y_min = plane_points_2.points[i].y;
            if (plane_points_2.points[i].y>plane_2.y_max && plane_points_2.points[i].y< 0.5) plane_2.y_max = plane_points_2.points[i].y;
            if (plane_points_2.points[i].z<plane_2.z_min && plane_points_2.points[i].z>-0.5) plane_2.z_min = plane_points_2.points[i].z;
            if (plane_points_2.points[i].z>plane_2.z_max && plane_points_2.points[i].z< 0.5) plane_2.z_max = plane_points_2.points[i].z;
           }
                

           ROS_INFO ("[PlaneFitter::input_callback] Success in computing plane.x_min,plane.x_max,plane.y_min,plane.y_max,plane.z_min,plane.z_max: %f,%f,%f,%f,%f,%f.",plane_2.x_min,plane_2.x_max,plane_2.y_min,plane_2.y_max,plane_2.z_min,plane_2.z_max);

           geometry_msgs::Pose plane_pose_2;
           // pusblish the cylinder in the camera frame
           tf::poseTFToMsg(plane_2_trans, plane_pose_2);
           // pusblish the cylinder in the robot frame
           //tf::poseTFToMsg(plane_1_in_base, plane_pose);
           plane_2.pose.pose = plane_pose_2;

           plane_2.x_min=plane_2.x_min;
           plane_2.x_max= plane_2.x_max;
           plane_2.y_min=plane_2.y_min;
           plane_2.y_max=plane_2.y_max;
           plane_2.z_min=plane_2.z_min;
           plane_2.z_max=plane_2.z_max;

           tf::Vector3 point_A(plane_2.x_max,plane_2.y_min,plane_2.z_max);
           tf::Vector3 point_B(plane_2.x_max,plane_2.y_max,plane_2.z_max) ;
           tf::Vector3 point_C(plane_2.x_min,plane_2.y_max,plane_2.z_max);

           height_2 = sqrt((plane_2.x_max-plane_2.x_min)*(plane_2.x_max-plane_2.x_min)+(plane_2.y_max-plane_2.y_max)*(plane_2.y_max-plane_2.y_max)+(plane_2.z_max-plane_2.z_max)*(plane_2.z_max-plane_2.z_max));

           width_2  = sqrt((plane_2.x_max-plane_2.x_max)*(plane_2.x_max-plane_2.x_max)+(plane_2.y_max-plane_2.y_min)*(plane_2.y_max-plane_2.y_min)+(plane_2.z_max-plane_2.z_max)*(plane_2.z_max-plane_2.z_max));

           depth_2  = sqrt((plane_2.x_max-plane_2.x_max)*(plane_2.x_max-plane_2.x_max)+(plane_2.y_min-plane_2.y_min)*(plane_2.y_min-plane_2.y_min)+(plane_2.z_max-plane_2.z_min)*(plane_2.z_max-plane_2.z_min));
           

           plane_2.height=height_2;
           plane_2.width=width_2;
           plane_2.depth=depth_2;    
           ROS_INFO ("[ParallelepipedFitter::input_callback] Success for plane 2 in computing height,width,depth: %f,%f,%f.",height_2,width_2,depth_2);
           if(height_2<1.0 & width_2<1.0 & depth_2<1.0)
           {
             response.plane_2 = plane_2;

            //Parallelepiped parallelepiped;

             uint32_t cube_shape = visualization_msgs::Marker::CUBE;
             visualization_msgs::Marker cube_marker;
             // Set the marker type to arrow
             cube_marker.type = cube_shape;

             // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
             cube_marker.header.frame_id = "/head_asus/camera_rgb_optical_frame";
             //cube_marker.header.frame_id = "/vito_anchor";

             cube_marker.header.stamp = ros::Time::now();

             // // Set the namespace and id for this marker.  This serves to create a unique ID
             // // Any marker sent with the same namespace and id will overwrite the old one
             cube_marker.ns = "parallelepiped_fitting_node";
             cube_marker.id = n_object+2;

             // // Set the marker action.  Options are ADD and DELETE
             cube_marker.action = visualization_msgs::Marker::ADD;

             //tf::Vector3 origin_2 = plane_2_in_base.getOrigin();
             //tf::Quaternion quaternion_2 = plane_2_in_base.getRotation();
               
             cube_marker.pose.position.x = origin_2[0];
             cube_marker.pose.position.y = origin_2[1];
             cube_marker.pose.position.z = origin_2[2];
             cube_marker.pose.orientation.x = quaternion_2[0];
             cube_marker.pose.orientation.y = quaternion_2[1];
             cube_marker.pose.orientation.z = quaternion_2[2];
             cube_marker.pose.orientation.w = quaternion_2[3];
                
             //Controll plane fitting
             int n_inliers_2;
             n_inliers_2=plane_model2_->countWithinDistance(plane_coeff_2_optimized,(double)0.025);
             ROS_INFO("n_inliers del fitting %d",n_inliers_2);
             ROS_INFO("(int)cloud_ptr->size() %d",(int)cloud_2->size());

          
             //I= (float)plane_inliers_1.size()/(float)cloud_ptr->size();
             I_2= (float)n_inliers_2/(float)cloud_2->size();
             ROS_INFO("Indice di qualità del fitting %f",I_2);
           
             // // Set the scale of the marker 
             cube_marker.scale.x = fabs(height_2);
             cube_marker.scale.y = fabs(width_2);
             cube_marker.scale.z = fabs(depth_2);

             // // Set the color -- be sure to set alpha to something non-zero!
             cube_marker.color.r = 1.0f-(double)I_2;
             cube_marker.color.g = (double)I_2;
             cube_marker.color.b = 0.0f;
             cube_marker.color.a = 1.0;
             if((double)I_2>0.45)
             { 
             //response.result=response.SUCCESS;
             cube_marker.lifetime = ros::Duration(2);
             ROS_INFO("FINAL, publish the markers");
              // // Publish the marker
              parallelepipedfitting_pub_.publish(cube_marker);
             }
             else
             {
              cube_marker.lifetime = ros::Duration(0.5); 
              ROS_INFO("FINAL, publish the markers");
              // // Publish the marker
              parallelepipedfitting_pub_.publish(cube_marker);
             }          
           }
         } 


         }
   
         
         }

         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
         /////////// PLANE 3 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         
         if(j>=2){
         pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_3_ptr (new pcl::PointCloud<pcl::Normal>); 
         n3d_cluster_3.setInputCloud(cloud_3);
         n3d_cluster_3.compute(*cloud_normals_3_ptr); 
         pcl::SampleConsensusModelParallelPlane<Point>::Ptr plane_model3_ (new pcl::SampleConsensusModelParallelPlane<Point> (cloud_3));
         //pcl::SampleConsensusModelPlane<Point>::Ptr plane_model3_ (new pcl::SampleConsensusModelPlane<Point> (cloud_3));
         //pcl::SampleConsensusModelNormalParallelPlane<Point,PointN>::Ptr plane_model3_ (new pcl::SampleConsensusModelNormalParallelPlane<Point,PointN> (cloud_3,cloud_normals_3_ptr));
         plane_model3_->setAxis(Eigen::Vector3f (0.0, 1.0, 0.0));
         plane_model3_->setEpsAngle (pcl::deg2rad (45.0));
         ROS_INFO("WAYTING FOR FITTING plane  3 !!!"); 
         pcl::RandomSampleConsensus<Point> ransac3 (plane_model3_,1);
         ransac3.computeModel();
     
         ransac3.getModelCoefficients (pl_coeff_3);
         if (pl_coeff_1.size () <=3) {
         ROS_INFO("Failed to fit a plane to the cluster");
         }
         else
         {
          ROS_INFO(" Now, try to do a better fit, get the inliers for plane 3");
          std::vector<int> plane_inliers_3;
          ransac3.getInliers (plane_inliers_3);

          ROS_INFO("Give the previous coeff as an initial guess and optimize to fit to the inliers");
          Eigen::VectorXf plane_coeff_3_optimized;
          plane_model3_-> optimizeModelCoefficients(plane_inliers_3, pl_coeff_3, plane_coeff_3_optimized);

          ROS_INFO ("[ObjectFitter::input_callback] Plane Model coefficients optimized are: [%f %f %f %f].", plane_coeff_3_optimized[0], plane_coeff_3_optimized[1], plane_coeff_3_optimized[2], plane_coeff_3_optimized[3]);
          ROS_INFO("Step 3 Plane_3 done");

          //Project the inliers into the cylinder model to obtain the cylinder height and the center of the cylinder.
          // Hard code to convert from Eigen::VecctorXf to pcl::ModelCoefficients and pcl::PointIndices  
          pcl::PointIndices::Ptr plane_inliers_3_ptr (new pcl::PointIndices); 
          plane_inliers_3_ptr->indices.resize((int)plane_inliers_3.size());
                
          pcl::ModelCoefficients::Ptr plane_3_coefficients_ptr (new pcl::ModelCoefficients);
          plane_3_coefficients_ptr->values.resize(plane_coeff_3_optimized.size());
             
          for (int i = 0; i < plane_coeff_3_optimized.size(); i++)
          {   
            plane_3_coefficients_ptr->values[i] = plane_coeff_3_optimized[i];
          }
                 
          for (int i = 0; i < (int)plane_inliers_3.size(); i++)
           {
             plane_inliers_3_ptr->indices[i] = plane_inliers_3[i];
                  
           }
          // Step 4 : Project the table inliers on the table
          pcl::PointCloud<Point>::Ptr plane_projected_3_ptr (new pcl::PointCloud<Point>); 
          proj_.setInputCloud (cloud_3);
          proj_.setIndices (plane_inliers_3_ptr);
          proj_.setModelCoefficients (plane_3_coefficients_ptr);
          proj_.filter (*plane_projected_3_ptr);

          //          // Get Plane transform 
          tf::Transform plane_3_trans;
          tf::Transform plane_3_trans_flat;
          sensor_msgs::PointCloud plane_3_points;
          sensor_msgs::PointCloud plane_hull_3_points;


          plane_3_trans = getPlaneTransform(*plane_3_coefficients_ptr,up_direction_,false);
          hull_.setInputCloud (plane_projected_3_ptr);
          hull_.reconstruct (*plane_hull_3_ptr);
          plane_3_trans_flat = getPlaneTransform (*plane_3_coefficients_ptr, up_direction_, true);
          tf::Vector3 new_plane_pos_3;
          double avg_x_3, avg_y_3, avg_z_3;
          avg_x_3 = avg_y_3 = avg_z_3 = 0;
          for (size_t i=0; i<plane_projected_3_ptr->points.size(); i++)
          {
            avg_x_3 += plane_projected_3_ptr->points[i].x;
            avg_y_3 += plane_projected_3_ptr->points[i].y;
            avg_z_3 += plane_projected_3_ptr->points[i].z;
          }
          avg_x_3 /= plane_projected_3_ptr->points.size();
          avg_y_3 /= plane_projected_3_ptr->points.size();
          avg_z_3 /= plane_projected_3_ptr->points.size();
          ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x_3, avg_y_3, avg_z_3);

          // place the new plane frame in the center of the convex hull
          new_plane_pos_3[0] = avg_x_3;
          new_plane_pos_3[1] = avg_y_3;
          new_plane_pos_3[2] = avg_z_3;
          plane_3_trans.setOrigin(new_plane_pos_3);


          // shift the non-flat plane frame to the center of the convex hull as well
          plane_3_trans_flat.setOrigin(new_plane_pos_3); 
          
         ROS_INFO ("[PlaneFitter::input_callback] Success in computing the plane 3 transformation with 7 elements (pos, quat).");
         origin_3 = plane_3_trans.getOrigin();
         quaternion_3 = plane_3_trans.getRotation();
         ROS_INFO("Center of the plane at [%f %f %f]", origin_3[0], origin_3[1], origin_3[2]);
         
                
         //listener_.waitForTransform("plane_1", "left_arm_7_link", now, ros::Duration(4));
         //listener_.lookupTransform("plane_1", "left_arm_7_link", now, paralle_2_twist_stamped);

         tf::Transform plane_3_in_base;

         plane_3_in_base = base_2_camera*plane_3_trans;

         sensor_msgs::PointCloud plane_points_3;
         plane_points_3.header.frame_id="plane_3";

         ros::Time now = ros::Time::now();
         // broadcaster_.sendTransform(tf::StampedTransform(plane_1_trans_flat, ros::Time::now(), "head_asus/camera_rgb_optical_frame", "plane_1_trans_flat"));
         broadcaster_.sendTransform(tf::StampedTransform(plane_3_trans, now, "head_asus/camera_rgb_optical_frame", "plane_3"));
         //tf::StampedTransform paralle_2_twist_stamped;

         if (getPlanePoints<Point>(*plane_projected_3_ptr, plane_3_trans, plane_points_3,"plane_3"))
         {
           Plane plane_3;
           //get the extents of the table
           if (!plane_points_3.points.empty()) 
           {
            plane_3.x_min = plane_points_3.points[0].x;
            plane_3.x_max = plane_points_3.points[0].x;
            plane_3.y_min = plane_points_3.points[0].y;
            plane_3.y_max = plane_points_3.points[0].y;
            plane_3.z_min = plane_points_3.points[0].z;
            plane_3.z_max = plane_points_3.points[0].z;
           }  
           for (size_t i=1; i<plane_points_3.points.size(); ++i) 
           {
            if (plane_points_3.points[i].x<plane_3.x_min && plane_points_3.points[i].x>-0.5) plane_3.x_min = plane_points_3.points[i].x;
            if (plane_points_3.points[i].x>plane_3.x_max && plane_points_3.points[i].x< 0.5) plane_3.x_max = plane_points_3.points[i].x;
            if (plane_points_3.points[i].y<plane_3.y_min && plane_points_3.points[i].y>-0.5) plane_3.y_min = plane_points_3.points[i].y;
            if (plane_points_3.points[i].y>plane_3.y_max && plane_points_3.points[i].y< 0.5) plane_3.y_max = plane_points_3.points[i].y;
            if (plane_points_3.points[i].z<plane_3.z_min && plane_points_3.points[i].z>-0.5) plane_3.z_min = plane_points_3.points[i].z;
            if (plane_points_3.points[i].z>plane_3.z_max && plane_points_3.points[i].z< 0.5) plane_3.z_max = plane_points_3.points[i].z;
           }
                

           ROS_INFO ("[PlaneFitter::input_callback] Success for plane 3 in computing plane.x_min,plane.x_max,plane.y_min,plane.y_max,plane.z_min,plane.z_max: %f,%f,%f,%f,%f,%f.",plane_3.x_min,plane_3.x_max,plane_3.y_min,plane_3.y_max,plane_3.z_min,plane_3.z_max);

           geometry_msgs::Pose plane_pose_3;
           // pusblish the cylinder in the camera frame
           tf::poseTFToMsg(plane_3_trans, plane_pose_3);
           // pusblish the cylinder in the robot frame
           //tf::poseTFToMsg(plane_1_in_base, plane_pose);
           plane_3.pose.pose = plane_pose_3;

           plane_3.x_min=plane_3.x_min;
           plane_3.x_max= plane_3.x_max;
           plane_3.y_min=plane_3.y_min;
           plane_3.y_max=plane_3.y_max;
           plane_3.z_min=plane_3.z_min;
           plane_3.z_max=plane_3.z_max;

           tf::Vector3 point_A(plane_3.x_max,plane_3.y_min,plane_3.z_max);
           tf::Vector3 point_B(plane_3.x_max,plane_3.y_max,plane_3.z_max) ;
           tf::Vector3 point_C(plane_3.x_min,plane_3.y_max,plane_3.z_max);

           height_3 = sqrt((plane_3.x_max-plane_3.x_min)*(plane_3.x_max-plane_3.x_min)+(plane_3.y_max-plane_3.y_max)*(plane_3.y_max-plane_3.y_max)+(plane_3.z_max-plane_3.z_max)*(plane_3.z_max-plane_3.z_max));
 
           width_3  = sqrt((plane_3.x_max-plane_3.x_max)*(plane_3.x_max-plane_3.x_max)+(plane_3.y_max-plane_3.y_min)*(plane_3.y_max-plane_3.y_min)+(plane_3.z_max-plane_3.z_max)*(plane_3.z_max-plane_3.z_max));
 
           depth_3  = sqrt((plane_3.x_max-plane_3.x_max)*(plane_3.x_max-plane_3.x_max)+(plane_3.y_min-plane_3.y_min)*(plane_3.y_min-plane_3.y_min)+(plane_3.z_max-plane_3.z_min)*(plane_3.z_max-plane_3.z_min));
               
           ROS_INFO ("[ParallelepipedFitter::input_callback] Success in computing height,width,depth: %f,%f,%f.",height_3,width_3,depth_3);
           

           plane_3.height=height_3;
           plane_3.width=width_3;
           plane_3.depth=depth_3;

           if(height_3<1.0 & width_3<1.0 & depth_3<1.0)
           {
             response.plane_3 = plane_3;

             //Parallelepiped parallelepiped;

              uint32_t cube_shape = visualization_msgs::Marker::CUBE;
              visualization_msgs::Marker cube_marker;
              // Set the marker type to arrow
              cube_marker.type = cube_shape;

              // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
              cube_marker.header.frame_id = "/head_asus/camera_rgb_optical_frame";
              //cube_marker.header.frame_id = "/vito_anchor";

              cube_marker.header.stamp = ros::Time::now();

              // // Set the namespace and id for this marker.  This serves to create a unique ID
              // // Any marker sent with the same namespace and id will overwrite the old one
              cube_marker.ns = "parallelepiped_fitting_node";
              cube_marker.id = n_object+3;

              // // Set the marker action.  Options are ADD and DELETE
              cube_marker.action = visualization_msgs::Marker::ADD;

              //tf::Vector3 origin_3 = plane_3_in_base.getOrigin();
              //tf::Quaternion quaternion_3 = plane_3_in_base.getRotation();
               
              cube_marker.pose.position.x = origin_3[0];
              cube_marker.pose.position.y = origin_3[1];
              cube_marker.pose.position.z = origin_3[2];
              cube_marker.pose.orientation.x = quaternion_3[0];
              cube_marker.pose.orientation.y = quaternion_3[1];
              cube_marker.pose.orientation.z = quaternion_3[2];
              cube_marker.pose.orientation.w = quaternion_3[3];
                
              //Controll plane fitting
              int n_inliers_3;
              n_inliers_3=plane_model3_->countWithinDistance(plane_coeff_3_optimized,(double)0.025);
              ROS_INFO("n_inliers del fitting %d",n_inliers_3);
              ROS_INFO("(int)cloud_ptr->size() %d",(int)cloud_3->size());

             
             //I= (float)plane_inliers_1.size()/(float)cloud_ptr->size();
             I_3= (float)n_inliers_3/(float)cloud_3->size();
             ROS_INFO("Indice di qualità del fitting %f",I_3);
           
             // // Set the scale of the marker 
             cube_marker.scale.x = fabs(height_3);
             cube_marker.scale.y = fabs(width_3);
             cube_marker.scale.z = fabs(depth_3);

             // // Set the color -- be sure to set alpha to something non-zero!
             cube_marker.color.r = 1.0f-(double)I_3;
             cube_marker.color.g = (double)I_3;
             cube_marker.color.b = 0.0f;
             cube_marker.color.a = 1.0;
             if((double)I_3>0.45)
             { 
             //response.result=response.SUCCESS;
             cube_marker.lifetime = ros::Duration(2);
             ROS_INFO("FINAL, publish the markers");
              // // Publish the marker
              parallelepipedfitting_pub_.publish(cube_marker);
             }
             else
             {
              cube_marker.lifetime = ros::Duration(0.5); 
              ROS_INFO("FINAL, publish the markers");
              // // Publish the marker
              parallelepipedfitting_pub_.publish(cube_marker);
             }          
           }
         } 


        }
      
      }   
      
      
     
       
       uint32_t cube_shape = visualization_msgs::Marker::CUBE;
              visualization_msgs::Marker cube_marker;
              // Set the marker type to arrow
              cube_marker.type = cube_shape;

              // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
              cube_marker.header.frame_id = "/head_asus/camera_rgb_optical_frame";
              //cube_marker.header.frame_id = "/vito_anchor";

              cube_marker.header.stamp = ros::Time::now();

              // // Set the namespace and id for this marker.  This serves to create a unique ID
              // // Any marker sent with the same namespace and id will overwrite the old one
              cube_marker.ns = "parallelepiped_fitting_node";
              cube_marker.id = n_object+4;

              // // Set the marker action.  Options are ADD and DELETE
              cube_marker.action = visualization_msgs::Marker::ADD;

              //tf::Vector3 origin_3 = plane_3_in_base.getOrigin();
              //tf::Quaternion quaternion_3 = plane_3_in_base.getRotation();
               
              //cube_marker.pose.position.x = (origin_1[0]+origin_3[0])/2;
              //cube_marker.pose.position.y = (origin_1[1]+origin_3[1])/2;
              // cube_marker.pose.position.z = (origin_1[2]+origin_3[2])/2;
              cube_marker.pose.position.x = origin_1[0];
              cube_marker.pose.position.y = origin_1[1];
              cube_marker.pose.position.z = origin_1[2];
              cube_marker.pose.orientation.x = quaternion_1[0];
              cube_marker.pose.orientation.y = quaternion_1[1];
              cube_marker.pose.orientation.z = quaternion_1[2];
              cube_marker.pose.orientation.w = quaternion_1[3];
              
           
             // // Set the scale of the marker 
             cube_marker.scale.x = fabs(width_1);
             cube_marker.scale.y = fabs(height_1);
             
              if(height_3==(float)0)
             {
               ROS_INFO("height = 0");
               cube_marker.scale.z = fabs(height_2);
               response.parallelepiped.w=height_2; 
               }
              else
             {
              ROS_INFO("height_3 != 0 ");
             
               cube_marker.scale.z = fabs(width_3/2);
               response.parallelepiped.w=width_3/2;
              }

             // // Set the color -- be sure to set alpha to something non-zero!
             cube_marker.color.r = 0.0f;
             cube_marker.color.g = 1.0f;
             cube_marker.color.b = 0.0f;
             cube_marker.color.a = 1.0;
            
             cube_marker.lifetime = ros::Duration(10);
             ROS_INFO("FINAL, publish the markers");
              // // Publish the marker
            parallelepipedfitting_pub_.publish(cube_marker);
            
            response.I=I_1*I_2*I_3;
            geometry_msgs::Pose paral_pose;
            tf::Transform paral_trans;
            paral_trans.setOrigin(origin_1);
            paral_trans.setRotation(quaternion_1);
            tf::poseTFToMsg(paral_trans, paral_pose);
            
            visual_perception::Parallelepiped parallelepiped;
            
            parallelepiped.l=height_1;
            parallelepiped.b=width_1;
            //parallelepiped.w=width_3/2;
            parallelepiped.pose.pose=paral_pose;
            response.parallelepiped=parallelepiped;
             
 
    }
  }   
}     










} //namespace visual_perception

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "parallelepiped_fitting_node");
  ros::NodeHandle nh;

  visual_perception::ParallelepipedFitter node(nh);

  ros::spin();
  return 0;
}