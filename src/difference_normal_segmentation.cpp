// New Pcl Difference of Normals Example for PCL Segmentation 
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>


ROS_INFO("Difference of normals segmentation: start !!");
    ///The smallest scale to use in the DoN filter.
    double scale1=0.1;

    ///The largest scale to use in the DoN filter.
    double scale2=5;

    ///The minimum DoN magnitude to threshold by
    double threshold=0.5;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius=0.5;

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
      pcl::ConditionalRemoval<PointN> condrem (range_cond);
      condrem.setInputCloud (doncloud);

      pcl::PointCloud<PointN>::Ptr doncloud_filtered (new pcl::PointCloud<PointN>);

      // Apply filter
      condrem.filter (*doncloud_filtered);
      pcl::PointIndices cluster_indices_rem;
      condrem.getRemovedIndices(cluster_indices_rem);

      doncloud = doncloud_filtered;

      // Save filtered output
      ROS_INFO("Filtered Pointcloud data points %f: ",(float) doncloud->points.size ());

      //  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

        // Filter by magnitude
      ROS_INFO("Clustering using EuclideanClusterExtraction with tolerance <= %f", (float)segradius);

      pcl::search::KdTree<PointN>::Ptr segtree (new pcl::search::KdTree<PointN>);
      segtree->setInputCloud (doncloud);

      // std::vector<pcl::PointIndices> cluster_indices;
      // pcl::EuclideanClusterExtraction<PointN> ec;

      // ec.setClusterTolerance (segradius);
      // ec.setMinClusterSize (50);
      // ec.setMaxClusterSize (10000);
      // ec.setSearchMethod (segtree);
      // ec.setInputCloud (doncloud);
      // ec.extract (cluster_indices);
      // pcl::PCDWriter writer;
      
        
          pcl::PointCloud<PointN>::Ptr cloud_cluster_rem (new pcl::PointCloud<PointN>);
          for (std::vector<int>::const_iterator pit = cluster_indices_rem.begin (); pit != cluster_indices_rem.end (); ++pit)
          {
            cluster_indices_rem->points.push_back (doncloud->points[*pit]);
          }

          cloud_cluster_rem->width = int (cloud_cluster_rem->points.size ());
          cloud_cluster_rem->height = 1;
          cloud_cluster_rem->is_dense = true;

          //Save cluster
           ROS_INFO("PointCloud representing the Cluster: %d data points.", (int)cloud_cluster_rem->points.size());
           std::stringstream ss;
           ss << "/home/pacman/Projects/pcd_file/rem_cluster.pcd";
           writer.write<PointN> (ss.str (), *cloud_cluster_rem, false);

           //pcl::visualization::CloudViewer viewer("Normal segmentation Viewer");
        
           //pcl::visualization::CloudViewer::showCloud(cloud_cluster_don,"don_cluster_"+j);   

           //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
           //viewer->initCameraParameters ();
           //viewer->setBackgroundColor (0, 0, 0);
           //viewer->addPointCloud<Point>(*cloud_cluster_don, "sample cloud");
           //viewer.showCloud(cloud_cluster_don);
           //while (!viewer.wasStopped()){}     
        


      // int j = 0;
      //   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
      //   {
      //     pcl::PointCloud<PointN>::Ptr cloud_cluster_don (new pcl::PointCloud<PointN>);
      //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      //     {
      //       cloud_cluster_don->points.push_back (doncloud->points[*pit]);
      //     }

      //     cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
      //     cloud_cluster_don->height = 1;
      //     cloud_cluster_don->is_dense = true;

      //     //Save cluster
      //      ROS_INFO("PointCloud representing the Cluster: %d data points.", (int)cloud_cluster_don->points.size());
      //      std::stringstream ss;
      //      ss << "/home/pacman/Projects/pcd_file/don_cluster_" << j << ".pcd";
      //      writer.write<PointN> (ss.str (), *cloud_cluster_don, false);

      //      //pcl::visualization::CloudViewer viewer("Normal segmentation Viewer");
        
      //      //pcl::visualization::CloudViewer::showCloud(cloud_cluster_don,"don_cluster_"+j);   

      //      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      //      //viewer->initCameraParameters ();
      //      //viewer->setBackgroundColor (0, 0, 0);
      //      //viewer->addPointCloud<Point>(*cloud_cluster_don, "sample cloud");
      //      //viewer.showCloud(cloud_cluster_don);
      //      //while (!viewer.wasStopped()){}     
      //   }

      }  