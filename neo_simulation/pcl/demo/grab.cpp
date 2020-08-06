#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <obj_recognition/SegmentedClustersArray.h>
//#include <obj_recognition/ClusterData.h>




// Topics
static const std::string IMAGE_TOPIC = "/passthrough/output";
static const std::string PUBLISH_TOPIC = "/pcl/points";

static const float val = 0.02; 
// ROS Publisher
ros::Publisher pub;
pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2), cloud_filtered (new pcl::PCLPointCloud2);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>), plane (new pcl::PointCloud<pcl::PointXYZRGB>), object (new pcl::PointCloud<pcl::PointXYZRGB>);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
// Container for original & filtered data
    
    //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    //pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (val , val, val);
    vox.filter (*cloud_filtered);
    pcl::fromPCLPointCloud2(*cloud_filtered,*temp_cloud);
    //pcl::PointCloud<pcl::PointXYZRGB> *y_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (y_cloud_filtered);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr y_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object
    /*
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (temp_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.189, 2);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*temp_cloud);
    */
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
  
    seg.setInputCloud (temp_cloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane);

    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*object);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (object);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.2); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (object);
    ec.extract (cluster_indices);

    // declare an instance of the SegmentedClustersArray message
    //obj_recognition::SegmentedClustersArray CloudClusters;

    // declare the output variable instances
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
/*
  int j= 0;
 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	  {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		    {
                pcl::PointXYZRGB point;
                point.x = object->points[*pit].x;
                point.y = object->points[*pit].y;
                point.z = object->points[*pit].z;

                if (j == 0) //Red	#FF0000	(255,0,0)
			     {
				      point.r = 0;
				      point.g = 0;
				      point.b = 255;
			     }
			    else if (j == 1) //Lime	#00FF00	(0,255,0)
			     {
				      point.r = 0;
				      point.g = 255;
				      point.b = 0;
			     }
			    else if (j == 2) // Blue	#0000FF	(0,0,255)
			     {
				      point.r = 255;
				      point.g = 0;
				      point.b = 0;
			     }
			    else if (j == 3) // Yellow	#FFFF00	(255,255,0)
			     {
				      point.r = 255;
				      point.g = 255;
				      point.b = 0;
			     }
			    else if (j == 4) //Cyan	#00FFFF	(0,255,255)
			     {
				      point.r = 0;
				      point.g = 255;
				      point.b = 255;
			     }
			    else if (j == 5) // Magenta	#FF00FF	(255,0,255)
			     {
				      point.r = 255;
				      point.g = 0;
				      point.b = 255;
			     }
			    else if (j == 6) // Olive	#808000	(128,128,0)
		     	 {
				      point.r = 128;
				      point.g = 128;
				      point.b = 0;
			     }
			    else if (j == 7) // Teal	#008080	(0,128,128)
			     {
				      point.r = 0;
				      point.g = 128;
				      point.b = 128;
			     }
			    else if (j == 8) // Purple	#800080	(128,0,128)
		     	 {
				      point.r = 128;
				      point.g = 0;
				      point.b = 128;
			     }
			    else
		   	     {
				      if (j % 2 == 0)
				       {
					        point.r = 255 * j / (cluster_indices.size());
					        point.g = 128;
					        point.b = 50;
				       }
				      else
				       {
					        point.r = 0;
					        point.g = 255 * j / (cluster_indices.size());
					        point.b = 128;
				       }
                 }
                point_cloud_segmented->push_back(point);
			
            }
        j++;
    }
  */
  // Convert to ROS data type
  //point_cloud_segmented->header.frame_id = object->header.frame_id;
  //if(point_cloud_segmented->size()) 
  //{
      pcl::toPCLPointCloud2(*plane, outputPCL);
  //}
  
  //else 
  //{
  //   pcl::toPCLPointCloud2(*object, outputPCL);
  //}
  
  pcl_conversions::fromPCL(outputPCL, output);

  // Publish the data
  //pcl::toROSMsg(*plane, output);
  pub.publish (output);
  
}



int main (int argc, char** argv)
{
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "filter");
    ros::NodeHandle nh;

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
    //pub = nh.advertise<obj_recognition::SegmentedClustersArray> ("obj_recognition/pcl_clusters",1);
    // Spin
    ros::spin();

    // Success
    return 0;
}

