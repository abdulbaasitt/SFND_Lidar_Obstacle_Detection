/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <chrono>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}



/**
 * Ransac function that takes a point cloud, maximum number of iterations and distance tolerance as input
 * and returns a set of inliers that fit a 2D line model using RANSAC algorithm.
 * 
 * @param cloud A pointer to a point cloud of type pcl::PointCloud<pcl::PointXYZ>.
 * @param maxIterations The maximum number of iterations to run the RANSAC algorithm.
 * @param distanceTol The distance tolerance to determine if a point is an inlier or outlier.
 * @return A set of indices of inliers that fit a 2D line model using RANSAC algorithm.
 */

// std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
//   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
// 	std::unordered_set<int> inliersResult;
// 	srand(time(NULL));
	
// 	// TODO: Fill in this function

// 	// For max iterations 

// 	// Randomly sample subset and fit line

// 	// Measure distance between every point and fitted line
// 	// If distance is smaller than threshold count it as inlier

// 	// Return indicies of inliers from fitted line with most inliers
//   while (maxIterations--) {
//     int index1 = std::rand() % cloud->points.size();
//     int index2 = std::rand() % cloud->points.size();
//     if (index1 == index2) 
//       continue;
//     float x1 = cloud->points[index1].x; 
//     float y1 = cloud->points[index1].y;     
//     float x2 = cloud->points[index2].x; 
//     float y2 = cloud->points[index2].y; 

//     float a = y1 - y2;
//     float b = x2 - x1;
//     float c = x1 * y2 - x2 * y1;

//     std::unordered_set<int> inliers; 
//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//       float x3 = cloud->points[i].x;
//       float y3 = cloud->points[i].y;
//       float dist = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b); 
//       if (dist <= distanceTol) 
//         inliers.insert(i);
//     }

//     if (inliers.find(index1) == inliers.end()) inliers.insert(index1);
//     if (inliers.find(index2) == inliers.end()) inliers.insert(index2);

//     if (inliers.size() > inliersResult.size()) inliersResult = inliers;

//   }
	
//   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//   std::cout << "Ransac Time Duration = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

// 	return inliersResult;

// }

/**
 * RANSAC algorithm for 3D point cloud segmentation.
 * 
 * @param cloud Pointer to the input point cloud.
 * @param maxIterations Maximum number of iterations to run RANSAC.
 * @param distanceTol Distance tolerance threshold for inlier points.
 * @return Unordered set of indices of inlier points.
 */
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();   
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  while (maxIterations--) {
    int index1 = std::rand() % cloud->points.size();
    int index2 = std::rand() % cloud->points.size();
    int index3 = std::rand() % cloud->points.size();
    if (index1 == index2 || index1 == index3 || index2 == index3) 
      continue;
    float x1 = cloud->points[index1].x; 
    float y1 = cloud->points[index1].y;    
    float z1 = cloud->points[index1].z; 
    float x2 = cloud->points[index2].x; 
    float y2 = cloud->points[index2].y;
    float z2 = cloud->points[index2].z;
    float x3 = cloud->points[index3].x; 
    float y3 = cloud->points[index3].y; 
    float z3 = cloud->points[index3].z; 

    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);

    std::unordered_set<int> inliers; 
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      float x4 = cloud->points[i].x;
      float y4 = cloud->points[i].y;
      float z4 = cloud->points[i].z;
      float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c); 
      if (dist <= distanceTol) 
        inliers.insert(i);
    }

    if (inliers.find(index1) == inliers.end()) inliers.insert(index1);
    if (inliers.find(index2) == inliers.end()) inliers.insert(index2);
    if (inliers.find(index3) == inliers.end()) inliers.insert(index3);

    if (inliers.size() > inliersResult.size()) inliersResult = inliers;

  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Ransac3d Time Duration = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

  return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  	renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
