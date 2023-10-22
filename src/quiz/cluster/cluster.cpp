/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include <unordered_set>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}


/**
 * @brief Recursively finds nearby points within a given distance tolerance and adds them to a cluster.
 * 
 * @param points Vector of points to search from.
 * @param id Index of the point to search from.
 * @param tree Pointer to the KdTree object used for searching.
 * @param distanceTol Distance tolerance for finding nearby points.
 * @param visited Set of visited point indices.
 * @param cluster Vector of point indices belonging to the same cluster.
 */

void findNearPoints(const std::vector<std::vector<float>>& points, int id, KdTree* tree, float distanceTol, std::unordered_set<int>& visited, std::vector<int>& cluster) {
    visited.insert(id);
    cluster.push_back(id);
    std::vector<int> nearby = tree->search(points[id], distanceTol);
    for (auto index : nearby) {
        if (visited.find(index) == visited.end()) {
            findNearPoints(points, index, tree, distanceTol, visited, cluster);
        }
    }

}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

    std::unordered_set<int> visited; 
    for (size_t index = 0; index < points.size(); ++index) {
        if (visited.find(index) != visited.end()) 
            continue;
        std::vector<int> cluster;
        findNearPoints(points, index, tree, distanceTol, visited, cluster);
        if (!cluster.empty())
            clusters.push_back(cluster);
    }
 
	return clusters;

}

/**
 * @brief This is the main function that performs clustering on a set of 2D points using euclidean clustering algorithm.
 * 
 * It creates a 2D viewer window and initializes a point cloud with a set of 2D points. It then creates a KdTree and inserts the points into it.
 * The function then performs a search operation on the KdTree to find nearby points within a specified distance. 
 * It then performs euclidean clustering on the points and renders the clusters in different colors.
 * 
 * @return int 
 */
int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
        tree->insert(points[i],i); 

	int it = 0;
	render2DTree(tree->root,viewer,window, it);

	std::cout << "Test Search" << std::endl;
	std::vector<int> nearby = tree->search({-6,7},3.0);
	for(int index : nearby) {
        std::cout << "index: " << index << ": " << points[index][0] << "," << points[index][1] << std::endl;
    }
	std::cout << std::endl;

	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	//
	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
	//
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Render clusters
	int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
	for(std::vector<int> cluster : clusters)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
		for(int indice: cluster)
			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
		++clusterId;
	}
	if(clusters.size()==0)
		renderPointCloud(viewer,cloud,"data");

	while (!viewer->wasStopped ())
	{
	  viewer->spinOnce ();
	}
  	
}
