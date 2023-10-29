/* Created by Abdulbaasitt 
as correction to first submission */
// 

#ifndef RANSAC3D_H
#define RANSAC3D_H

#include <unordered_set>
#include <pcl/common/common.h>

namespace lidar_obstacle_detection {

    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>
    class Ransac {
    private:
        int maxIterations;
        float distanceTol;
        int num_points;

    public:
        Ransac(int maxIter, float distTol, int nPts) : maxIterations(maxIter), distanceTol(distTol), num_points(nPts) {}

        ~Ransac();

        std::unordered_set<int> Ransac3d(PtCdtr<PointT> cloud);
    };
}
#endif //RANSAC3D_H
