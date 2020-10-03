#include <chrono>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include <rspd/normalestimator.h>
#include <rspd/planedetector.h>
#include <rspd/pointcloud.h>
#include <rspd/point.h>

using namespace open3d;

PointCloud3d* fromOpen3D(const open3d::geometry::PointCloud& cloud)
{
    std::vector<Point3d> points;
    points.reserve(cloud.points_.size());
    for (const auto& p : cloud.points_) {
        points.emplace_back(p.cast<float>());
    }
    return new PointCloud3d(points);
}

// ----------------------------------------------------------------------------

void estimateNormals(PointCloud3d* pointCloud)
{
    Octree octree(pointCloud);
    octree.partition(10, 30);

    ConnectivityGraph* connectivity = new ConnectivityGraph(pointCloud->size());
    pointCloud->connectivity(connectivity);

    static constexpr int nrNeighbors = 21;
    NormalEstimator3d estimator(&octree, nrNeighbors, NormalEstimator3d::QUICK);

#pragma omp parallel for
    for (size_t i=0; i<pointCloud->size(); ++i) {
        NormalEstimator3d::Normal normal = estimator.estimate(i);
        #pragma omp critical
        connectivity->addNode(i, normal.neighbors);
        (*pointCloud)[i].normal(normal.normal);
        (*pointCloud)[i].normalConfidence(normal.confidence);
        (*pointCloud)[i].curvature(normal.curvature);
    }
}

// ----------------------------------------------------------------------------

std::shared_ptr<geometry::TriangleMesh> makePlane(
    const Eigen::Vector3f& center, const Eigen::Vector3f& normal,
    const Eigen::Vector3f& basisU, const Eigen::Vector3f& basisV)
{
    auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();

    mesh_ptr->vertices_.resize(8);
    mesh_ptr->vertices_[0] = (center - basisU - basisV).cast<double>();
    mesh_ptr->vertices_[1] = (center - basisU + basisV).cast<double>();
    mesh_ptr->vertices_[2] = (center + basisU - basisV).cast<double>();
    mesh_ptr->vertices_[3] = (center + basisU + basisV).cast<double>();

    mesh_ptr->vertices_[4] = (center + 0.01*normal - basisU - basisV).cast<double>();
    mesh_ptr->vertices_[5] = (center + 0.01*normal - basisU + basisV).cast<double>();
    mesh_ptr->vertices_[6] = (center + 0.01*normal + basisU - basisV).cast<double>();
    mesh_ptr->vertices_[7] = (center + 0.01*normal + basisU + basisV).cast<double>();
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(4, 7, 5));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(4, 6, 7));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(0, 2, 4));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(2, 6, 4));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(0, 1, 2));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 3, 2));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 5, 7));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 7, 3));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(2, 3, 7));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(2, 7, 6));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(0, 4, 1));
    mesh_ptr->triangles_.push_back(Eigen::Vector3i(1, 4, 5));
    return mesh_ptr;
}

// ----------------------------------------------------------------------------

int main(int argc, char *argv[]) {

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 2) {
        utility::LogInfo("Open3D {}", OPEN3D_VERSION);
        utility::LogInfo("Usage:");
        utility::LogInfo("    > TestVisualizer [filename]");
        return 1;
    }


    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(argv[1], *cloud_ptr)) {
        utility::LogInfo("Successfully read {}\n", argv[1]);
    } else {
        utility::LogWarning("Failed to read {}\n\n", argv[1]);
        return 1;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    cloud_ptr->EstimateNormals();
    std::cout << "o3d EstimateNormals: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count() << " seconds" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    PointCloud3d* pointCloud = fromOpen3D(*cloud_ptr);
    std::cout << "cloud convert: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count() << " seconds" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    estimateNormals(pointCloud);
    std::cout << "rspd estimateNormals: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count() << " seconds" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    PlaneDetector rspd(pointCloud);
    std::set<Plane*> planes = rspd.detect();
    std::cout << "rspd detect: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count() << " seconds" << std::endl;

    //
    // Visualization
    //

    // create a vector of geometries to visualize, starting with input point cloud
    std::vector<std::shared_ptr<const geometry::Geometry>> geometries;
    geometries.reserve(1 + planes.size());
    geometries.push_back(cloud_ptr);

    // Colors (default MATLAB colors)
    std::vector<Eigen::Vector3d> colors;
    colors.push_back(Eigen::Vector3d(0.8500, 0.3250, 0.0980));
    colors.push_back(Eigen::Vector3d(0.9290, 0.6940, 0.1250));
    colors.push_back(Eigen::Vector3d(0.4940, 0.1840, 0.5560));
    colors.push_back(Eigen::Vector3d(0.4660, 0.6740, 0.1880));
    colors.push_back(Eigen::Vector3d(0.3010, 0.7450, 0.9330));
    colors.push_back(Eigen::Vector3d(0.6350, 0.0780, 0.1840));

    // add any planes
    size_t i = 0;
    for (const auto& p : planes) {
        auto pviz = makePlane(p->center(), p->normal(), p->basisU(), p->basisV());
        pviz->PaintUniformColor(colors[i%6]);
        geometries.push_back(pviz);

        ++i;
    }

    visualization::DrawGeometries(geometries, "PointCloud", 1600, 900);
    utility::LogInfo("End of the test.\n");

    return 0;
}
