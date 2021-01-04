#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include <rspd/planedetector.h>

using namespace open3d;

std::shared_ptr<geometry::TriangleMesh> makePlane(
    const Eigen::Vector3d& center, const Eigen::Vector3d& normal,
    const Eigen::Vector3d& basisU, const Eigen::Vector3d& basisV)
{
    auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();

    mesh_ptr->vertices_.resize(8);
    mesh_ptr->vertices_[0] = center - basisU - basisV;
    mesh_ptr->vertices_[1] = center - basisU + basisV;
    mesh_ptr->vertices_[2] = center + basisU - basisV;
    mesh_ptr->vertices_[3] = center + basisU + basisV;

    mesh_ptr->vertices_[4] = center + 0.01*normal - basisU - basisV;
    mesh_ptr->vertices_[5] = center + 0.01*normal - basisU + basisV;
    mesh_ptr->vertices_[6] = center + 0.01*normal + basisU - basisV;
    mesh_ptr->vertices_[7] = center + 0.01*normal + basisU + basisV;
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

    // utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc < 2) {
        utility::LogInfo("Open3D {}", OPEN3D_VERSION);
        utility::LogInfo("Usage:");
        utility::LogInfo("    > TestVisualizer [filename]");
        return 1;
    }

    static constexpr int nrNeighbors = 75;
    const geometry::KDTreeSearchParam &search_param = geometry::KDTreeSearchParamKNN(nrNeighbors);

    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(argv[1], *cloud_ptr)) {
        utility::LogInfo("Successfully read {}\n", argv[1]);
    } else {
        utility::LogWarning("Failed to read {}\n\n", argv[1]);
        return 1;
    }

    if (cloud_ptr->HasNormals()) {
        std::cout << "already has normals" << std::endl;
    } else {
        std::cout << "no normals" << std::endl;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    cloud_ptr->EstimateNormals(search_param);
    const double t_normals = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
    std::cout << "o3d EstimateNormals: " << t_normals << " seconds" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*cloud_ptr);
    std::vector<std::vector<int>> neighbors;
    neighbors.resize(cloud_ptr->points_.size());
    std::cout << "num points: " << cloud_ptr->points_.size() << std::endl;
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)cloud_ptr->points_.size(); i++) {
        std::vector<int> indices;
        std::vector<double> distance2;
        if (kdtree.Search(cloud_ptr->points_[i], search_param, indices, distance2)/* >= 3*/) {
            neighbors[i] = indices;
        }
    }
    const double t_kdtree = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
    std::cout << "kdtree search: " << t_kdtree << " seconds" << std::endl;


    std::cout << "o3d cloud: center: " << cloud_ptr->GetCenter().transpose() << std::endl;
    Eigen::Vector3d bl = cloud_ptr->GetMinBound();
    Eigen::Vector3d tr = cloud_ptr->GetMaxBound();
    std::cout << "o3d cloud: ext center: " << ((bl + tr) / 2).transpose() << std::endl;
    std::cout << "o3d cloud: bottom left: " << bl.transpose() << std::endl;
    std::cout << "o3d cloud: top right: " << tr.transpose() << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    PlaneDetector rspd(cloud_ptr, neighbors);
    std::set<Plane*> planes = rspd.detect();
    const double t_rspd = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
    std::cout << "rspd detect: " << t_rspd << " seconds" << std::endl;
    std::cout << std::endl;

    std::cout << "Detected the following " << planes.size() << " planes:" << std::endl;
    for (const auto& p : planes) {
        std::cout << p->normal().transpose() << " ";
        std::cout << p->distanceFromOrigin() << "\t";
        std::cout << p->center().transpose() << "\t";
        std::cout << p->basisU().transpose() << "\t";
        std::cout << p->basisV().transpose() << std::endl;
    }
    std::cout << "==============================" << std::endl;

    std::cout << std::endl;
    std::cout << "EstimateNormals: " << t_normals << " seconds" << std::endl;
    std::cout << "DetectPlanarPatches: " << planes.size() << " in " << (t_rspd+t_kdtree) << " seconds" << std::endl;

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

    visualization::DrawGeometries(geometries, "Points and Planes", 1600, 900);

    // visualization::VisualizerWithVertexSelection visualizer;
    // visualizer.CreateVisualizerWindow("Plane Selection: Select Point Close to Desired Plane", 1600, 900);
    // visualizer.AddGeometry(cloud_ptr);
    // visualizer.Run();
    // visualizer.DestroyVisualizerWindow();
    // const auto pts = visualizer.GetPickedPoints();

    // for (const auto& pt : pts) {
    //     double d = std::numeric_limits<double>::max();
    //     Plane* closest_plane;
    //     for (const auto& p : planes) {
    //         if (std::abs(p->getSignedDistanceFromSurface(pt.coord.cast<float>())) < d) {
    //             d = std::abs(p->getSignedDistanceFromSurface(pt.coord.cast<float>()));
    //             closest_plane = p;
    //         }
    //     }

    //     if (closest_plane == nullptr) {
    //         std::cout << "Could not find closest plane to selected point!" << std::endl;
    //     } else {
    //         std::cout << closest_plane->normal().transpose() << " ";
    //         std::cout << closest_plane->distanceFromOrigin() << "\t";
    //         std::cout << closest_plane->center().transpose() << "\t";
    //         std::cout << closest_plane->basisU().transpose() << "\t";
    //         std::cout << closest_plane->basisV().transpose() << std::endl;
    //     }
    // }

    // utility::LogInfo("End of the test.\n");

    return 0;
}
