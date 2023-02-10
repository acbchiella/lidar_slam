#include <iostream>
#include <vector>
#include "include/utils.h"




int idx = 0;
bool matching = true;
void keyboardEvents(const pcl::visualization::KeyboardEvent& event, void* viewer) {
    if (event.getKeySym() == "space") {
        idx++;
        matching = false;
    }
}


int main() {

    std::unique_ptr<DataScans> ds(new DataScans("./Data/lidar.txt"));
    std::unique_ptr<DataGroundTruth> gt(new DataGroundTruth("./Data/ground_truth.txt"));
    std::unique_ptr<DataEncoder> odom(new DataEncoder("./Data/encoder.txt"));
    


    // Viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
    viewer->registerKeyboardCallback(keyboardEvents, (void*)&viewer);

 
    PointCloudT::Ptr scan (new PointCloudT);

    scan = ds->scans[0];
    // viewer->addPointCloud<PointT> (scan, "scan");

	// viewer->addPointCloud<PointT> (ds->scans[100], "scan2");
	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "scan2");

    for (int i = 0; i<gt->points.size()-2; i++) {
        std::string name = "GroundTruth" + std::to_string(i);
        viewer->addLine(gt->points[i], gt->points[i+1], 1, 0, 0, name);
        // std::cout << gt->points.size() << std::endl;
    }

	for (int i = 0; i<odom->poses.size()-2; i++) {
        std::string name = "Odom" + std::to_string(i);

		PointT pointB(odom->poses[i].position.x, odom->poses[i].position.y, 0);
		PointT pointA(odom->poses[i+1].position.x, odom->poses[i+1].position.y, 0);
        viewer->addLine(pointB, pointA, 0, 0, 1, name);
		// odom->poses[i].position.Print();
    }



    // ScanMatching
    Pose pose(Point(0,0,0), Rotate(3.717551306747922,0,0));
    Pose pose_b(Point(1850.0,1897.0,0), Rotate(3.717551306747922,0,0));
    Pose pose_c(Point(1850.0,1897.0,0), Rotate(3.717551306747922,0,0));
    for (int idx = 0; idx<ds->scans.size()-2; idx++) {
        Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
		transform = ICP(ds->scans[idx], ds->scans[idx+1], pose, 10);
	
		pose_b = pose_c;
		pose_c = getPose(transform);

		std::string nameSM = "SM" + std::to_string(idx);

		pose_c.position.x += pose_b.position.x;
		pose_c.position.y += pose_b.position.y;
		pose_c.position.z = 0;
		pose_c.rotation.yaw += pose_b.rotation.yaw;
		pose_c.rotation.pitch += pose_b.rotation.pitch;
		pose_c.rotation.roll += pose_b.rotation.roll;

		// pose.rotation.yaw = pose_c.rotation.yaw;
		pose.rotation.yaw = odom->poses[idx].rotation.yaw;
		pose.rotation.pitch = pose.rotation.pitch;
		pose.rotation.roll = pose.rotation.roll;

		PointT pointB(pose_b.position.x, pose_b.position.y, 0);
		PointT pointA(pose_c.position.x, pose_c.position.y, 0);

		viewer->addLine(pointB, pointA, 0, 1, 0, nameSM);
		// std::cout << idx << ": ";
		// pose_c.position.Print(); 
		// std::cout << idx << ": ";
		// pose_c.rotation.Print(); 
    }


    while (!viewer->wasStopped()) {
        
        if (matching == false) {
			matching = true;
        }
        
        viewer->spinOnce();
    }

    return 0;
}