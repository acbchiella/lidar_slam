#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using Tick = int[2];

struct Point{
	double x, y, z;

	Point()
		: x(0), y(0), z(0){}

	Point(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ){}

	void Print(){
		cout << "x: " << x << " y: " << y << " z: " << z << endl;
	}
};

struct Pair{

	Point p1;
	Point p2;

	Pair(Point setP1, Point setP2)
		: p1(setP1), p2(setP2){}
};

struct Rotate{
	double yaw, pitch, roll;

	Rotate()
		: yaw(0), pitch(0), roll(0){}

	Rotate(double setYaw, double setPitch, double setRoll)
		: yaw(setYaw), pitch(setPitch), roll(setRoll){}

	void Print(){
		cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl;
	}
};

struct Pose{

	Point position;
	Rotate rotation;

	Pose()
		: position(Point(0, 0, 0)), rotation(Rotate(0, 0, 0)){}

	Pose(Point setPos, Rotate setRotation)
		: position(setPos), rotation(setRotation) {}

	Pose operator-(const Pose& p)
    {
        Pose result(Point(position.x-p.position.x, position.y-p.position.y, position.z-p.position.z), Rotate(rotation.yaw-p.rotation.yaw, rotation.pitch-p.rotation.pitch, rotation.roll-p.rotation.roll) );
        return result;
    }
};

Pose getPose(Eigen::Matrix4d matrix){

	Pose pose(Point(matrix(0,3), matrix(1,3), matrix(2,3)), Rotate(atan2(matrix(1, 0),matrix(0, 0)), atan2(-matrix(2,0), sqrt(matrix(2,1)*matrix(2,1) + matrix(2,2)*matrix(2,2))), atan2(matrix(2,1),matrix(2,2))));
	return pose;
}

Eigen::Matrix4d transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt){

	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

	matrix(0, 3) = xt;
	matrix(1, 3) = yt;
	matrix(2, 3) = zt;

	matrix(0, 0) = cos(yaw) * cos(pitch);
	matrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	matrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
	matrix(1, 0) = sin(yaw) * cos(pitch);
	matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	matrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
	matrix(2, 0) = -sin(pitch);
	matrix(2, 1) = cos(pitch) * sin(roll);
	matrix(2, 2) = cos(pitch) * cos(roll);

	return matrix;
}

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

	// Defining a rotation matrix and translation vector
  	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  	// align source with starting pose
  	Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
  	PointCloudT::Ptr transformSource (new PointCloudT); 
  	pcl::transformPointCloud (*source, *transformSource, initTransform);

	PointCloudT::Ptr transformTarget (new PointCloudT); 
  	pcl::transformPointCloud (*target, *transformTarget, initTransform);
	
	// pcl::console::TicToc time;
  	// time.tic ();
  	pcl::IterativeClosestPoint<PointT, PointT> icp;
  	icp.setMaximumIterations (iterations);
  	icp.setInputSource (transformSource);
  	icp.setInputTarget (transformTarget);
	icp.setMaxCorrespondenceDistance (50);

  	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
  	icp.align (*cloud_icp);

  	if (icp.hasConverged ())
  	{
  		transformation_matrix = icp.getFinalTransformation().cast<double>();
  		transformation_matrix =  transformation_matrix * initTransform;
  		//print4x4Matrix(transformation_matrix); // debug purpose
  		return transformation_matrix;
  	}
	else
  		cout << "WARNING: ICP did not converge" << endl;
  	return transformation_matrix;

}

std::vector<int> NN(PointCloudT::Ptr target, PointCloudT::Ptr source, Eigen::Matrix4d initTransform, double dist){
	
	std::vector<int> associations;

	// This function returns a vector of target indicies that correspond to each source index inorder.
	// E.G. source index 0 -> target index 32, source index 1 -> target index 5, source index 2 -> target index 17, ... 

	// Create a KDtree with target as input
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud (target);

	// Transform source by initTransform
  	PointCloudT::Ptr transformSource (new PointCloudT); 
  	pcl::transformPointCloud (*source, *transformSource, initTransform); 
	// Append the nearest point to associaitons 
	int index = 0;
	for (PointT point : transformSource->points){
		std::vector<int> pointIdxRadiusSearch;
  		std::vector<float> pointRadiusSquaredDistance;
		if ( kdtree.radiusSearch (point, dist, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
 		{
			associations.push_back(pointIdxRadiusSearch[0]);
		}
		else{
			associations.push_back(-1);
		}
		index++;
	}

	return associations;
}

std::vector<Pair> PairPoints(std::vector<int> associations, PointCloudT::Ptr target, PointCloudT::Ptr source){

	std::vector<Pair> pairs;

	// Loop through each source point and using the corresponding associations append a Pair of (source point, associated target point)
	int index = 0;
	for(PointT point : source->points ){
		int i = associations[index];
		if( i >= 0)
		{
			PointT association = (*target)[i];
			pairs.push_back(Pair(Point(point.x, point.y,0), Point(association.x, association.y,0)) );
		}
		index++;
	}
	
	return pairs;
}



class DataScans {
    public:
        std::vector<PointCloudT::Ptr> scans;

        DataScans(std::string address) {
            this->address = address;
            loadPointCloud(address);
        }

        ~DataScans() {
            std::cout << "Data from " + address + " was removed." << std::endl;
        }

    private:
        std::string address;

        void loadPointCloud(std::string address) {
            std::fstream inputFile;
            inputFile.open(address);

            std::string fileWord, fileLine;

            while(!inputFile.eof()) {
                getline(inputFile, fileLine);

                std::stringstream streamLine(fileLine);
                PointCloudT::Ptr pcl (new PointCloudT);
                int beamIdx = 0;

                while (getline(streamLine, fileWord, ' ')) {
                    if (fileWord != "S") {
        
                        double angle = (beamIdx - 660/2) * 0.006135923151543;
                        double range = std::stod(fileWord); // in milimeters
                        PointT point(cos(angle) * range, sin(angle) * range, 0);
                        pcl->points.push_back(point);

                        beamIdx++;
                    }
                }
                scans.push_back(pcl);
            }
        }
};

class DataGroundTruth {
    
    public:
        std::vector<PointT> points;

        DataGroundTruth(std::string address) {
            this->address = address;
            loadGroundTruth(address);
        }

        ~DataGroundTruth () {
            std::cout << "Data from " + address + " was removed." << std::endl;
        }
    
    private:
        std::string address;

        void loadGroundTruth(std::string address) {
            std::fstream inputFile;
            inputFile.open(address);

            std::string fileWord, fileLine;

            while (!inputFile.eof()) {
                getline(inputFile, fileLine);

                std::stringstream streamLine(fileLine);
                int idx = 0;
                PointT point;
                while (getline(streamLine, fileWord, ' ')) {
                    if (fileWord != "P") {
                        switch (idx) {
                            case 1: point.x = std::stof(fileWord);
                            case 2: point.y = std::stof(fileWord);
                            case 0: point.z = 0;
                        } 
                        idx++;
                    }
                }
                this->points.push_back(point);         
            }    
        }
};

class DataEncoder {

    public:
        // typedef int Tick[2];
        std::vector<Pose> poses;
        std::vector<Point> motorTicks;

        DataEncoder(std::string address) {
            this->address = address;
            loadEncoder(address);
        }

        ~DataEncoder () {
            std::cout << "Data from " + address + " was removed." << std::endl;
        }

    private:
        std::string address;

        void loadEncoder(std::string address) {
            std::fstream inputFile;
            inputFile.open(address);

            std::string fileWord, fileLine;
            
            
            double tick[2] = {0,0};
            double lastTick[2] = {0, 0};
            Pose pose;
            pose.position.x = 1850.0;
            pose.position.y = 1897.0;
            pose.rotation.yaw = 3.717551306747922;

            poses.push_back(pose);

            bool first = true;
            while(!inputFile.eof()) {
                getline(inputFile, fileLine);

                int idx = 0;
                std::stringstream streamLine(fileLine);
                while (getline(streamLine, fileWord, ' ')) {
                    
                    if (fileWord != "M" && (idx == 2 || idx == 6)) {
                        switch (idx) {
                            case 2: tick[0] = std::stod(fileWord);

                            case 6: tick[1] = std::stod(fileWord);
                        }
                    }
                    idx++;
                }

                if (first == true) {
                    lastTick[0] = tick[0];
                    lastTick[1] = tick[1];
                    first = false;
                }

                // Acho que tem que criar o vetor na memoria heap. Procurar sobre....
                Point deltaTick(tick[0] - lastTick[0], tick[1] - lastTick[1], 0);

                motorTicks.push_back(deltaTick);
                lastTick[0] = tick[0];
                lastTick[1] = tick[1];

                // odometer
                const double ticksTomm = 0.349;
                const double robotWidth = 173.0;
                double deltaYaw = (deltaTick.y - deltaTick.x) * ticksTomm / robotWidth;
                double deltaD = (deltaTick.x + deltaTick.y) * ticksTomm / 2;

                pose.position.x += deltaD * std::cos(pose.rotation.yaw);
                pose.position.y += deltaD * std::sin(pose.rotation.yaw);
                pose.rotation.yaw = fmod(pose.rotation.yaw + deltaYaw, 2 * M_PI);
                poses.push_back(pose);
                // std::cout << deltaTick.x << ", " << deltaTick.y << std::endl;
            }

            // for(int i = 0; i<200; i++) {
            //     std::cout << motorTicks[i][0] << ", " << motorTicks[i][1] << std::endl;
            // }
        }
};







/*
 Eigen::MatrixXd R(double phi, double theta, double psi) {
            double r00 = cos(psi) * cos(theta);
            double r01 = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
            double r02 = sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi);

            double r10 = sin(psi) * cos(theta);
            double r11 = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
            double r12 = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi);

            double r20 = -sin(theta);
            double r21 = cos(theta) * sin(phi);
            double r22 = cos(theta) * cos(phi);

            Eigen::MatrixXd R(3,3);
            R << r00, r01, r02,
                r10, r11, r12,
                r20, r21, r22;

            return R;
        }

*/