#ifndef POINT_CLOUD_JNI_EXAMPLE_PLANE_MODEL_H_
#define POINT_CLOUD_JNI_EXAMPLE_PLANE_MODEL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <array>

struct PlaneModel
{
	static const int ModelSize=3;

	Eigen::Vector3f n;
	float d;
	//plane
	std::vector<Eigen::Vector3f> inliersV;
	//other point
	std::vector<Eigen::Vector3f> outliersV;
	float minX, maxX, minY, maxY, minZ, maxZ;


	void compute(const std::vector<Eigen::Vector3f>& data, const std::array<size_t,3>& indices)
	{
		Eigen::Vector3f a=data[indices[1]]-data[indices[0]];
		Eigen::Vector3f b=data[indices[2]]-data[indices[0]];
		n=a.cross(b).normalized();
		d=n.dot(data[indices[0]]);
	}
	int computeInliers(const std::vector<Eigen::Vector3f>& data, double threshold)
	{
		int inliers=0;
		for (size_t i=0;i<data.size();i++)
		{
			if (fabs(n.dot(data[i])-d)<threshold)
				inliers++;
		}
		//bestinliers = inliers;
		return inliers;
	}
	void refine(const std::vector<Eigen::Vector3f>& data, double threshold)
	{
		Eigen::Vector3f Ex=Eigen::Vector3f::Zero();
		Eigen::Matrix3f Exsqr=Eigen::Matrix3f::Zero();
		int inliers=0;
		minX = data[0][0];
		maxX = data[0][0];
		minY = data[0][1];
		maxY = data[0][1];
		minZ = data[0][2];
        maxZ = data[0][2];

		for (size_t i=0;i<data.size();i++)
		{
			if (fabs(n.dot(data[i])-d)<threshold)
			{
				inliers++;
				inliersV.push_back(data[i]);
				Ex+=data[i];
				Exsqr+=data[i]*data[i].transpose();
			}else if(data[i][2]<0.6){
				outliersV.push_back(data[i]);
				if(data[i][0]<minX){
					minX = data[i][0];
				}
				if(data[i][0]>maxX){
					maxX = data[i][0];
				}
				if(data[i][1]<minY){
					minY = data[i][1];
				}
				if(data[i][1]>maxY){
					maxY = data[i][1];
				}
				if(data[i][2]<minZ){
                	minZ = data[i][2];
                }
                if(data[i][2]>maxZ){
                	maxZ = data[i][2];
                }
			}

		}
		Ex/=inliers;
		Exsqr/=inliers;

		Eigen::Matrix3f cov=Exsqr-Ex*Ex.transpose();

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
		n=eig.eigenvectors().col(0);
		d=n.dot(Ex);
	}
};

#endif