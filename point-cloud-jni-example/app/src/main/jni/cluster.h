#ifndef POINT_CLOUD_JNI_EXAMPLE_CLUSTER_H_
#define POINT_CLOUD_JNI_EXAMPLE_CLUSTER_H_

#include <vector>
#include <array>
#include <random>
#include <set>

template<typename Cluster_, typename DataType>
Cluster_ cluster(const std::vector<DataType>& data)
{

	Cluster_ cluster_v;
	std::vector<Eigen::Vector3f> tempPoints;

	for(int i; i<data.size(); i++){
		tempPoints.push_back(data[i]);
		if(data[i][1]>0 && data[i+1][1]<0){
			break;
		}
	}
	cluster_v.push_back(tempPoints);
	return cluster_v;
}

#endif