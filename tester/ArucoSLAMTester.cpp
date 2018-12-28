//
// Created by steve on 12/28/18.
//

#include <opencv2/aruco.hpp>

#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>


int main(){

// Add prior on the first key
//	gtsam::NonlinearFactorGraph graphWithPrior = *graph;
	gtsam::noiseModel::Diagonal::shared_ptr priorModel = //
			gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());




}