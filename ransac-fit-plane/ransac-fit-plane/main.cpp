//
//  main.cpp
//  ransac-fit-plane
//
//  Created by Supannee Tanathong on 30/10/2024.
//
//  Objective: Fit 3D points to plane and return plane parameters
//             Ax + By + Cz + D = 0

#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <numeric>
#include <ctime>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Dense>

struct Plane {
    // Ax + By + Cz + D = 0
    double A;
    double B;
    double C;
    double D;
    Plane() : A(0), B(0), C(0), D(0) {}
    Plane(double A, double B, double C, double D) : A(A), B(B), C(C), D(D) {}
    Eigen::Vector3d getUnitNormalVector() {
        Eigen::Vector3d normal(A, B, C);
        normal.normalize();
        return normal;
    }
};

struct Parameters {
    // prob: the desired probability that the RANSAC algorithm provides
    //       at least one useful result after running
    double prob = 0.99;
    
    // maxIterations: maximum number of iterations
    int maxIterations = 100;
    
    // sampleSize: Minimum number of points to fit the model
    int sampleSize = 3;
    
    // thresholdFittingModel: defined as distance from point to plane
    double thresholdFittingModel = 0.5;
};

Plane fitPlane(const std::vector<Eigen::Vector3d>& points3d) {
    // 1. Compute 2 vectors that lie on plane
    Eigen::Vector3d v1 = points3d[0] - points3d[1];
    Eigen::Vector3d v2 = points3d[1] - points3d[2];
    
    // 2. Compute normal vector perpendicular to the plane
    Eigen::Vector3d normal = v1.cross(v2);
    
    // 3. Compute D as
    //    Ax1 + By1 + Cz1 + D = 0
    //    D = -(Ax1 + By1 + Cz1)
    double D = -normal.dot(points3d[0]);
    
    // 4. Return result
    Plane plane;
    plane.A = normal(0);
    plane.B = normal(1);
    plane.C = normal(2);
    plane.D = D;
    
    return plane;
}

double computeDistanceToPlane(const Plane& plane, const Eigen::Vector3d &point3d) {
    /*
      distance(point3D, plane) = |A(x) + B(y) + C(z) + D|
                                 ------------------------
                                   sqrt(A*A + B*B + C*C)
     */
    
    Eigen::Vector3d planeNormal(plane.A, plane.B, plane.C);
    double norm = planeNormal.norm();
    if (norm == 0)
        return std::numeric_limits<double>::max();
    
    double nominator = std::fabs(planeNormal.dot(point3d) + plane.D);
    return nominator/norm;
}

// Make sure that selected index are unique
std::unordered_set<int> randomNPoints(std::mt19937& rng, std::uniform_int_distribution<>& selector, int numSamples) {
    std::unordered_set<int> randomIndices;
    
    int N = 20; // maximum times to select points
    while (N-- > 0 && randomIndices.size() < numSamples) {
        randomIndices.insert(selector(rng));
    }
    
    // At this point, we we can't reach numSamples, simply fill the resulting set
    while (randomIndices.size() < numSamples) {
        randomIndices.insert(randomIndices.size()); // Just any number, here we just using the size of random index
    }
    
    return randomIndices;
}

std::vector<Eigen::Vector3d> pickRandomPoints(const std::vector<Eigen::Vector3d>& points, std::mt19937& rng, std::uniform_int_distribution<>& selector, int numSamples) {
    // Random indexes
    std::unordered_set<int> indices = randomNPoints(rng, selector, numSamples);
    
    // Add selected points to array
    std::vector<Eigen::Vector3d> selectedPoints;
    for (auto idx : indices) {
        selectedPoints.emplace_back(points[idx]);
    }
    return selectedPoints;
}

int recomputeIterations(const Parameters& params, int numInliers, int totalPoints) {
    const double epsilon = 1e-8;
    
    double inlierRatio = (double)numInliers/totalPoints;
    if (inlierRatio < epsilon)
        return std::numeric_limits<double>::max();
    
    double probAllInliers = std::pow(inlierRatio, params.sampleSize);
    double probAllOutliers = std::max(epsilon, 1.0 - probAllInliers);
    double k = log(1.0 - params.prob) / log(probAllOutliers);
    return k;
}

Plane ransacFitPlane(const std::vector<Eigen::Vector3d>& points, const Parameters& params, std::mt19937& rng) {
    Plane bestModel;
    int bestNumInliers = 0;
    int maxIters = params.maxIterations;
    
    // To randomly pick 3 points
    std::uniform_int_distribution<> selector(0, points.size()-1); // [0, num_points-1]
    
    for (int k = 0; k < std::min(maxIters, params.maxIterations); ++k) {
        std::vector<Eigen::Vector3d> selectedPoints = pickRandomPoints(points, rng, selector, params.sampleSize);
        
        // Fit the plane model to the random points
        Plane plane = fitPlane(selectedPoints);
        
        // Count the number of inliers
        int numInliers = 0;
        for (const Eigen::Vector3d& point : points) {
            if (computeDistanceToPlane(plane, point) < params.thresholdFittingModel) {
                ++numInliers;
            }
        }
        
        // Update the best plane model
        if (bestNumInliers < numInliers) {
            bestModel = plane;
            bestNumInliers = numInliers;
            
            // Update the number of iterations
            maxIters = recomputeIterations(params, bestNumInliers, points.size());
        }
    }
    
    return bestModel;
}


std::vector<Eigen::Vector3d> generatePointsOnPlane(const Plane& plane, std::mt19937& rng, int N = 100, double outlierRatio = 0.1) {
    // Randomly select outlier indexes
    int numOutliers = outlierRatio * N;
    std::uniform_int_distribution<> outlierSelector(0, N);
    std::unordered_set<int> outliers = randomNPoints(rng, outlierSelector, numOutliers);
    
    std::uniform_real_distribution<> generator(0.0, 20.0);
    std::uniform_real_distribution<> smallNoise(0.0, 0.05);
    std::uniform_real_distribution<> largeNoise(0.0, 5.0);
    
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < N; ++i) {
        // Ax + By + Cz + D = 0
        // z = -(Ax + By + D)/C
        double x = generator(rng);
        double y = generator(rng);
        double z = -(plane.A * x + plane.B * y + plane.D) / plane.C;
        
        // Add large noise if i is an outlier
        if (outliers.count(i)) {
            x += largeNoise(rng);
            y += largeNoise(rng);
            z += largeNoise(rng);
        } else {
            // Add small noise if it's inliers
            x += smallNoise(rng);
            y += smallNoise(rng);
            z += smallNoise(rng);
        }
        points.emplace_back(x, y, z);
    }
    return points;
}

int main(int argc, const char * argv[]) {
    // Define plane: -1.5x + 2.0y + 3.5z - 6.0 = 0
    Plane plane(-1.5, 2.0, 3.5, 6.0);
    
    // Randomly generated 100 points;
    std::mt19937 rng(std::time(nullptr));
    int N = 100;
    double ratioOutlers = 0.1;
    std::vector<Eigen::Vector3d> points = generatePointsOnPlane(plane, rng, N, ratioOutlers);
    
    // Fit points to plane using RANSAC
    Parameters params;
    Plane estimatedPlane = ransacFitPlane(points, params, rng);
    
    std::cout << "Original plane equation: \n\t" <<
        plane.A << "x + " << plane.B << "y + " << plane.C << "z + " << plane.D << std::endl;
    std::cout << "\t with unit vector: " << plane.getUnitNormalVector().transpose() << std::endl;
    
    std::cout << "Estimated plane equation from " << N << " points with " << ratioOutlers*100 << "% outliers: \n\t" <<
        estimatedPlane.A << "x + " << estimatedPlane.B << "y + " << estimatedPlane.C << "z + " << estimatedPlane.D << std::endl;
    std::cout << "\t with unit vector: " << estimatedPlane.getUnitNormalVector().transpose() << std::endl;
    
    return 0;
}
