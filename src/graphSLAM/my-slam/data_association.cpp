/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "data_association.h"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <Eigen/Core>
//#include <Eigen/Dense>

using namespace std;
using namespace g2o;
using namespace Eigen;

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/core/sparse_optimizer.h"

//#include "g2o/stuff/unscented.h"

//typedef SigmaPoint<VectorXd> MySigmaPoint;

bool incDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance) {
    // parameters
    bool noAssociation = true;
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    OptimizableGraph::Vertex* currentPose = vc[poseIndex];
    set<HyperGraph::Edge*> edgeSetCurr = currentPose->edges();
    vector<int> associated;
    associated.clear();

    
    // for all landmarks observed in current pose
    for (set<HyperGraph::Edge*>::iterator it1 = edgeSetCurr.begin(); it1 != edgeSetCurr.end(); ++it1) {
        // Assume landmarks are second vertex in vertexContainer
        OptimizableGraph::Vertex* v1 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[1]);
        // tested array
        vector<int> tested;
        tested.clear();
        if (v1->dimension() == 2) {
            // for all landmarks of the past view
            for (int i = poseIndex-1; i>=0; --i) {
                OptimizableGraph::Vertex* pastPose = vc[i];
                set<HyperGraph::Edge*> edgeSetPast = pastPose->edges();
                for (set<HyperGraph::Edge*>::iterator it2 = edgeSetPast.begin(); it2 != edgeSetPast.end(); ++it2) {
                    // Assume landmarks are second vertex in vertexContainer
                    OptimizableGraph::Vertex* v2 = static_cast<OptimizableGraph::Vertex*> ((*it2)->vertices()[1]);
                    // v2 is landmark different from v1
                    if (v2->dimension() == 2 && v2->id() != v1->id()) {
                        // if v2 hasn't been tested
                        if (find(tested.begin(), tested.end(), v2->id()) == tested.end()) {
                            tested.push_back(v2->id());
                            // if v2 hasn't been associated
                            if (find(associated.begin(), associated.end(), v2->id()) == associated.end()) {                                                            

                                if (distanceTest(v1, v2, maxDistance)) {
                                    
                                    //bool band = correspondenceTest(optimizer, v1, v2, xi);

                                    if (xi <= 0 || correspondenceTest(optimizer, v1, v2, xi)) {
                                    //if (xi <= 0 || band) {
                                        // association successful
                                        optimizer.mergeVertices(v1, v2, false);
                                        noAssociation = false;
                                        associated.push_back(v2->id());                                                                                                                
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return noAssociation;
}


bool fullDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance) {
    // parameters
    bool noAssociation = true;    
    
    // for all past poses
    for (int i = poseIndex; i>=0; --i) {
	    // test association
        noAssociation = noAssociation && incDataAssociation (optimizer, i, xi, maxDistance);
    }
    return noAssociation;
}

int band = 0;

bool correspondenceTest (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi) {

    // compute marginal values from optimizer
    std::vector<std::pair<int, int> > blockIndices;
    blockIndices.push_back(make_pair(v1->hessianIndex(), v1->hessianIndex()));
    blockIndices.push_back(make_pair(v1->hessianIndex(), v2->hessianIndex()));
    blockIndices.push_back(make_pair(v2->hessianIndex(), v1->hessianIndex()));
    blockIndices.push_back(make_pair(v2->hessianIndex(), v2->hessianIndex()));
    
    SparseBlockMatrix<MatrixXd> spinv;
    optimizer.computeMarginals(spinv, blockIndices);
        
    // get marginal covariance and information matrix between pair of landmarks
    Matrix4d marginCovMat;
    marginCovMat.setZero();
    marginCovMat.topLeftCorner(2, 2) = *(spinv.block(v1->hessianIndex(), v1->hessianIndex()));
    marginCovMat.topRightCorner(2, 2) = *(spinv.block(v1->hessianIndex(), v2->hessianIndex()));
    marginCovMat.bottomLeftCorner(2, 2) = *(spinv.block(v2->hessianIndex(), v1->hessianIndex()));
    marginCovMat.bottomRightCorner(2, 2) = *(spinv.block(v2->hessianIndex(), v2->hessianIndex()));
    Matrix4d marginInfoMat = marginCovMat.inverse();

    // get landmark estimate vectors
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    Vector2d v1Est(v1EstVec.data());
    std::vector<double> v2EstVec;
    v2->getEstimateData(v2EstVec);
    Vector2d v2Est(v2EstVec.data());
    

    // get landmark difference estimation vector
    Vector2d diffEstVec = v1Est - v2Est;
 
    // get landmark difference information matrix
    Matrix<double, 4, 2> diffMat;
    diffMat << Matrix2d::Identity(2,2), -1*Matrix2d::Identity(2,2);
    Matrix2d diffInfoMat = diffMat.transpose() * marginInfoMat * diffMat;

    //Clayder's modifications
    // diffInfoMat(0, 0) = 11.0/12.0;
    // diffInfoMat(0, 1) = 1.0/12.0;
    // diffInfoMat(1, 0) = 1.0/12.0;
    // diffInfoMat(1, 1) = 11.0/12.0;

    // cout<<endl<<"Cov Matrix"<<endl;
    // cout<<diffInfoMat<<endl;
    // cout<<"---------------"<<endl;

    // EigenSolver<Matrix2d> es(diffInfoMat, true);

    // Vector2cd eigenValues = diffInfoMat.eigenvalues();
    // //Vector2cd eigenValues = es.eigenvalues();
    // Vector2cd eigenVector1 = es.eigenvectors().col(0);
    // Vector2cd eigenVector2 = es.eigenvectors().col(1);

    // cout<<endl<<"Eigen Values DDD"<<endl;
    // //cout<<eigenValues<<endl;
    // cout<<eigenValues(0).real()<<endl;
    // cout<<eigenValues(1).real()<<endl;
    // cout<<"---------------"<<endl;

    // cout<<endl<<"Eigen Vector 1"<<endl;
    // cout<<eigenVector1(0).real()<<endl;
    // cout<<eigenVector1(1).real()<<endl;
    // cout<<"---------------"<<endl;

    // cout<<endl<<"Eigen Vector 2"<<endl;
    // cout<<eigenVector2(0).real()<<endl;
    // cout<<eigenVector2(1).real()<<endl;
    // cout<<"---------------"<<endl;
    //end clayder mods

    //EigenSolver<Matrix2d> es(diffInfoMat.inverse(), true);    
    
    //Vector2cd eigenValues = diffInfoMat.inverse().eigenvalues();    
    //Vector2cd eigenVector1 = es.eigenvectors().col(0);
    //Vector2cd eigenVector2 = es.eigenvectors().col(1);

    //semiMajorAxis = sqrt(eigenValues(0).real()*10);
    //semiMinorAxis = sqrt(eigenValues(1).real()*10);
    //angle = atan2(eigenVector2(1).real(), eigenVector2(0).real())*180.0/M_PI;
    //cout<<"Ang:"<<angle<<endl;
    //cout<<"SMA:"<<semiMajorAxis<<endl;
    //cout<<"SmA:"<<semiMinorAxis<<endl;    

    // get likelihood
    double likelihood;
    likelihood = pow((2*M_PI * diffInfoMat.inverse()).determinant(), -0.5) * exp(-0.5 * diffEstVec.transpose() * diffInfoMat * diffEstVec);
    return likelihood > xi;
}

bool distanceTest(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double maxDistance) {
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    Vector2d v1Est(v1EstVec.data());
    std::vector<double> v2EstVec;
    v2->getEstimateData(v2EstVec);
    Vector2d v2Est(v2EstVec.data());
    return (v1Est - v2Est).norm() < maxDistance;
}