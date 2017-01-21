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

#include "slam_functs.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::istringstream
#include <vector>

#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

void listRobustKernels (bool list) {
    if (list) {
        std::vector<std::string> kernels;
        RobustKernelFactory::instance()->fillKnownKernels(kernels);
        cout << "Robust Kernels:" << endl;
        for (size_t i = 0; i < kernels.size(); ++i) {
            cout << kernels[i] << endl;
        }
    }
}

void readDataFile (ifstream &ifs, string filename) {
    ifs.open(filename.c_str());
    if (! ifs) {
        cerr << "unable to open " << filename << endl;
    }
}

void writeDataFile (string outputFilename, SparseOptimizer &optimizer) {
    if (outputFilename.size() > 0) {
        if (outputFilename == "-") {
            cerr << "saving to stdout";
            optimizer.save(cout);
        } else {
            cerr << "saving " << outputFilename << " ... ";
            optimizer.save(outputFilename.c_str());
        }
        cerr << "done." << endl;
    }
}

void loadRobustKernel (string robustKernel, bool nonSequential, double huberWidth, SparseOptimizer &optimizer) {
    if (robustKernel.size() > 0) {
        AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(robustKernel);
        cerr << "# Preparing robust error function ... ";
        if (creator) {
            if (nonSequential) {
                for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
                    SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
                    if (e->vertices().size() >= 2 && std::abs(e->vertex(0)->id() - e->vertex(1)->id()) != 1) {
                        e->setRobustKernel(creator->construct());
                        if (huberWidth > 0)
                            e->robustKernel()->setDelta(huberWidth);
                    }
                }
            } else {
                for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
                    SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
                    e->setRobustKernel(creator->construct());
                    if (huberWidth > 0)
                        e->robustKernel()->setDelta(huberWidth);
                }
            }
            cerr << "done." << endl;
        } else {
            cerr << "Unknown Robust Kernel: " << robustKernel << endl;
        }
    }
}


void getAllPoses (SparseOptimizer &optimizer, OptimizableGraph::VertexContainer &poses) {
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();


    for (size_t i=0; i<vc.size(); ++i) {
        if (vc[i]->dimension() == 3) {
            poses.push_back(vc[i]);            
        }
    }
}


void getAllPoses (SparseOptimizer &optimizer, vector<pair<float, float>> &path, vector<float> &angles, vector<pair<float, float>> &landMarks) {
    
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    stringstream sstr;
    stringstream sstrL;
    float x, y, t;
    float lx, ly;    

    for (size_t i=0; i<vc.size(); ++i) {
        if (vc[i]->dimension() == 3) {
            std::vector<double> vEstVec;
            vc[i]->getEstimateData(vEstVec);            
            path.push_back(make_pair(vEstVec[0], vEstVec[1]));
            angles.push_back(vEstVec[2]);
        }

        if (vc[i]->dimension() == 2) {
            std::vector<double> vEstVec;
            vc[i]->getEstimateData(vEstVec);            
            landMarks.push_back(make_pair(vEstVec[0], vEstVec[1]));   
        }
    }
}


double getMaxDistance (double xi) {
    return 1/(sqrt(2*M_PI*M_E) * xi);
}

void getLandmarksEllipses (SparseOptimizer& optimizer, int poseIndex, vector<VectorXd> &vectorEllipses) {
        

    // parameters
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    OptimizableGraph::Vertex* v1 = vc[poseIndex];
    set<HyperGraph::Edge*> edgeSetCurr = v1->edges();

    VectorXd ellipseData(5);
    
    double xC, yC, major, minor, angle;    
    
    // for all landmarks observed in current pose
    for (set<HyperGraph::Edge*>::iterator it1 = edgeSetCurr.begin(); it1 != edgeSetCurr.end(); ++it1) {
        // Assume landmarks are second vertex in vertexContainer
        
        cout<<"size:"<<(*it1)->vertices().size()<<endl;

        //for(int i=0; i<(*it1)->vertices().size(); i++)
        //{
        OptimizableGraph::Vertex* v2 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[1]);
        //OptimizableGraph::Vertex* v2 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[i]);


        if (v2->dimension() == 2) {
            getEllipsesData(optimizer, v2, major, minor, angle);
            std::vector<double> v1EstVec;
            v2->getEstimateData(v1EstVec);
            xC = v1EstVec[0];
            yC = v1EstVec[1];

            ellipseData(0) = xC;
            ellipseData(1) = yC;
            ellipseData(2) = major;
            ellipseData(3) = minor;
            ellipseData(4) = angle;
            vectorEllipses.push_back(ellipseData);
        }
        //}
        
    }    
}

void getRobotEllipse (SparseOptimizer& optimizer, int poseIndex, VectorXd &ellipseData)
{
        
    // parameters
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    OptimizableGraph::Vertex* v1 = vc[poseIndex];
    set<HyperGraph::Edge*> edgeSetCurr = v1->edges();
    
    double xC, yC, major, minor, angle;    
    
    getEllipsesData(optimizer, v1, major, minor, angle);
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    xC = v1EstVec[0];
    yC = v1EstVec[1];

    ellipseData(0) = xC;
    ellipseData(1) = yC;
    ellipseData(2) = major;
    ellipseData(3) = minor;
    ellipseData(4) = angle;        
}

void getEllipsesData (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, double &semiMajorAxis, double &semiMinorAxis, double &angle) {

    // compute marginal values from optimizer
    std::vector<std::pair<int, int> > blockIndices;
    blockIndices.push_back(make_pair(v1->hessianIndex(), v1->hessianIndex()));    

    SparseBlockMatrix<MatrixXd> spinv;
    optimizer.computeMarginals(spinv, blockIndices);
    
    Matrix2d marginCovMat;
    marginCovMat.setZero();
    marginCovMat << *(spinv.block(v1->hessianIndex(), v1->hessianIndex()));
    
    cout<<endl<< "covariance" <<endl;
    cout << marginCovMat<< endl;
    cout<<"-----------------"<<endl;
                   
    EigenSolver<Matrix2d> es(marginCovMat, true);
    Vector2cd eigenValues = marginCovMat.eigenvalues()*15;    
    Vector2cd eigenVector1 = es.eigenvectors().col(0);
    Vector2cd eigenVector2 = es.eigenvectors().col(1);

    semiMajorAxis = sqrt(eigenValues(0).real());
    semiMinorAxis = sqrt(eigenValues(1).real());
    angle = atan2(eigenVector2(1).real(), eigenVector2(0).real())*180.0/M_PI;
    //angle = atan(eigenVector2(1).real()/eigenVector2(0).real())*180.0/M_PI;
    cout<<"Ang:"<<angle<<endl;
    cout<<"SMA:"<<semiMajorAxis<<endl;
    cout<<"SmA:"<<semiMinorAxis<<endl;    
}