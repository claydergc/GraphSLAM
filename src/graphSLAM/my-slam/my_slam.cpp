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

/** @file 
 * GraphSLAM algorithm main file 
 */ 

#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"

#include "g2o/core/sparse_optimizer_terminate_action.h"

#include <typeinfo>
#include <ctime>

#include "data_association.h"
#include "slam_functs.h"

#include <map>
#include <vector>
#include <cmath>
#include <iostream>
#include <pthread.h>

#include "gnuplot-iostream.h"

using namespace std;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

// functions
void incrementalOptimization (SparseOptimizer& optimizer, int nPoses, double xi, int maxIterations, double maxDistance, int poseSkip, int interOpt);
void fullOptimization (SparseOptimizer& optimizer, int pose, double xi, int maxIterations, double maxDistance);

vector<pair<float, float> > path;
vector<float> angles;
vector<pair<float, float> > landmarks;
vector<Vector2d> gt;
Gnuplot gp;

int main(int argc, char** argv) {

    // clock for time measurement
    clock_t begin = clock();

    // Command line parsing
    int iteration = 0;
    int maxIterations;
    double xi;
    string robustKernel;
    double huberWidth;
    bool listKernelsBool;
    bool nonSequential;
    string outputFilename;
    string inputFilename;
    int poseSkip;
    int interOpt;
    double disTest;
    CommandArgs arg;

    arg.param("i", maxIterations, 10, "perform n iterations, if negative consider the gain");
    arg.param("t", xi, 1.0, "threshold for data association");
    arg.param("robustKernel", robustKernel, "", "use this robust error function");
    arg.param("robustKernelWidth", huberWidth, -1., "width for the robust Kernel (only if robustKernel)");
    arg.param("listRobustKernels", listKernelsBool, false, "list the registered robust kernels");
    arg.param("nonSequential", nonSequential, false, "apply the robust kernel only on loop closures and not odometries");
    arg.param("o", outputFilename, "", "output final version of the graph");
    arg.param("poseSkip", poseSkip, 1, "Optimization step");
    arg.param("interOpt", interOpt, 100000, "Inter optimization step");
    arg.param("disTest", disTest, 0, "max distant for association, used only if distTest > 0");
    arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
    arg.parseArgs(argc, argv);
    
    // list robust kernel
    listRobustKernels (listKernelsBool);

    // create the linear solver
    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();

    // create the block solver on top of the linear solver
    BlockSolverX* blockSolver = new BlockSolverX(linearSolver);

    // create the algorithm to carry out the optimization
    //OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

    // NOTE: We skip to fix a variable here, either this is stored in the file
    // itself or Levenberg will handle it.

    // create the optimizer to load the data and carry out the optimization
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(optimizationAlgorithm);

    // read data file and load to optimizer

    ifstream ifs;
    readDataFile (ifs, inputFilename);
    optimizer.load(ifs);
    ifs.close();

    printf("Input File=%s\n", inputFilename.c_str());

    // load robust kernel
    loadRobustKernel (robustKernel, nonSequential,  huberWidth, optimizer);
    
    // initial guess
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);
    // end initial guess
    
    // get pose vertices
    OptimizableGraph::VertexContainer poses;
    getAllPoses (optimizer, poses);
   
    // compute max distance
    double maxDistance;
    if (disTest > 0) maxDistance = disTest;
    else maxDistance = getMaxDistance (xi);
    cout << "Max Distance: " << maxDistance << endl;

    //gp << "set xrange [-5:22]\nset yrange [-10:15]\n";
    gp << "set xrange [-5:30]\nset yrange [-15:15]\n";
    //gp << "set xrange [-60:100]\nset yrange [-100:100]\n";

    //gt.push_back(Vector2d(0,-5));
    //gt.push_back(Vector2d(0,5));
    //gt.push_back(Vector2d(10,-5));
    //gt.push_back(Vector2d(10,5));
    //gt.push_back(Vector2d(20,-5));
    //gt.push_back(Vector2d(20,5));

    gt.push_back(Vector2d(0,-5));
    gt.push_back(Vector2d(7,-5));
    gt.push_back(Vector2d(10,-10));
    gt.push_back(Vector2d(20,-7));
    gt.push_back(Vector2d(0,5));
    gt.push_back(Vector2d(7,8));
    gt.push_back(Vector2d(10,10));
    gt.push_back(Vector2d(20,3));
    
    // optimization
    incrementalOptimization (optimizer, poses.size(), xi, maxIterations, maxDistance, poseSkip, interOpt);
    
    // write data
    writeDataFile (outputFilename, optimizer);
    
    // compute time
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Elapsed time: " << elapsed_secs << " [s]" << endl;
    
    return 0;
}

vector<VectorXd> vectorEllipses;
VectorXd ellipseRobot(5);
float trx1 = -0.25;
float try1 = -0.25;
float trx2 = -0.25;
float try2 = 0.25;
float trx3 = 0.25;
float try3 = 0.0;
float tx1, tx2, tx3, ty1, ty2, ty3;
    
void incrementalOptimization (SparseOptimizer& optimizer, int nPoses, double xi, int maxIterations, double maxDistance, int poseSkip, int interOpt){
    /**
     * Performs incremetal optimization for solving the SLAM problem.
     * 
     * Iteratively performs incremental data association, optimization, and full optimization over the SLAM graph.
     * @param optimizer SparseOptimizer object that contains the SLAM graph
     * @param nPoses Number of poses of the robot path to optimize
     * @param xi Threshold for the correspondence test
     * @param maxIteration Maximum number of iteration for the optimization algorithm
     * @param maxDistance Maximum distance allowed between landmarks to produce an association
     * @param poseSkip Number of poses between optimization (1: no skkiping)
     * @param interOpt Number of poses between full optimization (1: full optimization at every pose)
     */
    

    for (int i=0; i<nPoses; ++i) {
    //for (int i=0; i<750; ++i) {
    
        cout << "\n### iteration pose " << i << " (incremental) ###" << endl;
        
        // data association
        cerr << "Testing associations ...";
        bool no_association = incDataAssociation(optimizer, i, xi, maxDistance);        
        cerr << " done." << endl;    

        // optimize
        if (i % poseSkip == 0 && !no_association) {
            optimizer.initializeOptimization();
            optimizer.optimize(maxIterations);
            getLandmarksEllipses(optimizer, i, vectorEllipses);
            getRobotEllipse(optimizer, i, ellipseRobot);

            //Agregado por Clayder                        
            VectorXd v = ellipseRobot;
            gp << "set obj 888 ellipse center "<<v[0]<<","<<v[1]<<" size "<<v[2]<<", "<<v[3]<<" angle "<<v[4]<<" fs empty border rgb '#24E711'"<<endl;
            
            
            for(int j=0; j<vectorEllipses.size(); j++)
            {
                VectorXd v = vectorEllipses[j];

                for(int k=0; k<gt.size(); k++)
                {
                    Vector2d v1 = gt[k];
                    Vector2d v2(v[0], v[1]);
                    
                    if ( (v1 - v2).norm()<1 )
                        gp << "set obj "<<(k+1)<< " ellipse center "<<v[0]<<","<<v[1]<<" size "<<v[2]<<", "<<v[3]<<" angle "<<v[4]<<" fs empty border rgb '#FE0000'"<<endl;
                }

                //gp << "set obj 1 ellipse center "<<v[0]<<","<<v[1]<<" size "<<v[2]<<", "<<v[3]<<" angle "<<v[4]<<" front fs empty bo 3"<<endl;
                //gp << "set obj "<<(j+1)<< " ellipse center "<<v[0]<<","<<v[1]<<" size "<<v[2]<<", "<<v[3]<<" angle "<<v[4]<<" front fs empty bo rgb '#FF000F'"<<endl;
            }
            
            getAllPoses (optimizer, path, angles, landmarks);

            //cout<<"PI: "<<M_PI<<endl;

            tx1 = path[i].first + trx1 + trx1*cos(angles[i]) - try1*sin(angles[i]);
            ty1 = path[i].second + try1 + trx1*sin(angles[i]) + try1*cos(angles[i]);
            tx2 = path[i].first + trx2 + trx2*cos(angles[i]) - try2*sin(angles[i]);
            ty2 = path[i].second + try2 + trx2*sin(angles[i]) + try2*cos(angles[i]);
            tx3 = path[i].first + trx3 + trx3*cos(angles[i]) - try3*sin(angles[i]);
            ty3 = path[i].second + try3 + trx3*sin(angles[i]) + try3*cos(angles[i]);
            

            gp << "set object 777 poly from "<<tx1<<","<<ty1<<" to "<<tx2<<","<<ty2<<" to "<<tx3<<","<<ty3<<" to "<<tx1<<","<<ty1<<" fs empty border 12"<<endl;            
            gp << "plot" << gp.file1d(path) << "with lines title 'robot path',"<< gp.file1d(landmarks) << "with points title 'landmarks'"<<endl;

            path.clear();
            angles.clear();
            landmarks.clear();

            //agregado por clayder
            vectorEllipses.clear();
            
            usleep(100000);
        }
        
        // inter optimize
        if (i % interOpt == 0) fullOptimization (optimizer, i, xi, maxIterations, maxDistance);        
    }


}

  
void fullOptimization (SparseOptimizer& optimizer, int pose, double xi, int maxIterations, double maxDistance) {
    /**
     * Performs full optimization for solving the SLAM problem.
     * 
     * Performs the full optimization algorithm over the whole SLAM graph until no more
     * association are found.
     * @param optimizer SparseOptimizer object that contains the SLAM graph
     * @param pose index at which consider the end of the graph (used to shorten the graph to optimize)
     * @param xi Threshold for the correspondence test
     * @param maxIteration Maximum number of iteration for the optimization algorithm
     * @param maxDistance Maximum distance allowed between landmarks to produce an association
     */

    double xC, yC, major, minor, angle;
    int i = 0;
    cout << "\n### Full Optimization ###\n" << endl;
    while(true) {

        //vectorEllipses.clear();
        // data association
        cout << "\n### iteration " << i++ << " (full) ###\n" << endl;
        cerr << "Testing associations ...";
        bool no_more_association = fullDataAssociation(optimizer, pose, xi, maxDistance);
        cerr << " done." << endl;

        // finish test
        if (no_more_association) break;

        // optimize
        optimizer.initializeOptimization();
        optimizer.optimize(maxIterations);

        //Agregado por Clayder 
        cout<<"***Clayder print 2****"<<endl;
        //OptimizableGraph::VertexContainer poses;
        //getAllPoses (optimizer, poses);

        for(int j=0; j<vectorEllipses.size(); j++)
        {
            VectorXd v = vectorEllipses[j];
            gp << "set obj "<<(j+1)<< " ellipse center "<<v[0]<<","<<v[1]<<" size "<<v[2]<<", "<<v[3]<<" angle "<<v[4]<<" front fs empty bo 3"<<endl;
        }

        path.clear();
        getAllPoses (optimizer, path, angles, landmarks);
        //gp << "plot" << gp.file1d(path) << "with lines title 'robot path'"<<endl;
        gp << "plot" << gp.file1d(path) << "with lines title 'robot path',"<< gp.file1d(landmarks) << "with points title 'landmarks'"<<endl;
        
        sleep(0.5);
    }
}
