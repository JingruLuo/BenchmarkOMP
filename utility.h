/* 
 * File:   utils.h
 * Author: Weiran
 *
 * Created on March 17, 2012, 4:16 PM
 */

#ifndef UTILITY_H
#define	UTILITY_H

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <time.h>

#include <math/matrix.h>
#include <math/vector.h>
#include <vector>

#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>

using namespace std;
using namespace Math;
namespace ob = ompl::base;

typedef Vector Config;

double elapsedTime();

/** create a directory by the name dir */
bool createDir(char* dir);

/** compute the path length */
double getSolutionPathLength(const vector<Vector> &path);

/** compute the distance of two vectors */
double normalize(int size, const Vector &v1, const Vector &v2);
double normalize(int size, const vector<double> &v1, const vector<double> &v2);

/** compute the determinanat of a matrix */
double determinantTriangleMatrix(const Math::Matrix &matrix);

/** convert ompl state to configuration */
void state_to_config(int n, const ob::ScopedState<>& state, Config& config);
void state_to_config(int n, const vector<double>& state, Config& config);


#endif	/* UTILITY_H */


