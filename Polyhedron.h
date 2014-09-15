/* 
 * File:   Polyhedron.h
 * Author: Jingru
 *
 * Created on November 28, 2011, 2:05 PM
 */

#ifndef POLYHEDRON_H
#define	POLYHEDRON_H

#include <math/matrix.h>
#include <math/vector.h>

/**
 * half space representation of polyhedron
 * a point is in this polyhedron if A*x >= b
 */
class Polyhedron
{
public:
    Polyhedron();
    Polyhedron(const Math::Matrix& M, const Math::Vector& V);
    Polyhedron(int, int, double*, double*);
    ~Polyhedron();
    
    void initialize(const Math::Matrix& M, const Math::Vector& V);

    /** check if a point is within the polyhedron */
    bool contains(const Math::Vector& x);
    /** check if this polyhedron is empty (both A and b are empty) */
    bool isEmpty();
        
    Math::Matrix A;
    Math::Vector b;
};

#endif	/* POLYHEDRON_H */

