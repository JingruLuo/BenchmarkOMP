/*
 * Linkage.h
 *
 *  Created on: Jan 19, 2014
 *      Author: iuiml
 */

#ifndef LINKAGE_H_
#define LINKAGE_H_

#include <vector>
#include <math3d/primitives.h>
#include <planning/CSpace.h>
#include <robotics/Frame.h>
#include <ostream>

using namespace std;
using namespace Math3D;

/**
 * planar linkage class
 */
class Linkage {
public:
	/**
	 * @param q the initial configuration of the linkage
	 * @param link_length the length of each link
	 */
	Linkage( const Config& q, const Config& link_length);
	Linkage(int nD);
	virtual ~Linkage();
	/** update linkage configuration */
	void updateConfig( const Config& q);
	/** set link length of each  */
	void setLinkLength(const vector<double> &length);
	/** find the end point position of a given link  */
	Vector2 getWorldPos( const int link, const Vector2& pos_local);
	/** check if any self collision */
	bool selfCollision() const;
	/** print out end point positions of each link */
	void printJointPos(std::ostream &out);

private:
	/** linkage configuration */
	Config q;
	/** number of joints */
	int dim;
	/** link lengths of linkage  */
	Config link_length;
	/** local frame of each link */
	vector<Frame2D> frames;
	/** world frame of each link */
	vector<Frame2D> frames_world;
	/** end point positions of each link */
	vector<Vector2> pos_joints;

	/** compute the end point positions of each link */
	void evalJointPos();
};

#endif /* LINKAGE_H_ */
