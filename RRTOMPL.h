/*********************************************************************
 *
 * RRTOMPL.h
 * implementation is from OMPL, modified for storing extra information
 *
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef RRTOMPL_H_
#define RRTOMPL_H_

#include "ompl/base/PlannerStatus.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "utility.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{

    namespace geometric
    {
		/** \brief Rapidly-exploring Random Trees */
		class RRTOMPL : public ob::Planner{
		public:

			/** \brief Constructor */
			RRTOMPL(const ob::SpaceInformationPtr &si);


			virtual ~RRTOMPL(void){
				freeMemory();
			}

			virtual void getPlannerData(ob::PlannerData &data) const;

			virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc);

			virtual void clear(void);

			/** \brief Set the goal bias

				In the process of randomly selecting states in
				the state space to attempt to go towards, the
				algorithm may in fact choose the actual goal state, if
				it knows it, with some probability. This probability
				is a real number between 0.0 and 1.0; its value should
				usually be around 0.05 and should not be too large. It
				is probably a good idea to use the default value. */
			void setGoalBias(double goalBias){
				goalBias_ = goalBias;
			}

			/** \brief Get the goal bias the planner is using */
			double getGoalBias(void) const{
				return goalBias_;
			}

			/** \brief Set the range the planner is supposed to use.

				This parameter greatly influences the runtime of the
				algorithm. It represents the maximum length of a
				motion to be added in the tree of motions. */
			void setRange(double distance){
				maxDistance_ = distance;
			}

			/** \brief Get the range the planner is using */
			double getRange(void) const{
				return maxDistance_;
			}

			/** \brief get collision checking time */
			double getCCTime(){
				return this->t_cc_;
			}

			/** \brief get iteration counter */
			unsigned int getIterationCounter(){
					return iterationCounter_;
			}

			/** \brief Set a different nearest neighbors datastructure */
			template<template<typename T> class NN>
			void setNearestNeighbors(void){
				nn_.reset(new NN<Motion*>());
			}

			virtual void setup(void);

			/** best path found so far */
			og::PathGeometric*								   best_path;

		//            void set_bestpath(const PathGeometric* path);
		protected:


			/** \brief Representation of a motion

				This only contains pointers to parent motions as we
				only need to go backwards in the tree. */
			class Motion{
			public:

				Motion(void) : state(NULL), parent(NULL){
				}

				/** \brief Constructor that allocates memory for the state */
				Motion(const ob::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL){
				}

				~Motion(void){
				}

				/** \brief The state contained by the motion */
				ob::State       *state;

				/** \brief The parent motion in the exploration tree */
				Motion            *parent;

			};

			/** \brief Free the memory allocated by this planner */
			void freeMemory(void);

			/** \brief Compute distance between motions (actually distance between contained states) */
			double distanceFunction(const Motion* a, const Motion* b) const{
				return si_->distance(a->state, b->state);
			}

			/** \brief State sampler */
			ob::StateSamplerPtr                          sampler_;

			/** \brief A nearest-neighbors datastructure containing the tree of motions */
			boost::shared_ptr< ompl::NearestNeighbors<Motion*> > nn_;

			/** \brief The/ fraction of time the goal is picked as the state to expand towards (if such a state is available) */
			double                                         goalBias_;

			/** \brief The maximum length of a motion to be added to a tree */
			double                                         maxDistance_;

			/** \brief The random number generator */
			ompl::RNG                                            rng_;

			/** number of iteration so far */
			unsigned int											   iterationCounter_;

			/** collision checking time */
			double 										   t_cc_;
		};
    }
}

#endif

