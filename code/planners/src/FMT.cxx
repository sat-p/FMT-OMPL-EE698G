#include "../include/FMT.h"

/************************************************************************/

namespace PMR = ompl::geometric::EE698G;

/************************************************************************/

PMR::FMT::FMT (const base::SpaceInformationPtr &si) :
    ompl::base::Planner (si, "PMR EE698G"),
    sampler_            (si->allocStateSampler())
{
}

/************************************************************************/

PMR::FMT::~FMT (void)
{
    
}    

/************************************************************************/
/************************************************************************/

void PMR::FMT::setup (void)
{
    // Checking if the problem definition has been set
    if (!pdef_) {
    
        OMPL_ERROR ("%s : setup() called without problem definition"
                    " being set. Setup canceled", getName().c_str());
        setup_ = false;
        
        return;
    }
    
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else {
    
        OMPL_INFORM ("%s : Problem definition lacks optimization objective."
                     " Using path length.", getName().c_str());
        
        typedef ompl::base::PathLengthOptimizationObjective PLO;
        
        opt_ = std::make_shared<PLO> (si_);
        pdef_->setOptimizationObjective (opt_);
    }
    
    V_.setDistanceFunction ([this](const ompl::base::State* x1,
                                   const ompl::base::State* x2) {
                                return opt_->motionCost(x1, x2).value();
                           });
    
    // Checking if current state is valid.
    // Calls parent class's member function checkValidity();
    // Throws exception if the planner is in an invalid state.
    checkValidity();
    
    setup_ = true;
}

/************************************************************************/

void PMR::FMT::clear (void)
{
    
}

/************************************************************************/
/************************************************************************/

ompl::base::PlannerStatus
PMR::FMT::solve (const base::PlannerTerminationCondition &tc)
{
    typedef ompl::base::PlannerStatus PS;
    
    // Sampling N states from the free configuration space.
    sampleFree (tc);
    
    /********** Ensuring that there are states near the goals ***********/
    
    auto* goal = dynamic_cast<ompl::base::GoalSampleableRegion*> (
                                                pdef_->getGoal().get());
    
    // Checking if goal is valid
    if (!goal) {
        
        OMPL_ERROR ("%s: Invalid Goal", getName().c_str());
        return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    
    sampleGoal (goal);
    
    return PS (PS::StatusType::EXACT_SOLUTION);
}

/************************************************************************/
/************************************************************************/

void PMR::FMT::getPlannerData (ompl::base::PlannerData &data) const
{
    
}

/************************************************************************/
/************************************************************************/

void
PMR::FMT::sampleFree (const ompl::base::PlannerTerminationCondition &tc)
{
    V_unvisited_.clear();
    V_unvisited_.resize (numSamples_);
    
    unsigned attempts = 0;
    unsigned numSampled = 0;
    
    ompl::base::State* sample = si_->allocState();
    
    // Keep on sampling until the required number of samples haven't
    // been samples and the termination condition has not been met.
    while (numSampled < numSamples_ && !tc) {
        ++attempts;
        
        sampler_->sampleUniform  (sample);
        
        // Checking if the sampled point is valid
        if (si_->isValid (sample)) {
        
            ++numSampled;
            V_unvisited_.push_back (sample);
            sample = si_->allocState();
        }
    }
    
    V_.add (V_unvisited_);
    
    si_->freeState (sample);
}

/************************************************************************/

void PMR::FMT::sampleGoal (const ompl::base::GoalSampleableRegion* goal)
{
    const auto threshold = goal->getThreshold();
    
    // Ensuring that there a valid state near each goal.
    while (const ompl::base::State* goalState = pis_.nextGoal()) {
        
        ompl::base::State* nearest;
        V_.nearest (nearest);
        
        // if nearest neighbor is further than threshold
        if (opt_->motionCost(goalState, nearest).value() > threshold) {
            
            V_unvisited_.push_back (goalState);
            V_.add (goalState);
        }
    }
}

/************************************************************************/
/************************************************************************/