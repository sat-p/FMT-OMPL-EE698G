#include "../include/FMT.h"

#include <cmath>

/************************************************************************/

namespace PMR = ompl::geometric::EE698G;

/************************************************************************/

static constexpr double pi = 3.14159265359;

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
    // Checking if start states are available
    if (!pis_.haveMoreStartStates()) {
    
        OMPL_ERROR ("%s: Invalid Start", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }
    
    /*
     * Adding start states to V_ and V_open_
     */
    
    sampleStart();
    
    /*
     * Sampling N states from the free configuration space.
     */
    
    sampleFree (tc);
    
    /*
     * Ensuring that there are states near the goals
     */
    
    auto* goal = dynamic_cast<ompl::base::GoalSampleableRegion*> (
                                                pdef_->getGoal().get());
    
    // Checking if goal is valid
    if (!goal) {
        
        OMPL_ERROR ("%s: Invalid Goal", getName().c_str());
        return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    
    sampleGoal (goal);
    
    /*
     * The main loop
     */
    
    V_closed_.clear();
    r_n_ = neighborDistance();
    
    
    typedef ompl::base::PlannerStatus PS;
    return PS (PS::StatusType::EXACT_SOLUTION);
}

/************************************************************************/
/************************************************************************/

void PMR::FMT::getPlannerData (ompl::base::PlannerData &data) const
{
    
}

/************************************************************************/
/************************************************************************/

void PMR::FMT::sampleStart (void)
{
    V_open_.clear();
    V_.clear();
    
    while (const ompl::base::State* start = pis_.nextStart()) {
    
        V_.add (start);
        V_open_.push_back (start);
    }
}

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
    
    // Setting the best estimate of the free Volume.
    mu_free_ = freeVolume (attempts, numSampled);
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

double PMR::FMT::unitBallVolume (const unsigned dim) const
{
    if (!dim)
        return 1.0;
    else if (dim == 1)
        return 2.0;
    else
        return 2 * pi * unitBallVolume (dim - 2) / dim;
}

/************************************************************************/

double PMR::FMT::freeVolume
(const unsigned attempts, const unsigned samples) const
{
    return  (si_->getSpaceMeasure() / attempts) * samples;
}

/************************************************************************/

double PMR::FMT::neighborDistance (void) const
{
    const double d = si_->getStateDimension();
    const double d_inv = 1 / d;
    
    const double ballVolume = unitBallVolume (d);
    
    const double gamma = 2 * std::pow (d_inv * mu_free_ / ballVolume,
                                       d_inv);
    
    const unsigned& n = numSamples_;
    
    return distMultiplier_ * gamma *
           std::pow (std::log (static_cast <double> (n)) / n,
                     d_inv);
}

/************************************************************************/
/************************************************************************/