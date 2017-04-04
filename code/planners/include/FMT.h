#ifndef __FMT_OMPL_EE698G__
#define __FMT_OMPL_EE698G__

/************************************************************************/

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <vector>

/************************************************************************/

namespace ompl {
namespace geometric {

namespace EE698G {

class FMT : public ompl::base::Planner
{
public:
    FMT (const base::SpaceInformationPtr &si);
    
    ~FMT (void) override;
    
public:
    void setup (void) override;
    void clear (void) override;
    
public:
    base::PlannerStatus
    solve (const ompl::base::PlannerTerminationCondition &tc) override;
    
public:
    void getPlannerData (ompl::base::PlannerData &data) const override;

public:
    void setNumSamples (const unsigned N)
    { numSamples_ = N; }
    
    unsigned getNumSamples (void) const
    { return numSamples_; }
    
    void setDistMultiplier (const double m)
    { distMultiplier_ = m; }
    
    double getDistMultiplier (void) const
    { return distMultiplier_; }
    
protected:
    /*
     * Samples the start states
     */
    void sampleStart (void);
    
    /*
     * Samples N states from the free configuration space while
     * termination condition is not met.
     */
    void sampleFree (const ompl::base::PlannerTerminationCondition &tc);
    
    /*
     * Samples states near the goals if absent.
     */
    void sampleGoal (const ompl::base::GoalSampleableRegion* goal);

protected:
    double unitBallVolume (const unsigned dim) const;
    
    double freeVolume (const unsigned attempts,
                       const unsigned samples) const;
    
    double neighborDistance (void) const;

protected:
    /*
     * The sampler of the provided state space.
     */
    ompl::base::StateSamplerPtr sampler_;

    /*
     * The optimization objective object
     */
    ompl::base::OptimizationObjectivePtr opt_;
    
    /*
     * The object used to perform the Near() query.
     */
    ompl::NearestNeighborsGNAT<const ompl::base::State*> V_;
    
protected:
    unsigned numSamples_;
    double   distMultiplier_ {1.0};
    
protected:
    std::vector<const ompl::base::State*> V_unvisited_;
    std::vector<const ompl::base::State*> V_open_;
    std::vector<const ompl::base::State*> V_closed_;

protected:
    double mu_free_; // The free space volume
    double r_n_; // The distance threshold for neighbors
}; // class FMT
    
}; // namespace EE698G
    
}}; // namespaces ompl, geometric

/************************************************************************/
/************************************************************************/

#endif