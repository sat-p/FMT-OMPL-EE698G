/************************************************************************/

#ifndef __FMT_OMPL_EE698G__
#define __FMT_OMPL_EE698G__

/************************************************************************/

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <vector>
#include <unordered_map>
#include <queue>
#include <utility>
#include <numeric>

/************************************************************************/

namespace ompl {
namespace geometric {

namespace EE698G {

/************************************************************************/
    
typedef std::vector<const ompl::base::State*> stateVector;

/************************************************************************/

enum FMT_SetType
{
    OPEN,
    UNVISITED,
    CLOSED
};

/************************************************************************/
/************************************************************************/

struct FMT_AuxData
{
public:
    FMT_AuxData (FMT_SetType _setType,
                 const double _cost = std::numeric_limits<double>::max()) :
        setType     (_setType),
        cost        (_cost),
        nnSearched  (false)
    {}
    
public:
    FMT_SetType              setType;
    double                   cost;
    bool                     nnSearched;
    const ompl::base::State* parent;
    stateVector              nbh;
}; // struct FMT_Aux

/************************************************************************/
/************************************************************************/

class FMTclone : public ompl::base::Planner
{
protected:
    static constexpr unsigned DEFAULT_NUM_SAMPLES = 1000;
    static constexpr double   DEFAULT_DIST_MULTIPLIER = 1.1;
    
public:
    FMTclone (const base::SpaceInformationPtr &si);
    
    ~FMTclone (void) override;
    
public:
    void setup (void) override;
    void clear (void) override;
    
    void free  (void);
    
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
    void addSolutionPath (void);
    
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
    void saveNear (const ompl::base::State* z);
    
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
    std::priority_queue< std::pair<double, const ompl::base::State*> >
    V_open_;
    
    // Hashed map of the auxillary data of a node.
    std::unordered_map<const ompl::base::State*, FMT_AuxData> auxData_;

protected:
    unsigned numSamples_     {DEFAULT_NUM_SAMPLES};
    double   distMultiplier_ {DEFAULT_DIST_MULTIPLIER};
        
protected:
    double mu_free_; // The free space volume
    double r_n_; // The distance threshold for neighbors
    
protected:
    const ompl::base::State* goal_;
}; // class FMT
    
}; // namespace EE698G
    
}}; // namespaces ompl, geometric

/************************************************************************/
/************************************************************************/

#endif