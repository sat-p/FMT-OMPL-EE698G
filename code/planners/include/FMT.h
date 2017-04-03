#ifndef __FMT_OMPL_EE698G__
#define __FMT_OMPL_EE698G__

/************************************************************************/

#include <ompl/geometric/planners/PlannerIncludes.h>

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
    ompl::base::PlannerStatus
    solve (const ompl::base::PlannerTerminationCondition &tc) override;
    
public:
    void getPlannerData (ompl::base::PlannerData &data) const override;
}; // class FMT
    
}; // namespace EE698G
    
}}; // namespaces ompl, geometric

/************************************************************************/
/************************************************************************/

#endif