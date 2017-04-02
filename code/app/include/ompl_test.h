#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

/************************************************************************/

namespace ob = ompl::base;
namespace og = ompl::geometric;

/************************************************************************/

class OMPL_TEST
{
public:
    OMPL_TEST (void) = default;

public:
    void planWithoutSimpleSetup (void);
    
private:
    static bool isStateValid (const ob::State *state);
};

/************************************************************************/
/************************************************************************/