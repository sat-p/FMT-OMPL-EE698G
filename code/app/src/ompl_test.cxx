/*
 * Made using demo code from :
 * http://ompl.kavrakilab.org/RigidBodyPlanning_8cpp_source.html
 * Retrieved on 2nd April 2017
 */

/************************************************************************/

#include "../include/ompl_test.h"

/************************************************************************/

void OMPL_TEST::planWithoutSimpleSetup (void)
{
    // constructing the state space
    auto space (std::make_shared<ob::SE3StateSpace>());
    
    // setting bounds for R^3 part
    ob::RealVectorBounds bounds (3);
    bounds.setLow  (-1);
    bounds.setHigh (1);
    
    space->setBounds (bounds);
    
    // constructing an instance of space information
    auto spaceInformation (std::make_shared<ob::SpaceInformation> (space));
    
    // setting state validity checker
    spaceInformation->setStateValidityChecker(isStateValid);
    
    // random start space
    ob::ScopedState<> start (space);
    start.random();
    
    // random goal space
    ob::ScopedState<> goal (space);
    goal.random();
    
    // constructing a problem instance
    auto pdef (std::make_shared<ob::ProblemDefinition> (spaceInformation));
    
    // setting the start and goal states
    pdef->setStartAndGoalStates (start, goal);
    
    // creating a planner
    auto planner (std::make_shared<og::EE698G::FMTclone> (spaceInformation));
    //auto planner (std::make_shared<og::FMT> (spaceInformation));
    
    // setting problem for the planner.
    planner->setProblemDefinition (pdef);
    
    // setup the planner
    planner->setup();
    
    // Prints
    spaceInformation->printSettings(std::cout);
    pdef->print(std::cout);

    // attempt to solve the problem in 1 second
    auto solved = planner->ob::Planner::solve (1.0);
    
    if (solved)
    {
        auto path = pdef->getSolutionPath();
        if (path)
            path->print (std::cerr);
    }
}

/************************************************************************/
/************************************************************************/

bool OMPL_TEST::isStateValid (const ob::State *state)
{
    
    // casting pointer to the se3 state type
    const auto* se3state = state->as<ob::SE3StateSpace::StateType>();
    
    // extracting the first component of the se3 state space, i.e. a real
    // vector state space
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
   
    // extracting the first component of the se3 state space, i.e. a
    // 3 orientation state space.
    const auto*rot = se3state->as<ob::SO3StateSpace::StateType>(1);
  
    return true;
}

/************************************************************************/
/************************************************************************/