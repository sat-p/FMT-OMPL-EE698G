#include "../include/ompl_test.h"

#include <iostream>

/************************************************************************/

int main (void)
{
    OMPL_TEST test;
    
    test.planWithoutSimpleSetup();
    
    std::cerr << "Ran test " << std::endl;
    while (1);
    
    return 0;
}

/************************************************************************/
/************************************************************************/
