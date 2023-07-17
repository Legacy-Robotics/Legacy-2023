#include "tap/drivers.hpp"

//todo: include driver mocks here once we begin unit testing
namespace src::standard
{
    class Drivers : public tap::Drivers
    {
        friend class DriversSingleton;

    #ifdef ENV_UNIT_TESTS
    public:
    #endif //ENV_UNIT_TESTS
        //initialize mock drivers here with the constructor
        Drivers() : tap::Drivers() {}
    
    #if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
        //declare drivers as members here
    #endif
    }; 
} //namespace src::standard