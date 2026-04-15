#ifndef __ITINVERTER_CAN__H_
#define __ITINVERTER_CAN__H_

class ITInverterCan
{
public:
    ITInverterCan() {};
    virtual ~ITInverterCan() = default;

    virtual void task(void) = 0;
};


#endif // __ITINVERTER_CAN__H_
