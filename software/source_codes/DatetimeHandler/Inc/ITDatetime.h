#ifndef __ITDATETIME__H_
#define __ITDATETIME__H_


class ITDatetime
{
public:
    ITDatetime() {};
    virtual ~ITDatetime() = default;

    virtual void task(void) = 0;

};


#endif // __ITDATETIME__H_