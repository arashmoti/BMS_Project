#ifndef __IDATETIME__H_
#define __IDATETIME__H_

#include "stddef.h"
#include "datetime_handler_params.h"


class IDatetime
{
public:
    IDatetime() {};
    virtual ~IDatetime() = default;

    virtual void setDate(RtcArgs::st_dateInfo_t* date) = 0;

};


#endif // __IDATETIME__H_