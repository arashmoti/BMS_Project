#ifndef __ITPOWER_STATE__H_
#define __ITPOWER_STATE__H_

class ITPowerState
{
public:
  ITPowerState() {};
  virtual ~ITPowerState() = default;

  virtual void task(void) = 0;


};


#endif // __ITPOWER_STATE__H_