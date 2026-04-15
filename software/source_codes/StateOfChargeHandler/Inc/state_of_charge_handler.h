#ifndef __STATE_OF_CHARGE_HANDLER__H_
#define __STATE_OF_CHARGE_HANDLER__H_

#include "mbed.h"
#include "Settings.h"
#include "IMStateOfCharge.h"
#include <map>

class StateOfChargeHandler : public IMStateOfCharge
{
public:
  StateOfChargeHandler();
  virtual ~StateOfChargeHandler();

  virtual uint16_t getVoltageToSoc(float cell_voltage, int32_t pack_current, int32_t temperature_mc) override;

private:
  float Interpolate(const float voltage_table[], const float soc_table[], int table_size, float x);

private:
  // OCV lookup table using C-style arrays
  const float ocv_voltage[11] = {2.921, 3.223, 3.278, 3.303, 3.304, 3.307, 3.324, 3.339, 3.340, 3.341, 3.520};
  const float ocv_soc[11] = {0.0, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
  const int ocv_table_size = 11;

  // Temperature to internal resistance lookup table
  const float temp_points[3] = {-20.0f, 0.0f, 25.0f};
  const float resistance_points[3] = {7.46f, 2.54f, 1.15f};
  const int resistance_table_size = 3;
};

#endif