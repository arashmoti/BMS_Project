#include "state_of_charge_handler.h"
#include <algorithm>

StateOfChargeHandler::StateOfChargeHandler() {}
StateOfChargeHandler::~StateOfChargeHandler() {}

uint16_t StateOfChargeHandler::getVoltageToSoc(float cell_voltage, int32_t pack_current, int32_t temperature_mc)
{
    float temp_c = (float)temperature_mc / 1000.0f;
    float internal_resistance_mohms = Interpolate(temp_points, resistance_points, resistance_table_size, temp_c);
    float resistance_ohms = internal_resistance_mohms / 1000.0f;
    float current_amps = (float)pack_current / 1000.0f;
    float ir_compensation = current_amps * resistance_ohms;
    float estimated_ocv = cell_voltage + ir_compensation;

    return static_cast<uint16_t>(Interpolate(ocv_voltage, ocv_soc, ocv_table_size, estimated_ocv) * 1000.0);
}

float StateOfChargeHandler::Interpolate(const float x_table[], const float y_table[], int table_size, float x)
{
    if (x <= x_table[0]) return y_table[0];
    if (x >= x_table[table_size - 1]) return y_table[table_size - 1];

    for (int i = 0; i < table_size - 1; ++i)
    {
        if (x >= x_table[i] && x <= x_table[i + 1])
        {
            const float delta = (x - x_table[i]) / (x_table[i + 1] - x_table[i]);
            return y_table[i] + delta * (y_table[i + 1] - y_table[i]);
        }
    }
    return y_table[table_size - 1]; // Should not be reached
}