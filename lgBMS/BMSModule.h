#pragma once
#include <FlexCAN.h>

class BMSModule
{
  public:
    BMSModule();
    void decodecan(CAN_message_t &msg);
    float getCellVoltage(int cell);
    float getTemperature(int sensor);
    float getLowCellV();
    float getHighCellV();
    float getLowTemp();
    float getHighTemp();
    float getHighestCellVolt(int cell);
    float getLowestCellVolt(int cell);
    float getHighestTemp(int sensor);
    float getLowestTemp(int sensor);
    uint16_t getBalStat();

  private:
    bool exists;
    uint32_t dataReceived;
    uint16_t balstat;
    float cellVolt[16];
    float lowestCellVolt[16];
    float highestCellVolt[16];
    float temperature[2];
    float lowestTemperature[2];
    float highestTemperature[2];
};
