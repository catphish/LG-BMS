#pragma once
#include <FlexCAN.h>

class BMSModule
{
  public:
    BMSModule();
    void clearModule();
    void decodecan(CAN_message_t &msg);
    bool isDataValid();
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
    float getModuleVoltage();
    uint16_t getBalStat();

  private:
    uint32_t dataReceived;
    uint16_t balstat;
    float cellVolt[16];
    float lowestCellVolt[16];
    float highestCellVolt[16];
    float temperature[2];
    float lowestTemperature[2];
    float highestTemperature[2];
    float decodeVoltage(uint16_t data);
    float decodeTemperature(uint16_t data);
    uint32_t lastData;
};
