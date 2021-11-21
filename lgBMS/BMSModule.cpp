#include "config.h"
#include "BMSModule.h"
#include "BMSUtil.h"
#include "Logger.h"

BMSModule::BMSModule()
{
}

// Convert a 16-bit ADC value to a float voltage
float BMSModule::decodeVoltage(uint16_t data) {
  return(data * 5.0f / 65535.0f);
}

// Convert a 16-bit ADC value to a float temperature in degrees C
float BMSModule::decodeTemperature(uint16_t data) {
  // Calculate NTC resistance
  float r = 0.0000000347363427499292f * data * data - 0.001025770762903f * data + 2.68235340614337f;
  // Calculate NTC temperature
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}

// Decode an incoming CAN message. This function assumes that the
// CAN messages relates to this module.
void BMSModule::decodecan(CAN_message_t &msg) {
  // We're receiving data from this module, so it exists
  exists = true;
  // Extract 16 bit data vale from CAN message
  uint16_t data = ((uint16_t)(message->data[2]) << 8) | message->data[3];
  uint8_t cell = buf[1];
  // Update the received data bitmap to indicate what has been received
  dataReceived |= (1 << cell);
  // Store the received data and update the high and low points
  if(cell < 16) {
    // Cell voltages
    cellVolt[buf[1]] = decodeVoltage(data);
    if(cellVolt[cell] < lowestCellVolt[cell])  lowestCellVolt[cell]  = cellVolt[cell];
    if(cellVolt[cell] > highestCellVolt[cell]) highestCellVolt[cell] = cellVolt[cell];
  } else if(cell == 17) {
    // External NTC - negative side
    temperature[0] = decodeTemperature(data);
    if(temperature[0] > highestTemperature[0]) highestTemperature[0] = temperature[0];
    if(temperature[0] < lowestTemperature[0])  lowestTemperature[0]  = temperature[0];
  } else if(cell == 18) {
    // External NTC - positive side
    temperature[1] = decodeTemperature(data);
    if(temperature[1] > highestTemperature[1]) highestTemperature[1] = temperature[1];
    if(temperature[1] < lowestTemperature[1])  lowestTemperature[1]  = temperature[1];
  } else if(cell == 0xff) {
    // Balancing status bitmap
    balstat = data;
  }
}

// Return the voltage of a specified cell
float BMSModule::getCellVoltage(int cell) {
  return cellVolt[cell];
}

// Return the temperature of a specified sensor
float BMSModule::getTemperature(int sensor) {
  return temperature[sensor];
}

// Return the voltage of the lowest voltage cell in the module
float BMSModule::getLowCellV() {
  float lowCellV = 5.0f;
  for(int n=0; n<16; n++)
    if(cellVolt[n] < lowCellV)
      lowCellV = cellVolt[n];
  return lowCellV;
}

// Return the voltage of the highest voltage cell in the module
float BMSModule::getHighCellV() {
  float highCellV = 0.0f;
  for(int n=0; n<16; n++)
    if(cellVolt[n] < highCellV)
      highCellV = cellVolt[n];
  return highCellV;
}

// Return the lowest temperature in the module
float BMSModule::getLowTemp() {
  if(temperature[1] < temperature[0]) return temperature[1];
  else return temperature[0];
}

// Return the highest temperature in the module
float BMSModule::getHighTemp() {
  if(temperature[1] > temperature[0]) return temperature[1];
  else return temperature[0];
}

// Return the highest voltage recorded for a specified cell
float BMSModule::getHighestCellVolt(int cell)
{
  return highestCellVolt[cell];
}

// Return the lowest voltage recorded for a specified cell
float BMSModule::getLowestCellVolt(int cell)
{
  return lowestCellVolt[cell];
}

// Return the highest temperature recorded by a specified sensor
float BMSModule::getHighestTemp(int sensor)
{
  return highestTemperature[sensor];
}

// Return the lowest temperature recorded by a specified sensor
float BMSModule::getLowestTemp(int sensor)
{
  return lowestTemperature[sensor];
}

// Return the balancing status bitmap
uint16_t BMSModule::getBalStat()
{
  return balstat;
}
