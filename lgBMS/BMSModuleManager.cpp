#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

extern EEPROMSettings settings;

BMSModuleManager::BMSModuleManager()
{
  clearmodules();
}

// Decode CAN data and dispatch to appropriate module
void BMSModuleManager::decodecan(CAN_message_t &msg)
{
  int chain_id, module_id;
  if(msg.ext) return;

  // Each daisychain of 16 modules has its own CAN connection and address
  if      (msg.id == 0x4f0) chain_id = 0;
  else if (msg.id == 0x4f1) chain_id = 1;
  else if (msg.id == 0x4f2) chain_id = 2;
  else if (msg.id == 0x4f3) chain_id = 3;
  else return;
  // Module ID within daisychain
  module_id = msg.buf[0];

  if(module_id == 0xff) {
    // Module manager status
  } else {
    // Module data
    modules[chain_id * 16 + module_id].decodecan(msg);
  }
}

// Clear module status
void BMSModuleManager::clearmodules()
{
  // Clear state of all modules
  for (int y = 0; y <= MAX_MODULE_ADDR; y++)
      modules[y].clearModule();
}

// Return full pack voltage
float BMSModuleManager::getPackVoltage()
{
  if(pStrings == 0) return 0;
  float packVoltage = 0.0f;
  for (int y = 0; y <= MAX_MODULE_ADDR; y++)
    if(modules[y].isDataValid())
      packVoltage += modules[y].getModuleVoltage();
  return packVoltage / (float)pStrings;
}

// Return the highest voltage of any cell in the pack
float BMSModuleManager::getHighCellVolt()
{
  float highCellVolt = 0.0;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++)
    if (modules[x].isDataValid())
      if (modules[x].getHighCellV() > highCellVolt) 
        highCellVolt = modules[x].getHighCellV();
  return highCellVolt;
}

// Return the lowest voltage of any cell in the pack
float BMSModuleManager::getLowCellVolt()
{
  float lowCellVolt = 5.0;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++)
    if (modules[x].isDataValid())
      if (modules[x].getLowCellV() < lowCellVolt) 
        lowCellVolt = modules[x].getLowCellV();
  return lowCellVolt;
}

// Return the highest temperature of any sensor in the pack
float BMSModuleManager::getHighTemperature()
{
  float highTemp = -200;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++)
    if (modules[x].isDataValid())
      if (modules[x].getHighTemp() > highTemp) 
        highTemp = modules[x].getHighTemp();
  return highTemp;
}

// Return the lowest temperature of any sensor in the pack
float BMSModuleManager::getLowTemperature()
{
  float lowTemp = -200;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++)
    if (modules[x].isDataValid())
      if (modules[x].getLowTemp() < lowTemp) 
        lowTemp = modules[x].getLowTemp();
  return lowTemp;
}

// Return the average cell voltage in the pack
float BMSModuleManager::getAvgCellVolt()
{
  float avg = 0.0f;
  int cellCount = 0;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isDataValid()) {
      avg += modules[x].getModuleVoltage();
      cellCount += 16;
    }
  }
  if(cellCount == 0) return 0;

  return avg / (float)cellCount;
}

// Return average temperature of pack
float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  int sensorCount = 0;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isDataValid()) {
      avg += modules[x].getTemperature(0);
      avg += modules[x].getTemperature(1);
      sensorCount += 2;
    }
  }
  if(sensorCount == 0) return 0;
  return avg / (float)sensorCount;
}

// Return the number of detected modules in the pack
int BMSModuleManager::getNumModules()
{
  int moduleCount = 0;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++)
    if (modules[x].isDataValid())
      moduleCount++;
  return moduleCount;
}

// Set the number of parallel strings to divide total pack voltage
void BMSModuleManager::setPstrings(int n)
{
  pStrings = n;
}

// Return the number of cells in series in the pack
int BMSModuleManager::seriescells()
{
  if(pStrings == 0) return 0;
  return getNumModules() * 16 / pStrings;
}

// Print all cell data to serial console in CSV format
void BMSModuleManager::printAllCSV(unsigned long timestamp, float current, int SOC)
{
  for (int y = 0; y <= MAX_MODULE_ADDR; y++)
  {
    if (modules[y].isDataValid())
    {
      SERIALCONSOLE.print(timestamp);
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(current, 0);
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(SOC);
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(y);
      SERIALCONSOLE.print(",");
      for (int i = 0; i < 16; i++)
      {
        SERIALCONSOLE.print(modules[y].getCellVoltage(i));
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      SERIALCONSOLE.println();
    }
  }
  for (int y = 0; y <= MAX_MODULE_ADDR; y++)
  {
    if (modules[y].isDataValid())
    {
      Serial2.print(timestamp);
      Serial2.print(",");
      Serial2.print(current, 0);
      Serial2.print(",");
      Serial2.print(SOC);
      Serial2.print(",");
      Serial2.print(y);
      Serial2.print(",");
      for (int i = 0; i < 8; i++)
      {
        Serial2.print(modules[y].getCellVoltage(i));
        Serial2.print(",");
      }
      Serial2.print(modules[y].getTemperature(0));
      Serial2.print(",");
      Serial2.print(modules[y].getTemperature(1));
      Serial2.println();
    }
  }
}

// Print general information about the state of the pack
void BMSModuleManager::printPackSummary()
{
  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i  Cells: %i  Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", getNumModules(), seriescells(),
                  getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  Logger::console("");
  for (int y = 0; y <= MAX_MODULE_ADDR; y++)
  {
    if (modules[y].isDataValid())
    {
      Logger::console("                               Module #%i", y);

      Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
    }
  }
}

// Print detailed pack data
void BMSModuleManager::printPackDetails(int digits, bool showbal)
{
  int cellNum = 0;
  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV Delta Voltage: %zmV   Avg Temp: %fC ", getNumModules(), seriescells(),
                  pStrings, getPackVoltage(), getAvgCellVolt(), getLowCellVolt(), getHighCellVolt(), (getLowCellVolt() - getHighCellVolt()) * 1000, getAvgTemperature());
  Logger::console("");
  for (int y = 0; y <= MAX_MODULE_ADDR; y++)
  {
    if (modules[y].isDataValid())
    {
      uint16_t bal = modules[y].getBalStat();

      SERIALCONSOLE.print("Module #");
      SERIALCONSOLE.print(y);
      if (y < 10) SERIALCONSOLE.print(" ");
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(modules[y].getModuleVoltage(), digits);
      SERIALCONSOLE.print("V");
      for (int i = 0; i < 16; i++)
      {
        if (cellNum < 10) SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print("  Cell");
        SERIALCONSOLE.print(cellNum++);
        SERIALCONSOLE.print(": ");
        SERIALCONSOLE.print(modules[y].getCellVoltage(i), digits);
        SERIALCONSOLE.print("V");
        if (showbal == 1)
        {
          if ((bal & (0x1 << i)) > 0)
          {
            SERIALCONSOLE.print(" X");
          }
          else
          {
            SERIALCONSOLE.print(" -");
          }
        }
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.print(" Temp 1: ");
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print("C Temp 2: ");
      SERIALCONSOLE.print(modules[y].getTemperature(1));

      if (showbal == 1)
      {
        SERIALCONSOLE.print("C  Bal Stat: ");
        SERIALCONSOLE.println( bal, BIN);
      }
      else
      {
        SERIALCONSOLE.println("C");
      }

    }
  }
}
