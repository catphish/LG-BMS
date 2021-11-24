#include  "config.h"
#include "BMSModule.h"
#include <FlexCAN.h>

class BMSModuleManager
{
  public:
    BMSModuleManager();
    int seriescells();
    void decodecan(CAN_message_t &msg);
    void setPstrings(int Pstrings);
    float getPackVoltage();
    float getAvgTemperature();
    float getHighTemperature();
    float getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getHighVoltage();
    float getLowVoltage();
    void printAllCSV(unsigned long timestamp,float current, int SOC);
    void printPackSummary();
    void printPackDetails(int digits,bool showbal);
    int getNumModules();

  private:
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    void clearmodules();
    int pStrings;
};
