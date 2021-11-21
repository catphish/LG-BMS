#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

extern EEPROMSettings settings;

BMSModuleManager::BMSModuleManager()
{
}

// Decode CAN data and dispatch to appropriate module
void BMSModuleManager::decodecan(CAN_message_t &msg)
{
  int chain_id, module_id;
  if(msg->extended) return;

  // Each daisychain of 16 modules has its own CAN connection and address
  if      (msg->id == 0x4f0) chain_id = 0;
  else if (msg->id == 0x4f1) chain_id = 1;
  else if (msg->id == 0x4f2) chain_id = 2;
  else if (msg->id == 0x4f3) chain_id = 3;
  else return;
  // Module ID within daisychain
  module_id = msg->buf[0];

  if(module_id = 0xff) {
    // Module manager status
  } else {
    // Module data
    modules[chain_id * 16 + module_id].decodecan(msg);
  }
}
