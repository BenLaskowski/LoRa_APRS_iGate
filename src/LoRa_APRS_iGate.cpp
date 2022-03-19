#include <map>

#include <APRS-IS.h>
#include <BoardFinder.h>
#include <System.h>
#include <TaskManager.h>
#include <logger.h>
#include <power_management.h>

#include "TaskAprsIs.h"
#include "TaskDisplay.h"
#include "TaskEth.h"
#include "TaskFTP.h"
#include "TaskMQTT.h"
#include "TaskModem.h"
#include "TaskNTP.h"
#include "TaskOTA.h"
#include "TaskRouter.h"
#include "TaskWifi.h"
#include "project_configuration.h"

#define VERSION     "22.11.1"
#define MODULE_NAME "Main"

String create_lat_aprs(double lat);
String create_long_aprs(double lng);

TaskQueue<std::shared_ptr<APRSMessage>> toAprsIs;
TaskQueue<std::shared_ptr<APRSMessage>> fromModem;
TaskQueue<std::shared_ptr<APRSMessage>> toModem;
TaskQueue<std::shared_ptr<APRSMessage>> toMQTT;

System        LoRaSystem;
Configuration userConfig;

DisplayTask displayTask;
ModemTask   modemTask(fromModem, toModem);
EthTask     ethTask;
WifiTask    wifiTask;
OTATask     otaTask;
NTPTask     ntpTask;
FTPTask     ftpTask;
MQTTTask    mqttTask(toMQTT);
AprsIsTask  aprsIsTask(toAprsIs);
RouterTask  routerTask(fromModem, toModem, toAprsIs, toMQTT);

void setup() {
  Serial.begin(115200);
  LoRaSystem.getLogger().setSerial(&Serial);
  delay(500);
  LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_INFO, MODULE_NAME, "LoRa APRS iGate by OE5BPA (Peter Buchegger)");
  LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_INFO, MODULE_NAME, "Version: %s", VERSION);

  std::list<BoardConfig const *> boardConfigs;
  boardConfigs.push_back(&TTGO_LORA32_V1);
  boardConfigs.push_back(&TTGO_LORA32_V2);
  boardConfigs.push_back(&TTGO_T_Beam_V0_7);
  boardConfigs.push_back(&TTGO_T_Beam_V1_0);
  boardConfigs.push_back(&ETH_BOARD);
  boardConfigs.push_back(&TRACKERD);
  boardConfigs.push_back(&HELTEC_WIFI_LORA_32_V1);
  boardConfigs.push_back(&HELTEC_WIFI_LORA_32_V2);

  ProjectConfigurationManagement confmg;
  confmg.readConfiguration(userConfig);

  BoardFinder        finder(boardConfigs);
  BoardConfig const *boardConfig = finder.getBoardConfig(userConfig.board);
  if (!boardConfig) {
    boardConfig = finder.searchBoardConfig(LoRaSystem.getLogger());
    if (!boardConfig) {
      LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, MODULE_NAME, "Board config not set and search failed!");
      while (true)
        ;
    } else {
      userConfig.board = boardConfig->Name;
      confmg.writeConfiguration(userConfig);
      LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_INFO, MODULE_NAME, "will restart board now!");
      ESP.restart();
    }
  }

  LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_INFO, MODULE_NAME, "Board %s loaded.", boardConfig->Name);

  if (boardConfig->Type == eTTGO_T_Beam_V1_0) {
    Wire.begin(boardConfig->OledSda, boardConfig->OledScl);
    PowerManagement powerManagement;
    if (!powerManagement.begin(Wire)) {
      LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_INFO, MODULE_NAME, "AXP192 init done!");
    } else {
      LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, MODULE_NAME, "AXP192 init failed!");
    }
    powerManagement.activateLoRa();
    powerManagement.activateOLED();
    powerManagement.deactivateGPS();
  }

  LoRaSystem.setBoardConfig(boardConfig);
  LoRaSystem.setUserConfig(&userConfig);
  LoRaSystem.getTaskManager().addTask(&displayTask);
  LoRaSystem.getTaskManager().addTask(&modemTask);
  LoRaSystem.getTaskManager().addTask(&routerTask);

  if (userConfig.aprs_is.active) {
    if (boardConfig->Type == eETH_BOARD && !userConfig.wifi.active) {
      LoRaSystem.getTaskManager().addAlwaysRunTask(&ethTask);
    }
    if (userConfig.wifi.active) {
      LoRaSystem.getTaskManager().addAlwaysRunTask(&wifiTask);
    }
    LoRaSystem.getTaskManager().addTask(&otaTask);
    LoRaSystem.getTaskManager().addTask(&ntpTask);
    if (userConfig.ftp.active) {
      LoRaSystem.getTaskManager().addTask(&ftpTask);
    }
    LoRaSystem.getTaskManager().addTask(&aprsIsTask);
  }

  if (userConfig.mqtt.active) {
    LoRaSystem.getTaskManager().addTask(&mqttTask);
  }

  LoRaSystem.getTaskManager().setup(LoRaSystem);

  LoRaSystem.getDisplay().showSpashScreen("LoRa APRS iGate", VERSION);

  if (userConfig.callsign == "NOCALL-10") {
    LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, MODULE_NAME, "You have to change your settings in 'data/is-cfg.json' and upload it via 'Upload File System image'!");
    LoRaSystem.getDisplay().showStatusScreen("ERROR", "You have to change your settings in 'data/is-cfg.json' and upload it via \"Upload File System image\"!");
    while (true)
      ;
  }
  if ((!userConfig.aprs_is.active) && !(userConfig.digi.active)) {
    LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, MODULE_NAME, "No mode selected (iGate or Digi)! You have to activate one of iGate or Digi.");
    LoRaSystem.getDisplay().showStatusScreen("ERROR", "No mode selected (iGate or Digi)! You have to activate one of iGate or Digi.");
    while (true)
      ;
  }

  if (userConfig.display.overwritePin != 0) {
    pinMode(userConfig.display.overwritePin, INPUT);
    pinMode(userConfig.display.overwritePin, INPUT_PULLUP);
  }

  delay(5000);
  LoRaSystem.getLogger().log(logging::LoggerLevel::LOGGER_LEVEL_INFO, MODULE_NAME, "setup done...");
}

void loop() {
  LoRaSystem.getTaskManager().loop(LoRaSystem);
}

String create_lat_aprs(double lat) {
  char str[20];
  char n_s = 'N';
  if (lat < 0) {
    n_s = 'S';
  }
  lat = std::abs(lat);
  sprintf(str, "%02d%05.2f%c", (int)lat, (lat - (double)((int)lat)) * 60.0, n_s);
  String lat_str(str);
  return lat_str;
}

String create_long_aprs(double lng) {
  char str[20];
  char e_w = 'E';
  if (lng < 0) {
    e_w = 'W';
  }
  lng = std::abs(lng);
  sprintf(str, "%03d%05.2f%c", (int)lng, (lng - (double)((int)lng)) * 60.0, e_w);
  String lng_str(str);
  return lng_str;
}
