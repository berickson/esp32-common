#include <Arduino.h>
#include "SSD1306Wire.h"
#include "WiFi.h"
#include "Preferences.h"
#include "StringStream.h"

/*
lib_deps=
    ESP8266 and ESP32 OLED driver for SSD1306 displays,
    CmdParser@984806de91
    ESP Async WebServer@1.2.3
*/
// https://github.com/pvizeli/CmdParser
#include "CmdBuffer.hpp"
#include "CmdParser.hpp"
#include "CmdCallback.hpp"

#include "SPIFFS.h"
#include "esp_wifi.h"
#include "ESPAsyncWebServer.h"

// board at https://www.amazon.com/gp/product/B07DKD79Y9
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_sdl = 15;
const int pin_oled_rst = 16;

SSD1306Wire display(oled_address, pin_oled_sda, pin_oled_sdl);
Preferences preferences;

bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

class LineReader {
  public:
  const uint32_t buffer_size = 500;
  String buffer;
  String line;
  bool line_available = true;

  LineReader() {
    buffer.reserve(buffer_size);
    line.reserve(buffer_size);
  }

  bool get_line(Stream & stream) {
    while(stream.available()) {
      char c = (char)stream.read();
      if(c=='\n') continue;
      if(c== '\r') {
        if(true){ //buffer.length() > 0) {
          line = buffer;
          buffer = "";
          return true;
        }
      } else {
        buffer.concat(c);
      }
    }
    return false;
  }
};

class CommandEnvironment {
public:
  CommandEnvironment(CmdParser & args, Stream & cout, Stream & cerr) 
  : args(args),cout(cout),cerr(cerr)
  {
  }
  CmdParser & args;
  Stream & cout;
  Stream & cerr;
  bool ok = true;
};

typedef void (*CommandCallback)(CommandEnvironment &env);


class Command {
public:
  Command() 
    : execute(nullptr)
  {}
  Command(const char * name, CommandCallback callback, const char * helpstring = nullptr) {
    this->name = name;
    this->execute = callback;
    this->helpstring = helpstring;
  }
  String name;
  CommandCallback execute;
  String helpstring;
};

const int wifi_port = 80;
AsyncWebServer server(wifi_port);

class WifiTask {
public:
  unsigned long connect_start_ms = 0;
  unsigned long last_execute_ms = 0;
  String ssid;
  String password;

  bool enabled=false;
  bool trace = false;
  bool log_serial = true;

  enum {
    status_disabled,
    status_not_connected,
    status_connecting,
    status_connected
  } current_state = status_disabled;

  void set_enable(bool enable_wifi) {
    if(enable_wifi==this->enabled) return;
    enabled = enable_wifi;
    if(enabled) {
      current_state = status_not_connected;
    } else {
      WiFi.disconnect(true, true);
      current_state = status_disabled;
    }
  }

  void set_connection_info(String ssid, String password) {
    WiFi.disconnect();
    this->ssid = ssid;
    this->password = password;
    if(current_state != status_disabled) {
      current_state = status_not_connected;
    }

    Serial.print("connection info set to ssid: ");
    Serial.print(ssid);
    Serial.print(" password: ");
    Serial.print(password);
    Serial.println();
  }


  void execute() {
    auto ms = millis();
    auto wifi_status = WiFi.status();

    switch (current_state) {
      case status_disabled:
        break;

      case status_not_connected:
        connect_start_ms = ms;
        WiFi.mode(WIFI_STA);
        esp_wifi_set_ps (WIFI_PS_NONE);
        WiFi.begin(ssid.c_str(), password.c_str());
        current_state = status_connecting;
        break;
      
      case status_connecting:
        if(wifi_status == WL_CONNECT_FAILED) {
          current_state = status_not_connected;
          if(trace) Serial.println("connection failed");
          current_state = status_not_connected;
          break;
        }
        if (wifi_status == WL_CONNECTED) {
          server.begin();
          current_state = status_connected;
          if(trace) Serial.print("wifi connected, web server started");
        } else {
          if(ms - connect_start_ms > 5000) {
            if(trace) Serial.print("couldn't connect, trying again");
            WiFi.disconnect();
            current_state = status_not_connected;
            break;
          }
        }
        break;

      case status_connected:
        if (wifi_status != WL_CONNECTED) {
          if(trace) Serial.print("wifi disconnected, web server stopped");
          current_state = status_not_connected;
          server.end();
          WiFi.disconnect();
        }
        break;

      default:
        Serial.println("invalid sate in WifiTask");
    }
    last_execute_ms = ms;
  }
};

WifiTask wifi_task;

void cmd_set_wifi_config(CommandEnvironment & env) {
  char * ssid = env.args.getCmdParam(1);
  char * password = env.args.getCmdParam(2);
  preferences.begin("main",false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();
  wifi_task.set_connection_info(ssid, password);
  preferences.begin("main",true);
  env.cout.print("ssid set to \"");
  env.cout.print(preferences.getString("ssid"));
  env.cout.print("\", password set to \"");
  env.cout.print(preferences.getString("password"));
  env.cout.print("\"");
  env.cout.println();
  preferences.end();
}


std::vector<Command> commands;

void cmd_help(CommandEnvironment & env) {
  for(auto command : commands) {
    env.cout.print(command.name);
    env.cout.print(": ");
    env.cout.print(command.helpstring);
    env.cout.println();
  }
}
Command * get_command_by_name(const char * command_name) {
  for(auto & command : commands) {
    if(command.name == command_name) {
      return &command;
    }
  }
  return nullptr;
}

void esp32_common_setup() {
  Serial.begin(921600);
  if(!SPIFFS.begin()){
    Serial.println("SPIFFS.begin Failed");
    return;
  }
  Serial.println("SPIFFS started");
  preferences.begin("main", true);
  wifi_task.set_connection_info(preferences.getString("ssid",""), preferences.getString("password",""));
  wifi_task.set_enable(preferences.getBool("enable_wifi", true));
  preferences.end();

  // prepare oled display
  pinMode(pin_oled_rst, OUTPUT);
  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);
  display.init();

  commands.reserve(50);
  commands.emplace_back(Command{"help", cmd_help, "displays list of available commands"});
  commands.emplace_back(Command{"set_wifi_config", cmd_set_wifi_config, "{ssid} {password}"});
//  commands.emplace_back(Command{"set_enable_wifi", cmd_set_enable_wifi});

   server.on(
    "/command",
    HTTP_POST,
    // request
    [](AsyncWebServerRequest * request){
      //Serial.println("got a request");
      //Serial.println(request->contentLength());

      auto params=request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost() && p->name() == "body") {
          CmdParser parser;
          String body = p->value();

          parser.parseCmd((char *)body.c_str());
          auto command = get_command_by_name(parser.getCommand());
          if(command == nullptr) {
              request->send(200,"text/plain","command failed");
          } else {
              String output_string;
              output_string.clear();
              output_string.reserve(500*30);
              StringStream output_stream(output_string);
              CommandEnvironment env(parser, output_stream, output_stream);
              command->execute(env);
              if(env.ok) {
                request->send(200,"text/plain", output_string);
                return;
              } else {
                request->send(200,"text/plain","command failed");
                return;
              }
          }
        }
        request->send(400,"text/plain","no post body sent");
      }
    }
    );
  

}

void esp32_common_loop() {
  static uint32_t last_loop_ms = 0;
  uint32_t loop_ms = millis();

  if(every_n_ms(loop_ms, last_loop_ms, 100)) {
    wifi_task.execute();
  }
  static LineReader line_reader;
  static String last_bluetooth_line;
  auto & serial = Serial;

  // check for a new line in bluetooth
  if(line_reader.get_line(serial)) {
    CmdParser parser;
    parser.parseCmd((char *)line_reader.line.c_str());
    Command * command = get_command_by_name(parser.getCommand());
    if(command) {
      CommandEnvironment env(parser, serial, serial);
      command->execute(env);
    } else {
      serial.print("ERROR: Command not found - ");
      serial.println(parser.getCommand());
    }
    last_bluetooth_line = line_reader.line;
  }

  last_loop_ms = loop_ms;
}
