#include <Arduino.h>
#include <ESPTelnet.h>


ESPTelnet telnet;
WiFiServer Server(23);



void telnetTask(void*) {
    while (true) {
        telnet.loop();
        if (Serial.available()) {
            telnet.print(Serial.read());
        }
    }
}

int telnetLogCallback(const char *fmt, va_list args) {
    if (telnet.isConnected()) {
        char buf[256];
        vsnprintf(buf, sizeof(buf), fmt, args);
        telnet.print(buf);
    }
    return ESP_OK;
}

void onTelnetInput(String str) {
    // checks for a certain command
    if (str == "ping") {
        telnet.println("> pong");
    // disconnect the client
    } else if (str == "bye" || str == "quit") {
        telnet.println("> disconnecting you...");
        telnet.disconnectClient();
    }
    ESP_LOG_LEVEL(ESP_LOG_INFO, "telnet", "received: %s", str.c_str());
}


/**
 * @brief 
 * 
 * @param fmt 
 * @param args 
 * @return `ESP_OK` on success
 */
int udpLogCallback(const char *fmt, va_list args) {
    static WiFiClient client;
    char buf[256];
    if (Server.hasClient()) {
        if (client.connected()) {
            Server.available().stop();
        } else {

        client = Server.available();
        }
    }
        vsnprintf(buf, sizeof(buf), fmt, args);
        client.print(buf);
    return ESP_OK;
}


