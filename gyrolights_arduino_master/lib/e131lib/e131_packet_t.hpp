#pragma once
#include <stdint.h>

#if defined(ARDUINO)
    #include <Arduino.h>
#elif defined(_WIN64) || defined(_WIN32)
    #include <WinSock2.h>

#elif defined(__linux__)

#else 
    #error Platform not supported!
#endif

#if defined(ESP8266) 
    #include <ESP8266WiFi.h>
    #include <lwip/def.h>
    #define SHORT_AT(x)  (((*(x))<<8 )+ *(x+1))
    #define LONG_AT(x)  ((0xff000000 & ((*(x+3))<<24)) | (0xff0000 & ((*(x+2))<<16)) | (0xff00 & (((*(x+1)))<<8)) | ( 0xff & (*(x))))
#else
    #define SHORT_AT(x) *reinterpret_cast<uint16_t*>(x)
    #define LONG_AT(x) *reinterpret_cast<uint32_t*>(x)
#endif

#if defined(ESP32)
    #include <WiFi.h>
#endif


#define VECTOR_ROOT_E131_DATA 0x00000004
#define VECTOR_ROOT_E131_EXTENDED 0x00000008
#define VECTOR_DMP_SET_PROPERTY 0x02 // Informative
#define VECTOR_E131_DATA_PACKET 0x00000002
#define VECTOR_E131_EXTENDED_SYNCHRONIZATION 0x00000001
#define VECTOR_E131_EXTENDED_DISCOVERY 0x00000002
#define VECTOR_UNIVERSE_DISCOVERY_UNIVERSE_LIST 0x00000001
#define E131_E131_UNIVERSE_DISCOVERY_INTERVAL 10000 // milliseconds
#define E131_NETWORK_DATA_LOSS_TIMEOUT 2500 // milliseconds
#define E131_DISCOVERY_UNIVERSE 64214
#define ACN_SDT_MULTICAST_PORT 5568

// Error Types
typedef enum {
    ERROR_NONE,
    ERROR_IGNORE,
    ERROR_ACN_ID,
    ERROR_PACKET_SIZE,
    ERROR_VECTOR_ROOT,
    ERROR_VECTOR_FRAME,
    ERROR_VECTOR_DMP,
    ERROR_PREAMBLE_SIZE,
    ERROR_POSTAMBLE_SIZE,
    ERROR_ADDRESS_TYPE_AND_DATA_TYPE,
    ERROR_FIRST_PROPERTY_ADDRESS,
    ERROR_ADDRESS_INCREMENT,
    ERROR_VECTOR_UNIVERSE_DISCOVERY_LAYER
} e131_error_t;


typedef struct {
        uint16_t raw;
        
        uint16_t Flags_and_Length() {return ntohs(raw);};
        uint8_t Flags() {return (raw>>12)&0xf;};
        uint16_t Length() {return ntohs(raw&htons(0x0fff));};
} flags_and_length_t;



typedef struct {uint8_t raw[12];} acn_packet_identifier_t;
typedef struct {uint8_t raw[16];} CID_t;

typedef struct {
    uint8_t raw[38];

    uint16_t Preamble_Size() {return ntohs(*reinterpret_cast<uint16_t*>(raw));};
    uint16_t Postamble_Size() {return ntohs(*reinterpret_cast<uint16_t*>(raw+2));};
    acn_packet_identifier_t ACN_Packet_Identifier() {return *reinterpret_cast<acn_packet_identifier_t*>(raw+4);}; // = {0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};
    flags_and_length_t Flags_and_Length() {return *reinterpret_cast<flags_and_length_t*>(raw+16);};
    uint32_t Vector() {return ntohl(LONG_AT(raw+18));}; // breaks on esp8266
    CID_t CID() {return *reinterpret_cast<CID_t*>(raw+22);};
} root_layer_t;



typedef struct {uint8_t raw[64];} source_name_t;


typedef struct {
    uint8_t raw[77];
    flags_and_length_t Flags_and_Length() {return *reinterpret_cast<flags_and_length_t*>(raw);};
    uint32_t Vector() {return ntohl(LONG_AT(raw+2));};  // breaks esp8266
    source_name_t Source_Name() {return *reinterpret_cast<source_name_t*>(raw+6);};
    uint8_t Priority() {return *reinterpret_cast<uint8_t*>(raw+70);}; //0-200, default of 100 
    uint16_t Synchronization_Address() {return ntohs(*reinterpret_cast<uint16_t*>(raw+71));};
    uint8_t Sequence_Number() {return *reinterpret_cast<uint8_t*>(raw+73);};
    uint8_t Options() {return *reinterpret_cast<uint8_t*>(raw+74);};
    uint16_t Universe() {return ntohs(*reinterpret_cast<uint16_t*>(raw+75));};
} e131_data_packet_framing_layer_t;
    


typedef struct {uint8_t raw[513];} property_values_t; // actually variable length!

typedef struct {
    uint8_t raw[523];
    flags_and_length_t Flags_and_Length() {return *reinterpret_cast<flags_and_length_t*>(raw);};
    uint8_t Vector() {return *reinterpret_cast<uint8_t*>(raw+2);};
    uint8_t  Address_Type_Data_Type() {return *reinterpret_cast<uint8_t*>(raw+3);};
    uint16_t First_Property_Address() {return ntohs(SHORT_AT(raw+4));}; //breaks on esp8266
    uint16_t Address_Increment() {return ntohs(*reinterpret_cast<uint16_t*>(raw+6));};
    uint16_t Property_value_count() {return ntohs(*reinterpret_cast<uint16_t*>(raw+8));};
    property_values_t Property_values() {return *reinterpret_cast<property_values_t*>(raw+10);};
} dmp_layer_t;


typedef struct {
    uint8_t raw[10];
    flags_and_length_t Flags_and_Length() {return *reinterpret_cast<flags_and_length_t*>(raw);};
    uint32_t Vector() {return ntohl(*reinterpret_cast<uint32_t*>(raw+2));};
    uint8_t Page() {return *reinterpret_cast<uint8_t*>(raw+6);};
    uint8_t Lase_Page() {return *reinterpret_cast<uint8_t*>(raw+7);};
    uint8_t* List_of_Universes() {return raw+8;};


} universe_discovery_layer_t;


typedef struct e131_packet {
    uint8_t* raw = nullptr;
    unsigned int length = 0;
    root_layer_t* rootLayer = nullptr;
    e131_data_packet_framing_layer_t* framingLayer = nullptr;
    dmp_layer_t* dmpLayer = nullptr;
    universe_discovery_layer_t* universerDiscoveryLayer = nullptr;
    e131_packet();
    e131_packet(uint8_t* datagram, unsigned int length);
    const static uint8_t acn_ID[12];
} e131_packet_t;