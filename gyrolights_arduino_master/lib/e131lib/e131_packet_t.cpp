#include "e131_packet_t.hpp"
#include <stdint.h>
//#include <iostream>
//#include <cstring>

const uint8_t e131_packet::acn_ID[12] = {0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};

e131_packet::e131_packet()
{
}

e131_packet::e131_packet(uint8_t *datagram, unsigned int length)
    : raw(datagram),
      length(length)
{
    this->rootLayer = reinterpret_cast<root_layer_t*>(this->raw);
    if (this->rootLayer->Preamble_Size() != 0x010)
        throw e131_error_t::ERROR_PREAMBLE_SIZE;
    if (this->rootLayer->Postamble_Size() != 0x000)
        throw e131_error_t::ERROR_POSTAMBLE_SIZE;
    if (memcmp(this->rootLayer->ACN_Packet_Identifier().raw, this->acn_ID , sizeof(this->acn_ID)))
        throw e131_error_t::ERROR_ACN_ID;
    if (this->rootLayer->Vector() == VECTOR_ROOT_E131_DATA) {
        this->framingLayer = reinterpret_cast<e131_data_packet_framing_layer_t*>(this->raw + sizeof(root_layer_t));
        if (this->framingLayer->Vector() != VECTOR_E131_DATA_PACKET)
            throw e131_error_t::ERROR_VECTOR_FRAME;
        else {
            this->dmpLayer = reinterpret_cast<dmp_layer_t*>(this->raw + sizeof(root_layer_t) + sizeof(e131_data_packet_framing_layer_t));
            if (this->dmpLayer->Vector() != VECTOR_DMP_SET_PROPERTY)
                throw e131_error_t::ERROR_VECTOR_DMP;
            if (this->dmpLayer->Address_Type_Data_Type() != 0xa1)
                throw e131_error_t::ERROR_ADDRESS_TYPE_AND_DATA_TYPE;
            if (this->dmpLayer->First_Property_Address() != 0x0000)
                throw e131_error_t::ERROR_FIRST_PROPERTY_ADDRESS;
        }

    } else if (this->rootLayer->Vector() == VECTOR_ROOT_E131_EXTENDED) {

    } else {
        throw e131_error_t::ERROR_VECTOR_ROOT;
    }
    this->universerDiscoveryLayer = nullptr;
}