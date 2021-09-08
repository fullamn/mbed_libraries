#include <iostream>
#include <cstddef>
#include <string>
#include <stdint.h>
#include <stdio.h>

#ifndef BITOMEROTARYVALVE
#define BITOMEROTARYVALVE


/********
OVERVIEW
    Builds the RotaryValve class. (All values should be in hex)
    The rotary valve currently in use is the ERV-06 Valve(documentation on OneNote/Gingko Project/Hardware and Software)
    Frame structure:
        STX|ADDR|FUNC|PARAML|PARAMH|ETX|SUML|SUMH

        Values:
            STX(start): Start byte of command packet (from rotary valve documentation)
            ADDR(address): Address of rotary valve connected to system
            FUNC(func): Function to be called by the rotary valve (See VALID_FUNC below)
            PARAML/H(param): Parameter of called function (See VALID_FUNC below)
            ETX(end): End byte of command packet (from rotary valve documentation)
            SUML/H(checksum): Checksum to determine corruption
            buff[8]: 8-byte buffer that is used to store command packet values before being pushed out.

        Functions:
            Constructor: Standard constructor, inputs UART pins, address (0x00 by default), and baud rate (platform default by default).
            buildPacket: Builds command packet and stores them in the buffer.
            getBuff: Returns a pointer equivalent to &buff.
            getMessage: Returns a pointer equivalent to &message. Useful for getting error codes from valve/ status from valve.
UNF         sendPacket: Sends a packet to via established pins, waits for valve to respond and puts the response into message
            Verify: Ensures checksum is valid. Use before sending packet.
            Move: Updates func and param to be valid move commands. Moves the valve so the center is connected to output Position
            Home: Updates func and param to be the home command. When sent, moves the valve to the home position.
********/

/********
TYPE DEFINITIONS
********/

/* Action: stores function and parameter in valid pairs of the format:
FUNC|PARAML|PARAMH */
typedef enum {
    STATUS_VALVE = ((0x3E << 16) | 0x0000),
    STATUS_MOTOR = ((0x4A << 16) | 0x0000)
} Action;

typedef enum {
    ONE = (0x0001),
    TWO = (0x0002),
    THREE = (0x0003),
    FOUR = (0x0004),
    FIVE = (0x0005),
    SIX = (0x0006),
    SEVEN = (0x0007),
    EIGHT = (0x0008),
    NINE = (0x0009),
    TEN = (0x000A),
    ELEVEN = (0x000B),
    TWELVE = (0x000C),
} Position;

/********
CLASSES
********/

class RotaryValve
{
private:

    PinName tx;
    PinName rx;
    uint32_t baudrate;
    uint8_t start;
    uint8_t address;
    uint8_t func;
    uint16_t param;
    Action action;
    uint8_t end;
    uint16_t checksum;
    unsigned char buff[8],message[8];

public:

    bool isHome = false;

    RotaryValve(PinName TX, PinName RX, uint8_t adrs = 0x00, uint32_t baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE)
    {
        tx = TX;
        rx = RX;
        address = adrs;
        baudrate = baud;
    };

    void buildPacket()
    {
        start = 0xCC;
        end = 0xDD;
        checksum = start + address + func + param + end;
        uint8_t paramH= (param & 0xFF00) >> 8;
        uint8_t paramL= (param & 0xFF);
        uint8_t sumH = (checksum & 0xFF00) >> 8;
        uint8_t sumL = (checksum & 0xFF);

        buff[0]= start;
        buff[1]= address;
        buff[2]= func;
        buff[3]= paramL;
        buff[4]= paramH;
        buff[5]= end;
        buff[6]= sumL;
        buff[7]= sumH;
    }

    unsigned char* getBuff()
    {
        static unsigned char packet[8];
        for(char i = 0; i<sizeof(buff); i++) {
            packet[i]=buff[i];
        }

        return packet;
    }

    unsigned char* getMessage()
    {
        static unsigned char packet[8];
        for(char i = 0; i<sizeof(message); i++) {
            packet[i]=message[i];
        }

        return packet;
    }

    bool Verify()
    {
        if(checksum!=(start + address + func + param + end)) {
            checksum = start + address + func + param + end;
        }

        if(checksum==(start + address + func + param + end)) {
            return true;
        } else {
            return false;
        }
    }

    void sendPacket()
    {
        if(Verify()) {
            bool listening = true;
            BufferedSerial out(tx, rx,baudrate);
            out.enable_output(1);
            out.enable_input(1);
            out.write(&buff, 8);
            wait_us(1000);
            std::memset(&message,0,8);
            do {
                out.read(&message,1);
                if(message[0]==0xCC) {
                    //printf("--\n"); //If I don't print at least two char and a return the error code doesn't properly read.
                    //I have no idea why this happens.
                    //Will continue work on this some other time -Nate 6/9/21
                    out.read(&(message[1]),7);
                    listening = false;

//                    This was a test printf to check if the code was received correctly
//                    likely will reimplement in a different function for error reporting

                    /*                    if(message[2]!= 0x00) {
                                            printf("Error \n");
                                        }*/
                }
            } while(listening) ;
            wait_us(1000);
        }
    }

    void Move(Position position)
    {
        func = 0x44;
        param = position;
        buildPacket();
        sendPacket();
        isHome = false;
    }

    void Home()
    {
        func = 0x45;
        param = 0x00;
        buildPacket();
        sendPacket();
        isHome = true;
    };
};

#endif //BITOMEROTARYVALVE