#ifndef __ILINTERFACE_COMM__H_
#define __ILINTERFACE_COMM__H_

class ILInterfaceComm
{
public:
    ILInterfaceComm() {};
    virtual ~ILInterfaceComm() = default;

    virtual bool interfaceCommPrint(const char *command, ...)                                   = 0;
    virtual void processPacket(unsigned char *data, unsigned char len)                          = 0;
    virtual void printToInterface(const char * format, ...)                                     = 0;
    virtual void send1Byte(unsigned char msg_type, unsigned char data)                          = 0;
    virtual void send2Bytes(unsigned char msg_type, unsigned char dataH, unsigned char dataL)   = 0;

};

#endif // __ILINTERFACE_COMM__H_