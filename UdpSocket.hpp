#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

typedef int  _socket_t;



namespace Socket
{
    class UdpSocket
    {
    public:
        UdpSocket(int port = 0);
        ~UdpSocket();
        bool Open();
        bool IsOpen() const;
        void Close();
        int Write(const void *buffer, unsigned int size, struct sockaddr *dest_addr);
        int Read(void *buffer, unsigned int size, struct sockaddr *source_addr);
        int GetPort() const;
        
    private:
        struct sockaddr socket_addr;
        int port;
        _socket_t handle;
        bool isOpen;
    };
}
