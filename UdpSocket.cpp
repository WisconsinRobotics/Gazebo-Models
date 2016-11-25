#include "UdpSocket.hpp"
#include <cstring>

using namespace Socket;

constexpr _socket_t INVALID_SOCKET_HANDLE = ((_socket_t)(-1));

UdpSocket::UdpSocket(int port)
{
    this->port = port;
    this->isOpen = false;
}

UdpSocket::~UdpSocket()
{
    if (this->isOpen)
        this->Close();
}

bool UdpSocket::Open()
{
    _socket_t sock_handle;
    struct sockaddr_in socket_addr;


#define OPEN_FAIL() { return false; }


    sock_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_handle == INVALID_SOCKET_HANDLE)
        OPEN_FAIL();

    std::memset(&socket_addr, 0, sizeof(struct sockaddr_in));
    socket_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = htons(port);

    if (bind(sock_handle, (struct sockaddr *) &socket_addr, sizeof(struct sockaddr)) != 0)
        OPEN_FAIL();

    this->handle = sock_handle;
    this->isOpen = true;
    return true;
    #undef OPEN_FAIL
}

void UdpSocket::Close()
{
    if (!this->isOpen)
        return;

    this->isOpen = false;
    
    close(handle);
    
}

int UdpSocket::Write(const void *buffer, unsigned int size, struct sockaddr *dest_addr)
{
    if (buffer == nullptr || dest_addr == nullptr)
        return -1;
    
    return sendto(handle, (const char *)buffer, size, 0, dest_addr, sizeof(struct sockaddr));
}

int UdpSocket::Read(void *buffer, unsigned int size, struct sockaddr *source_addr)
{
    socklen_t source_addr_size;

    if (buffer == nullptr)
        return -1;

    source_addr_size = sizeof(struct sockaddr);
    if (source_addr)
        return recvfrom(handle, (char *)buffer, size, 0, source_addr, &source_addr_size);
    else
        return recvfrom(handle, (char *)buffer, size, 0, nullptr, nullptr);
}

int UdpSocket::GetPort() const
{
    return port;
}

bool UdpSocket::IsOpen() const
{
    return this->isOpen;
}
