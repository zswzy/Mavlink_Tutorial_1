/*****************************************************************
 * @file   samll_udp.h
 * @brief  A very simple header-only library for udp communication in Windows.
 * 
 * @author zswzy
 * @date   20230529
 *********************************************************************/
#ifndef SMALL_UDP_H
#define SMALL_UDP_H


#include <stdio.h>
#include <string>
#include <WS2tcpip.h>

#pragma comment (lib, "ws2_32.lib")

class UdpCom {
private:
	SOCKET			m_in;

	sockaddr_in		m_local;
	const char*		m_local_ip;
	int				m_local_port;
	int				m_local_length;

	sockaddr_in		m_dest;
	const char*		m_dest_ip;
	int				m_dest_port;
	int				m_dest_length;
	char			m_buffer[4096];

public:
	/**
	 * @brief The UDP class constructor.
	 * 
	 * @return 
	 */
	UdpCom()
	{
		m_in			= NULL;
		m_local			= sockaddr_in();
		m_local_length  = sizeof(m_local);
		m_local_ip		= "127.0.0.1";
		m_local_port	= 0;
		m_dest			= sockaddr_in();
		m_dest_length	= sizeof(m_dest);
		m_dest_ip		= "0.0.0.0";
		m_dest_port		= 0;
		memset(m_buffer, 0, sizeof(m_buffer));

		ZeroMemory(&m_dest, sizeof(m_dest));
	}

	/**
	 * @brief destructor.
	 * 
	 */
	~UdpCom()
	{
		Close();
	}

	/**
	 * @brief Open UDP.
	 * 
	 * @return 0: normal, 1: Winsock error, 2: bind error.
	 */
	int Open()
	{
		int return_code = 0;

		// Startup Winsock 开启winsock
		WSADATA data;
		WORD version = MAKEWORD(2, 2);
		int wsOK = WSAStartup(version, &data);
		if (wsOK != 0) {
			printf("can't start Winsock. Error: %d.\n", wsOK);
			return_code = 1;
		}

		// Bind socket 绑定本地端口号
		m_in = socket(AF_INET, SOCK_DGRAM, 0);
		m_local.sin_family = AF_INET;
		m_local.sin_port = htons(m_local_port); // Convert from little to big endian
		inet_pton(AF_INET, m_local_ip, &m_local.sin_addr);

		// 检测是否成功绑定
		if (bind(m_in, (sockaddr*)&m_local, sizeof(m_local)) == SOCKET_ERROR)
		{
			printf("Can't bind socket! Error: %d.\n", WSAGetLastError());
			return_code = 2;
		}

		// 绑定远端
		m_dest.sin_family = AF_INET;
		inet_pton(AF_INET, m_dest_ip, &m_dest.sin_addr);
		m_dest.sin_port = htons(m_dest_port);

		printf("UDP is open.\n");
	
		return return_code;
	}

	/**
	 * @brief Open UDP with parameters.
	 *
	 * @param local_ip		local IPv4 address.
	 * @param local_port	local port.
	 * @param dest_ip		remote IPv4 address.
	 * @param dest_port		remote port.
	 * @return 
	 */
	int Open(const char* local_ip, const int local_port, const char* dest_ip, const int dest_port)
	{
		SetLocal(local_ip, local_port);
		SetDest(dest_ip, dest_port);
		Open();
		return 0;
	}

	/**
	 * @brief Receive message from remote.
	 * 
	 * @return 
	 */
	int Receive()
	{
		ZeroMemory(m_buffer, 4096);

		// 接收消息，储存为变量buf，类型为字符串
		int byte_in = recvfrom(m_in, m_buffer, 4096, 0, (sockaddr*)&m_dest, &m_dest_length);

		if (byte_in == SOCKET_ERROR)
		{
			printf("Error receiving from client. Error code: %d", WSAGetLastError());
		}

		printf("%s \n",m_buffer);
		//recv_data = jsonxx::json::parse(m_buffer);

		return 0;
	}

	/**
	 * @brief Send message to remote.
	 * 
	 * @param message_send
	 * @return 0: normal, 1: Error.
	 */
	int Send(char* message_send, int len)
	{
		int return_code = 0;
		//printf("message_send: %s, sizeof: %d\n", message_send, strlen(message_send));

		int send_ok = sendto(m_in, message_send, len, 0, (sockaddr*)&m_dest, sizeof(m_dest));

		if (send_ok == SOCKET_ERROR)
		{
			printf("Send error.\n");
			return_code = 1;
		}
		else
		{
			// std::cout << "Send OK" << std::endl; 
		}

		return return_code;
	}

	/**
	 * @brief Send message to remote.
	 *
	 * @param message_send
	 * @return 0: normal, 1: Error.
	 */
	int Send(char* message_send)
	{
		int return_code = 0;
		//printf("message_send: %s, sizeof: %d\n", message_send, strlen(message_send));

		int send_ok = sendto(m_in, message_send, strlen(message_send), 0, (sockaddr*)&m_dest, sizeof(m_dest));

		if (send_ok == SOCKET_ERROR)
		{
			printf("Send error.\n");
			return_code = 1;
		}
		else
		{
			// std::cout << "Send OK" << std::endl; 
		}

		return return_code;
	}

	/**
	 * @brief Close UDP. It is called in the destructor.
	 * 
	 * @return 
	 */
	int Close()
	{
		// Close socket
		closesocket(m_in);

		// Shutdown winsock
		WSACleanup();

		return 0;
	}

	/**
	 * @brief Set remote parameters: IP and port.
	 * 
	 * @param dest_ip	remote IPv4 address.
	 * @param dest_port remote port.
	 * @return 
	 */
	int SetDest(const char* dest_ip, const int dest_port)
	{
		m_dest_ip = dest_ip;
		m_dest_port = dest_port;

		return 0;
	}

	/**
	 * @brief Set local parameters: IP and port.
	 * 
	 * @param local_ip		local IPv4 address.
	 * @param local_port	local port
	 * @return 
	 */
	int SetLocal(const char* local_ip, const int local_port)
	{
		m_local_ip = local_ip;
		m_local_port = local_port;
		return 0;
	}

	/**
	 * @brief Get received message.
	 * 
	 * @return 
	 */
	char* GetContent()
	{
		return m_buffer;
	}
};

#endif

