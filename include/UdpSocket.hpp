#include <cstring>
#include <unistd.h>       
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



class UdpSocket
{

public:
	UdpSocket(int local_port, const char* remote_ip, int remote_port){
		sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);	// Creazione del socket
		if (sockfd_ < 0)
		{
			// logging
			return;
		}

		serverAddr_.sin_family = AF_INET;
		serverAddr_.sin_port = htonl(local_port);
		serverAddr_.sin_addr.s_addr = htonl(INADDR_ANY);

		if (bind(sockfd_, (const struct sockaddr *)&serverAddr_, sizeof(serverAddr_)) < 0)
		{
			// Logging
			return;
		}
		
		clientAddr_.sin_family = AF_INET;
		clientAddr_.sin_port = htons(remote_port);
		inet_pton(AF_INET, remote_ip, &clientAddr_.sin_addr);
	};

	~UdpSocket() = default;

	int send(const uint8_t *buf, int len){
		return sendto(sockfd_, buf, len, 0, (const struct sockaddr *)&clientAddr_, sizeof(clientAddr_));
	}

	int recive(uint8_t *buf, int max_len){
		return recv(sockfd_, buf, max_len, 0);
	}


private:
	int sockfd_{0};
	struct sockaddr_in serverAddr_{};
	struct sockaddr_in clientAddr_{}; 

};



