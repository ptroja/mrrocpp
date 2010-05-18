/***************************************************************************
 *   Copyright (C) 2006 by Sven Wünschmann                                 *
 *   sven.wuenschmann@web.de                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/



#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "C_CPN.h"

#define QUEUE_SIZE 10
// Maximum size of one packet in bytes; used for message's segmentation
#define PACKET_SIZE 128
// Header's length is 1 byte
#define PAYLOAD_SIZE 127



int socketfd = -1;

typedef struct bufferlist_node {
	char* buffer;
	unsigned short size;
	struct bufferlist_node* next;
} bufferlist_node_t;





/** Establish a connection to an external process actively.
 *
 *  \param hostName Host to connect to.
 *  \param port The port number, the server is listening to.
 *
 *  \return Success of operation is indicated by
 *
 *          - 0: connection could be established
 *          - -1: error while connecting, errno is set to the appropriate value
 *          - -2: error while looking up host, h_errno is set
 *          - -3: error while setting up socket
 *          - -4: error, we are already connected
 */
int CPN_connect(const char* hostName, unsigned short port)
{
	// exit, if we are already connected. multiple connections not supported
	if (socketfd >= 0) return -4;
	
	struct hostent *h;
	struct sockaddr_in channel;
	
	h = gethostbyname(hostName);
	if (!h) return -2;
	
	socketfd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);	
	if (socketfd < 0) return -3;
	
	memset(&channel, 0, sizeof(channel));
	channel.sin_family = AF_INET;
	memcpy(&channel.sin_addr.s_addr, h->h_addr, h->h_length);
	channel.sin_port = htons(port);
	
	return connect(socketfd, (struct sockaddr *) &channel, sizeof(channel));
}

/** Establish a connection with an external process passively, i.e. listen for
 * calling clients. Our Server listens on all interfaces.
 *
 *  \param port Specifies the port the server listens to.
 *
 *  \return Success of operation is indicated by
 *
 *          - 0: connection could be established
 *          - 1: server socket could not be created
 *          - 2: server socket could not be bound, errno set according to man 2 bind
 *          - 3: listen() failed, errno set according to man 2 listen
 *          - 4: accept() failed
 */
int CPN_accept(unsigned short port)
{
	struct sockaddr_in channel;
	
	/* Build address structure to bind to socket */
	memset(&channel, 0, sizeof(channel)); // zero channel
	channel.sin_family = AF_INET;
	channel.sin_addr.s_addr = htonl(INADDR_ANY); // server listens on all avail interfaces
	channel.sin_port = htons(port);
	
	/* Passive open. Wait for connection. */
	int serversocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serversocket < 0) return -1;
	
	int on = 1;
	setsockopt(serversocket, SOL_SOCKET, SO_REUSEADDR, (char *) &on, sizeof(on));
	
	int b = bind(serversocket, (struct sockaddr *) &channel, sizeof(channel));
	if (b < 0) return -2;
	
	int l = listen(serversocket, QUEUE_SIZE);
	if (l < 0) return -3;
	
	/* Socket is now set up and bound. Wait for incoming connection. */
	socketfd = accept(serversocket, 0, 0);
	if (socketfd < 0) return -4;
	
	close(serversocket);
	
	return 0;
}

/** Send data to an external process. The segmentation into packets occurs
 * in this function. Bytes are read from the C-string, a maximum
 * of 127 at a time, and a single byte header is added indicating the
 * number of payload bytes (header is 1 to 127) or that there is more data 
 * in a following packet (header is -1). The data packets formed are then
 * transmitted to the external process through low-level I/O-functions.
 *
 * \param data Pointer to the null-terminating string that is to be sent.
 *
 * \return If successful, number of bytes sent will be returned, or
 *          - -1 if function fails. errno will be set then
 *          - -2 if there is no connection yet.
 *
 * Note: Due to packaging, the number of sent bytes will be greater
 *       than sizeof(data).
 */
int CPN_send(const char* data)
{
	if (socketfd < 0) return -2;
	
	char* packet = malloc(PACKET_SIZE);
	int i = 0, total_bytes_sent = 0;
	int data_length = strlen(data);
	
	/* Send chained, maximum-length packets */
	while ( data_length > PAYLOAD_SIZE )
	{
		packet[0] = (char)-1;
		memcpy(&packet[1], &data[i], PAYLOAD_SIZE);
		
		i += PAYLOAD_SIZE;
		data_length -= PAYLOAD_SIZE;
		
		int bytes_transmitted = 0;
		do {
			int s = write(socketfd, &packet[bytes_transmitted],
				      PACKET_SIZE - bytes_transmitted);
			if (s < 0) {
				//printf("debug: CPN_send() could not write full packet\n");
				return -1;
			}
			bytes_transmitted += s;
		} while (bytes_transmitted < PACKET_SIZE);
		
		total_bytes_sent += bytes_transmitted;
	}
	
	/* Send the transmission's last packet */
	// data_length is [0,...,127] now
	packet[0] = (char)data_length;
	memcpy(&packet[1], &data[i], data_length);
	
	int bytes_to_transmit = data_length + 1; // 1 header-byte
	int bytes_transmitted = 0;
	do {
		int s = write(socketfd, &packet[bytes_transmitted],
			      bytes_to_transmit - bytes_transmitted);
		if (s < 0) {
			return -1;
		}
		bytes_transmitted += s;
	} while(bytes_transmitted < bytes_to_transmit);
	
	total_bytes_sent += bytes_transmitted;
	
	free(packet);
	return total_bytes_sent;
}

/** Receive data from an external process. The function is
 * blocking until some data is received or an error occurs (e.g. socket
 * closed or invalid packet received).
 *
 * \return A null-terminating string, representing the received data,
 *         NULL if function fails; errno might be set then.
 */
char* CPN_receive()
{
	int total_payload_size = 0;
	
	bufferlist_node_t* head = malloc(sizeof(bufferlist_node_t));
	bufferlist_node_t* current_node = head;
	
	/* Read the transmission, maybe consisting of multiple packets	*/
	char header;
	char packet_payload_size = PAYLOAD_SIZE;
	do
	{	
		// read header of incoming packet
		int s = read(socketfd, &header, sizeof(char));
		if (s <= 0) {
			return NULL;
		}
		
		if (header >= 0) { // header indicates the last packet
			packet_payload_size = header;
		}

		// read content of incoming packet
		current_node->buffer = malloc(packet_payload_size);
		int packet_read = 0;
		do
		{
			s = read(socketfd, 
				     &current_node->buffer[packet_read],
				     packet_payload_size - packet_read);
			if (s <= 0) {
				return NULL;
			}
		
			packet_read += s;
		} while (packet_read < packet_payload_size);
		
		total_payload_size += packet_payload_size;
		
		// we could read the packet, so fill in data into the node of the list
		current_node->size = packet_payload_size;
		current_node->next = malloc(sizeof(bufferlist_node_t));
		current_node = current_node->next;
		current_node->next = NULL; // terminate the list, if no element is appended
		
	} while (header < 0);  // we expect a further packet in that transmission
	
	/* Build the return-string from the buffers in the linked-list
	 * and free the list
	 */
	char* data = malloc(total_payload_size + 1);
	int i = 0;
	current_node = head;
	
	while (current_node->next != NULL) // not last element of list
	{
		memcpy(&data[i], current_node->buffer, current_node->size);
		
		i += current_node->size;
		
		free(current_node->buffer);
		
		bufferlist_node_t* tmp = current_node;
		current_node = current_node->next;
		free(tmp);
	}
	// current is the last element of the list after the while-loop
	free(current_node);
	
	// terminate the string
	data[total_payload_size] = '\0';
	
	return data;
}

/** Close a connection to an external process.
 *
 * \return 0 if successful, -1 otherwise
 */
int CPN_disconnect()
{
	int s = close(socketfd);
		
	if (s == 0) socketfd = -1; // everything went fine, reset socketfd
	return s;
}
