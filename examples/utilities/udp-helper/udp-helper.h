/* udp-helper.h
 *
 * See udp-helper.c.
 */

/* Only include this file once. */
#pragma once

/* Includes */
#include <netinet/in.h>

/* 65,507 is the maximum size in IP4. */
#ifndef SOCKET_BUFFER_LENGTH
#define SOCKET_BUFFER_LENGTH 16384
#endif

typedef enum {
    UDPHelperUninitialized=0,
    UDPHelperBadSock,
    UDPHelperCouldNotBind,
    UDPHelperReady
} UDPStatus;

typedef struct {
    UDPStatus status;
    struct sockaddr_in our_sock;
    struct sockaddr_in target_sock;
    int id;
    long packets_received;
    long packets_sent;
    char input_buffer[SOCKET_BUFFER_LENGTH];
    char output_buffer[SOCKET_BUFFER_LENGTH];
    int read_head;
    int write_head;
    int respond_to_sender;
} UDPHelper;

/* Create a UDPHelper. */
UDPStatus udp_helper(UDPHelper * helper, int listen_port, int target_port, int respond_to_sender, const char * ip_address);

/* Wait for a UDP packet to arrive and copy it into the input buffer. The data will be available via udp_pull. */
int udp_receive(UDPHelper * helper);

/* Pull some amount of data out of the read buffer. */
int udp_pull(UDPHelper * helper, Nothing * data, int length);

/* Skip length bytes in the input buffer. */
int udp_burn(UDPHelper * helper, int length);

/* Put new data into the send buffer. */
int udp_push(UDPHelper * helper, Nothing * data, int length);

/* Send the queued packet. */
int udp_transmit(UDPHelper * helper);

/* Reset the write head. */
Nothing udp_flush_output(UDPHelper * helper);

/* Close the socket whwen done. */
Nothing udp_close(UDPHelper * helper);
