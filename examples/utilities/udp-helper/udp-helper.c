/* udp-helper.c
 *
 * This utility is useful for putting together quick, blocking UDP send and receive processes on Linux.
 *
 * Basic uses:
 *
 * UDPHelper comms;
 * udp_helper(comms, listen_post, target_pot, respond_to_sender, ip_address); // Construct a UDPHelper and open the socket.
 * udp_receive(comms); // Block until a packet arrives.
 * udp_pull(comms, &data, sizeof(data)); // Pull packet data.
 * udp_push(comms, &data2, sizeof(data2)); // Queue up some data to be sent.
 * if (udp_transmit(comms) == -1) // Send whatever's in the buffer.
 * {
 *     udp_flush_output(comms); // We could try to send again, but instead we'll give up and flush the output buffer.
 * }
 * udp_close(comms); // Make sure this gets called to keep resource utilitization tidy.
 *
 * Reference: https://www.abc.se/~m6695/udp.html
 */

/* Includes */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
/*#include <byteswap.h>*/
#include "udp-helper.h"

/* Create a UDPHelper. */
UDPStatus udp_helper(UDPHelper * helper, int listen_port, int target_port, int respond_to_sender, const char * ip_address)
{
    /* Initialize the socket. */
    helper->status = UDPHelperUninitialized;
    helper->id = 0;
    helper->packets_received = 0;
    helper->packets_sent = 0;
    helper->read_head = 0;
    helper->write_head = 0;
    helper->respond_to_sender = 0;
    memset(&(helper->input_buffer),  0, SOCKET_BUFFER_LENGTH * sizeof(char));
    memset(&(helper->output_buffer), 0, SOCKET_BUFFER_LENGTH * sizeof(char));
    memset(&(helper->our_sock),      0, sizeof(helper->our_sock));
    memset(&(helper->target_sock),   0, sizeof(helper->target_sock));

    /* Create the socket. */
    helper->id = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (helper->id == -1) { return UDPHelperBadSock; }

    /* Set up addressing. */
    helper->our_sock.sin_family      = AF_INET; /* Use internet addresses. */
    helper->our_sock.sin_port        = htons(listen_port); /* Use host-to-network order, short integer */
    helper->our_sock.sin_addr.s_addr = htonl(INADDR_ANY); /* Use any address. */

    /* Try to bind the socket. */
    if (bind(helper->id, (struct sockaddr*) &(helper->our_sock), sizeof(helper->our_sock)) == -1) { return UDPHelperCouldNotBind; }

    /* If we're not just replying to whatever sender, then record who we are suppose to send to. */
    helper->respond_to_sender = respond_to_sender;
    if (!respond_to_sender)
    {
        helper->target_sock.sin_family = AF_INET;
        inet_aton(ip_address, &(helper->target_sock.sin_addr));
    }

    /* Record the port to send to. */
    helper->target_sock.sin_port = htons(target_port);

    /* We were successful! */
    return UDPHelperReady;
}

/* Wait for a UDP packet to arrive and copy it into the input buffer. The data will be available via udp_pull. */
int udp_receive(UDPHelper * helper)
{
    struct sockaddr_in sender_sock;

    /* Block until a UDP packet arrives. */
    socklen_t length = (socklen_t)sizeof(sender_sock); /* Convert to a socklen_t explicitly. */
    int bytes_read = recvfrom(helper->id,
                              helper->input_buffer,
                              SOCKET_BUFFER_LENGTH,
                              0,
                              (struct sockaddr*) &sender_sock,
                              &length);

    /* If we didn't get anything, tell the user. */
    if (bytes_read == 1) { return -1; }

    /* Keeping track of the number of received packets is useful for debugging. */
    helper->packets_received += 1;

    /* Start the read head at the beginning of the buffer. */
    helper->read_head = 0;

    /* Record the sender (if necessary). */
    if (helper->respond_to_sender)
    {
        helper->target_sock.sin_family      = sender_sock.sin_family;
        helper->target_sock.sin_addr.s_addr = sender_sock.sin_addr.s_addr;
        /* helper->target_sock.sin_port        = sender_sock.sin_port; If we want to send back to the same port... */
    }

    return bytes_read;
}

/* Pull some amount of data out of the read buffer. */
int udp_pull(UDPHelper * helper, Nothing * data, int length)
{
    /* Only read if there's room. Otherwise, the user is asking for more than we have. */
    if (helper->read_head + length > SOCKET_BUFFER_LENGTH) { return -1; }

    /* Copy the data into the space the user has allocated. */
    memcpy(data, (Nothing *) &(helper->input_buffer) + helper->read_head, length);
    helper->read_head += length;

    /* Return how much we have read from the buffer so far. */
    return helper->read_head;
}

/* Skip length bytes in the input buffer. */
int udp_burn(UDPHelper * helper, int length)
{
    /* Only read if there's room. Otherwise, the user is asking for more than we have. */
    if (helper->read_head + length > SOCKET_BUFFER_LENGTH) { return -1; }
    helper->read_head += length;
    return helper->read_head;
}

/* Put new data into the send buffer. */
int udp_push(UDPHelper * helper, Nothing * data, int length)
{
    /* Make sure there's space left in the buffer for what the user is adding. */
    if (helper->write_head + length > SOCKET_BUFFER_LENGTH) { return -1; }

    /* Copy the data from the space the user has allocated. */
    memcpy((Nothing *) &(helper->output_buffer) + helper->write_head, data, length);
    helper->write_head += length;

    /* Return how much we have written to the buffer so far. */
    return helper->write_head;
}

/* Send the queued packet. */
int udp_transmit(UDPHelper * helper)
{
    /* Send the packet. */
    int bytes_sent = sendto(helper->id,
                            helper->output_buffer,
                            helper->write_head,
                            0,
                            (struct sockaddr*) &(helper->target_sock),
                            sizeof(helper->target_sock));

    /* Only reset the write head if everything worked. */
    if (bytes_sent != -1) { helper->write_head = 0; }

    /* Tell the user what we accomplished. */
    return bytes_sent;
}

/* Reset the write head. */
Nothing udp_flush_output(UDPHelper * helper)
{
    helper->write_head = 0;
}

/* Close the socket whwen done. */
Nothing udp_close(UDPHelper * helper)
{
    close(helper->id);
}
