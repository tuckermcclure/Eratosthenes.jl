/* udp-example.c
 *
 * An example of how to use udp-helper to listen on a port and send data back.
 *
 * Build with:
 *
 * Linux: gcc -o udp-example udp-example.c udp-helper.c
 */

#include <stdio.h>
#include "udp-helper.h"

int main(void)
{
    UDPHelper comms;
    const char * target_ip_address = "127.0.0.1";
    int listen_port = 2000;
    int target_port = 2001;
    int respond_to_sender = 1;
    long x = 0;
    double y = 0.;
    int bytes_received = 0;
    int bytes_sent = 0;

    printf("udp_helper: %d\n", udp_helper(&comms, listen_port, target_port, respond_to_sender, target_ip_address));

    while (x >= 0)
    {

        bytes_received = udp_receive(&comms);
        printf("Received %d bytes.\n", bytes_received);
        udp_pull(&comms, &x, sizeof(x));
        udp_pull(&comms, &y, sizeof(y));
        printf("x=%ld, y=%f\n", x, y);

        if (x < 0) { break; }

        x += 1;
        y += 1.;
        udp_push(&comms, &x, sizeof(x));
        udp_push(&comms, &y, sizeof(y));
        bytes_sent = udp_transmit(&comms);
        printf("Sent %d bytes.\n", bytes_sent);
    }

    udp_close(&comms);
    printf("Done.\n");
    return 0;
}
