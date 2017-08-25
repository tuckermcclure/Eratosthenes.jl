/*
 * pd_controller_pitl
 *
 * This code runs the pd_controller on the target flight computer, using UDP to
 * get inputs from the simulated sensors and to send commands back.
 *
 * Build with:
 *
 * Linux/macOS: gcc -o linux/pd-controller-pitl pd-controller.c pd-controller-pitl.c ../utilities/udp-helper/udp-helper.c -lm
 *
 */

#include <stdio.h>
#include <stdint.h>
#include "../utilities/udp-helper.h"

#include "pd-controller.h"

int main(void)
{
    UDPHelper comms;
    const char * target_ip_address = "127.0.0.1"; /* Dummy; we're respond to whomever talks to us first. */
    int listen_port = 2000;
    int target_port = 2001;
    int respond_to_sender = 1;
    uint64_t status = 0;
    double gains[2]  = {0., 0.};
    double q_TI[4]   = {0., 0., 0., 0.};
    double q_BI[4]   = {0., 0., 0., 0.};
    double w_BI_B[3] = {0., 0., 0.};
    double f_B[3]    = {0., 0., 0.};
    double tau_B[3]  = {0., 0., 0.};
    int bytes_received = 0;
    int bytes_sent = 0;

    /* Create a socket and listen for incoming UDP messages. */
    udp_helper(&comms, listen_port, target_port, respond_to_sender, target_ip_address);

    /* Loop until the first 8 bytes of the message tell us not to any more. */
    while (1)
    {
        /* Pull the data in. */
        printf("Waiting for new data.\n");
        bytes_received = udp_receive(&comms);
        printf("Received %d bytes.\n", bytes_received);
        udp_pull(&comms, &status, sizeof(status));
        udp_pull(&comms, gains,   sizeof(gains));
        udp_pull(&comms, q_TI,    sizeof(q_TI));
        udp_pull(&comms, q_BI,    sizeof(q_BI));
        udp_pull(&comms, w_BI_B,  sizeof(w_BI_B));
        printf("w1=%5.2f, w2=%5.2f, w3=%5.2f\n", w_BI_B[0], w_BI_B[1], w_BI_B[2]);

        /* See if we should quit. */
        if (status <= 0) { break; }

        /* Run the target code. */
        pd_controller(gains, q_TI, q_BI, w_BI_B, f_B, tau_B);

        /* Push the data back out. */
        udp_push(&comms, f_B,   sizeof(f_B));
        udp_push(&comms, tau_B, sizeof(tau_B));
        bytes_sent = udp_transmit(&comms);
        printf("Sent %d bytes.\n", bytes_sent);
    }

    udp_close(&comms);
    printf("Done.\n");
    return 0;
}
