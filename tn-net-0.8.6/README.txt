http://tnkernel.com/tn_net_1.html

1. Introduction

    TN NET TCP/IP stack was designed for embedded processors with RAM size 32 KBytes and more and FLASH size 128 KBytes and more (for instance, NXP (c) LPC23xx/LPC17XX, Atmel (c) AT91SAM7X, Luminary Micro (c) LM3S6xx/LM3S8xx/LM3S9xx, Freescale (c) MCF5223X and many other similar MCU).

    TN NET TCP/IP stack protocols implementation is based on the industry-standard BSD TCP/IP stack.
    A BSD sockets are used as the API (user interface).
   TN NET TCP/IP stack uses a TNKernel RTOS synchronization elements to provide a true multitasking reentrant stack.

2. TN NET TCP/IP stack executive model

    TN NET interface (an Ethernet interface is assumed here) provides the data sending/receiving procedure. An ARP protocol support is a part of the interface software. The interface contains a reception task and two TNKernel data queues - Interface Rx Queue and Interface Tx Queue. An Ethernet driver after reception from a wire puts (inside the Rx interrupt handler) the packet into the Interface Rx Queue. The IP or ARP protocols, as data senders, puts a packet to send into Interface Tx Queue directly. This event starts a transmitting, if transmitting is not yet active. If transmitting is already active (in progress), the End of Transmit interrupt handler will restart transmitting for the next packet from the Interface Tx Queue.

   An interface reception task works with Interface Rx Queue. If the queue is not empty, the reception task invokes appropriate protocol input functions (IP or ARP). According to the type of the received packet, IP input function calls a related protocol input handler (UDP/TCP/ICMP etc) for the further processing. For instance, UDP protocol input function adds a received packet to the
user's socket input queue (it is a TNKernel data queue also) and user's socket task will be wake-up. If there are no open UDP sockets for the received packet destination port, UDP will respond to the sender with ICMP error message.

3. TN NET TCP/IP stack implementation details

  3.1 Memory buffers

   TN NET TCP/IP stack uses 3 types of fixed-sized memory pools - with a 32, 128 and 1536 bytes elements memory size.
   A memory buffer in TN NET TCP/IP stack contains two items - descriptor and data buffer. A memory for each item is allocated separately.

   A descriptor size is 32 bytes, data buffer size can be 32, 128 or 1536 bytes. An allocation a separate memory block for the descriptor makes a memory buffers system very flexible (just a few examples):

             - for the zero-copy operation it is enough allocate a new descriptor only.

             - in the NXP LPC23xx CPU an Ethernet MAC has access only to the Ethernet RAM
               (16 kbyte). In this case, data buffers can be allocated in the Ethernet RAM
               and descriptors can be allocated in the regular RAM.

             - data buffer can be placed into the read-only memory.
   3.2 Protocols

   An IP protocol in the TN NET TCP/IP stack supports reassembly for the input packets and fragmentation for the output packets.

   An ICMP implementation is very basic - reflect/ping response and ICMP error sending.

   The stack uses BSD protocol control blocks (PCB) to store internal data for UDP/TCP protocols.

   3.3 Sockets

   In TN NET TCP/IP stack, an API (user interface) is a BSD sockets.
  The API functions, supported by this software, are a subset of the BSD socket API, supplied with UNIX compatible systems.
  The function names consist of BSD socket function names with a prefix 's_'

  Supported socket functions
Function	Supported protocols
s_socket()	TCP, UDP, RAW
s_close()	TCP, UDP, RAW
s_bind()	TCP, UDP, RAW
s_connect()	TCP, UDP, RAW (UDP, RAW - to set a peer address only)
s_accept()	TCP
s_listen()	TCP
s_recv()	TCP
s_recvfrom()	UDP, RAW
s_send()	TCP
s_sendto()	UDP, RAW
s_ioctl()	TCP, UDP
s_shutdown()	TCP
s_getpeername()	TCP, UDP, RAW
s_getsockopt()	TCP, UDP, RAW
s_setsockopt()	TCP, UDP, RAW
4. TN NET TCP/IP stack usage

   These two file are used to set the stack configuration:

                   tn_net_cfg.h   - sets CPU endian, buffers number etc.

                   lpc23xx_net.c - sets MAC address, IP addresses and mask

   The examples source code can be used as reference - how to initialize TN NET TCP/IP stack, create sockets, send/receive data etc.

    4.1 UDP Examples

     "UDP_Test_1" - This is a UDP data acquisition example. The device sends a packet to the host, host sends back an ACK response.
      This example works together with a PC-based "UDP_daq" This is a TDTP protocol demo (it uses UDP as a transport layer protocol).

     "UDP_Test_2" - the example works together with a PC-based "UDP_server" application.
There are 2 network related user tasks in the project - TASK_PROCESSING and TASK_DAQ. Each task has an own UDP socket to transfer data. Both tasks have an equal priority and perform a UDP data exchange simultaneously.
   A TASK_PROCESSING task sends a on-the-fly generated file (~4Mbytes) to the "UDP_server" application, when a "Receive to file" radio button is selected in the "UDP_server" dialog window.
   When "Send file" radio button is selected in the "UDP_server" dialog window, a file from the PC (user should choose the file) is transferred to the device.
   Both transfers are performed with TDTP protocol.

  A TASK_DAQ task simulates a data acquisition process by the "UDP_server" application request. After a request from the "UDP_server" application, the task sends a data to the "UDP_server".
   The transfer is performed with TDTP protocol.

    4.2 TCP Examples

  An examples "TCP_test_1", "TCP_test_2" and "TCP_test_3" show TN NET TCP protocol client operating - reception, transmission and both reception and transmission. The examples work together with a PC applications TCP_test_1", "TCP_test_2" and "TCP_test_3".
  An examples "TCP_test_4", "TCP_test_5" and "TCP_test_6" show TN NET TCP protocol server operating - reception, transmission and both reception and transmission. The examples work together with a PC applications TCP_test_4", "TCP_test_5" and "TCP_test_6".
  An examples "TCP_test_7" shows TN NET TCP server's sockets opening/closing.
  An examples "HTTP_test_1" shows TN NET simple embedded Web server. The Web site files for the server should be prepared by the "htm_to_c.exe" utility ( http-pc-1-0-src.zip ).
 The Web server supports forms and dynamic data updating.
