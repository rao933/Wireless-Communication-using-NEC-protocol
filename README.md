# Wireless-Communication-using-NEC-protocol

The goal of this project is to build a device capable of transmitting and receiving from multiple devices on
a 40 kHz modulated IR link.

The virtual COM port connected to UART0 will support these instructions:

Instructions
send “string”

send BYTE 0 [BYTE 1 […] ] Write one or more bytes to the TX FIFO feeding the the IR link
receive Display the last received message in the RX FIFO
address ADD Sets the address (ADD) of a node in EEPROM for subsequent use
set ADD, DATA Sets the value of the data associated with a node’s address ADD
rgb ADD, R, G, B Set the value of the {RGB} set associated with a node’s address ADD
get ADD Gets the value of the data associated with a node’s address ADD
poll Send a poll request to all devices on the bus
ack ON|OFF Set the ACK setting to use in subsequent commands sent

IR data format

A transmission requires a 9ms burst, followed by a 4.5ms pause, following by the following message
bytes using the NEC modulation using in Labs 6 and 7.
DST_ADD The address to which the message is being sent
SRC_ADD The address from which the message is being sent. This is used for the
destination address for nodes acknowledging a message.
ACK|CMD
The ACK bit is set if a acknowledge is requested. The CMD field is 7 bits
in length. Commands are Set (0x00), RGB (0x01), Get (0x10), Get
response (0x11), Poll (0x20)
ARGS
Zero or more bytes of command-specific data:
Set requires a DATA byte, RGB requires R, G, and B bytes, and Get
response requires a DATA byte.
CHECKSUM One’s complement of the sum of the above bytes
