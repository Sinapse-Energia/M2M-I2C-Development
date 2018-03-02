# M2M-CAN-Development
CAN development in M2M commboard

# TASKs TO-DO

Southbound functions to be developed

`result config_can(baud_rate, listen?, addreess_list)`
`result write_can_message(can_address, payload_to_be_send)`
`[can_address, payload_received] read_can_bus()`

where addresses and payloads are codified in hexadecimal format

This function should be able to:

1. Check address : Base or Extended
2. Check payload : If it is bigger than 8 bytes, it should divide the message in several messages
3. Create the CAN message
4. Send the CAN message
5. Receive the CAN response
6. Extract the address and the payload of the CAN response
7. Return address and payload
8. Ideally, these subparts should be developed in independent sub functions

Normally we will work with OBDII messages but we want to have a generic function in order to be compatible with any High Layer Protocol over CAN.

These function(s) should be developed in the file **southbound_ec.c**

The main function of this development should be called from **main.c** with different options in order to demonstrate its behaviour and ease the testing.  

# Ressources

* HW-FW document available in the Docs folder
* http://www.oneminuteinfo.com/2014/02/can-bus-obd-ii-explained-for-dummies.html
* https://en.wikipedia.org/wiki/OBD-II_PIDs
