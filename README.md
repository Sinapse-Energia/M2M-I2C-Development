# M2M-CAN-Development
CAN development in M2M commboard

# TASKs TO-DO

Southbound functions to be developed

## CAN configuration

`result config_can(baudRate, listen, address_list)`

*Inputs*

* baudRate (int): Speed to be used in the CAN interface
* listen (bool): Set if the device will work read and write the CAN bus or only the write it
* address_list (List): List of the addresses that will listen the device. This list will contain N CAN addresses if the _listen_ is true. It can contains the broadcast address that indicates the device listen everything. Addresses codfied in HEX

*Output*

* result: Number code indicating success (0) or error (number indicating the error code)

## Write CAN message

`result write_can_message(can_address, payload)`

*Inputs*

* can_address: Address that should contain the CAN message, could be a broadcast address. Address codified in HEX
* payload: Payload to be sent codified in HEX. If bigger than 8 bytes, several CAN messages should be sent


*Output*

* result: Number code indicating success (0) or error (number indicating the error code)


## Read CAN Bus

`can_messages_list read_can_bus()`

*Inputs*

No inputs for this function. Read / Listen the info of the devices with addresses set in _config_can_

*Output*

* can_messages_list (List of struct): Return a list of structs where each struct contains an address and a payload sent by a CAN device 



Normally we will work with OBDII messages but we want to have a generic function in order to be compatible with any High Layer Protocol over CAN.

These function(s) should be developed in the file **southbound_ec.c**

The main three functions of this development should be called from **main.c** with different options in order to demonstrate its behaviour and ease the testing.  

# Ressources

* HW-FW document available in the Docs folder
* http://www.oneminuteinfo.com/2014/02/can-bus-obd-ii-explained-for-dummies.html
* https://en.wikipedia.org/wiki/OBD-II_PIDs
