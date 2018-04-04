# M2M-CAN-Development
CAN development in M2M commboard

# TASKs TO-DO

Southbound functions to be developed

## I2C configuration

`result config_i2c(baudRate, listen, address_list)`

*Inputs*

* baudRate (int): Speed to be used in the I2C interface
* listen (bool): Set if the device will work read and write the I2C bus or only the write it
* address_list (List): List of the addresses that will listen the device. This list will contain N I2C addresses if the _listen_ is true. It can contains the broadcast address that indicates the device listen everything. Addresses codfied in HEX

*Output*

* result: Number code indicating success (0) or error (number indicating the error code)

## Write I2C message

`result write_can_message(i2c_address, payload)`

*Inputs*

* i2c_address: Address that should contain the I2C message, could be a broadcast address. Address codified in HEX
* payload: Payload to be sent codified in HEX. If bigger than 8 bytes, several I2C messages should be sent


*Output*

* result: Number code indicating success (0) or error (number indicating the error code)


## Read I2C Bus

`i2c_messages_list read_i2c_bus()`

*Inputs*

No inputs for this function. Read / Listen the info of the devices with addresses set in _config_i2c_

*Output*

* i2c_messages_list (List of struct): Return a list of structs where each struct contains an address and a payload sent by a i2c device 


# Additional Information

We want to have a generic function in order to be compatible with any High Layer Protocol over I2C.

These function(s) should be developed in the file **southbound_ec.c**

The main three functions of this development should be called from **main.c** with different options in order to demonstrate its behaviour and ease the testing.  

# Ressources

* HW-FW document available in the Docs folder
* http://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf
* https://en.wikipedia.org/wiki/I%C2%B2C
