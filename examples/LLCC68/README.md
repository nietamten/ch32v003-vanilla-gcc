Tested with two Ra-01SCH modules.

First device has 
uint8_t receiver = 0;
Second device has 
uint8_t receiver = 1;

Devices output messages to UART (with some bugs...).
Receiver messages are different when Sender is running.


Ra-01SCH	CH32V003
GND			GND
3V3			VDD (must be 3.3V)
NSS			PC1
MOSI		PC6
MISO		PC7
SCK			PC5
BUSY		PD0
DIO1		PD3
RST			PC3
ANT			...
