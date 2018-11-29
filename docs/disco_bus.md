# Disco Bus Protocol
This boot protocol is meant to enumerate a series of boards with ID numbers instead of requiring each board to have a unique identifier.

The protocol is as follows:
1) While in bootloader, the board listens on the broadcast channel (0) for a board ID
2) When a ID is sent, the board checks if its DISCO_BUS_IN is high; if so, this is now the board's ID
3) If this succeeds, the board will reply with its new board ID; if no response, try again (or check disco connection)
4) The board that was just identified will now set their DISCO_BUS_OUT high, queueing the next board in series

For the first board to work, the following is implemented:
* Input pins use pull-up resistors built in the STM32 to default active
* All outputs defaul drive low (will overpower the pull-ups ~30k [[table 48 in section 5.3.16](https://www.st.com/resource/en/datasheet/dm00037051.pdf)])
