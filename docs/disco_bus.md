# Disco Bus Protocol
This boot protocol is meant to enumerate a series of boards with ID numbers instead of requiring each board to have a unique identifier.

The protocol is as follows:
1) While in bootloader, the board listens on the broadcast channel (0) for any packet with an ID that's not the broadcast ID with the enumerate function code
2) When one such packet is found, the board checks if its DISCO_BUS_IN is low; if so, this is now the board's ID
3) If this succeeds, the board will reply with its new board ID; if no response, try again (or check disco connection)
4) When successful communication is established, lock ID into the current board
5) The board that was just locked will now set their DISCO_BUS_OUT low, queueing the next board in series and unlocking the rest of the bootloader

For the first board to work, the following is implemented:
* Input pins are pulled low by external resistors
* The hardware resistors decouple boards from one another (so their different grounds won't EMP hardware)
* All outputs defaul drive low (will overpower the pull-ups ~30k [[table 48 in section 5.3.16](https://www.st.com/resource/en/datasheet/dm00037051.pdf)])
