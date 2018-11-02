High perfomance remote for ESKATE

Features:

- 0.96inch OLED with 128x64 with high frequently display update rate to have smooth and accurate inforamtions (every 60ms)
- join settings menu with 5 seconds press on extra button or hold trigger on start up
- trigger can be used as DEADMAN-Switch or CUISE-Control
- vibration feature (Alarm on low batttery from Board, Remote battery...)
- extra button to switch between 3 different main Screens 
  - Speed, Disance, Voltage
  - Voltage, Motor amps, Battery amps
  - DEBUG RSSI, cycle time, failed transmissions
- headlight integration by pushing the extra button for 2 seconds
- over 16 updates per seconds from throttle position to PWM output (every 60ms)
- pair up to 200 boards with unique ID
  - one remote will handle all boards by selection the board ID from settings menu

Safety Features:

- automatic generated 16 bytes encryption key (15 bytes for unique key and 16th byte for board selection)
  - after first tun on from remote, it will generate a random 15 byte key
  - every new reciever can be paired from settings menu with the new key and board ID
- ESTOP automatic slow down until stop - more than 300ms no new values from remote will force the receiver into ESTOPMODE and slow down the trigger value every 50ms by 5 until it raches 25% break trigger to stop the board safe
  - ESTOP will armed automaticly after first valid transmission between remote and receiver
- priority transmission handling! Deactivating display refresh and UART pull if no valid transmission is received until
  - ESTOPMODE is triggered, then display is refreshing again for anaysing error
- breaklight integration with adaptive flash warning
  - if trigger is under middle position the breaklight will be bright 
  - if trigger is close to full break position the breaklight will flash

Upcomming features:

- different riding modes for begginers| intermidiate | pros
  - trigger value will be attached to 50% | 75% | 100%
- police mode
  - Trigger value will be cut of at 25%, can be unlocked with button press combination
...
