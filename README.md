Features:

- 0.96inch OLED with 128x64 with high frequently display update rate to have smooth and accurate inforamtions (every 60ms)
- vibration feature (Alarm on low batttery from Board, Remote battery...)
- extra button to switch between 3 different main Screens 
  - Speed, Disance, Voltage
  - Voltage, Motor amps, Battery amps
  - DEBUG RSSI, cycle time, failed transmissions
- headlight integration by pushing the extra button for 2 seconds
- over 16 updates per seconds from throttle position to PWM output (every 60ms)

Safety Features

- ESTOP automatic slow down until stop - more than 300ms no new values from remote will force the receiver
		into ESTOPMODE and slow down the trigger value every 50ms by 5 until it raches 25% break trigger to stop the board safe
- priority transmission handling! Deactivating display refresh and UART pull if no valid transmission is received until
  - ESTOPMODE is triggered, then display is refreshing again for anaysing error
- breaklight integration with adaptive flash warning
  - if trigger is under centerHallValue the breaklight will be bright 
  - if trigger is close to minHallValue the breaklight will flash
