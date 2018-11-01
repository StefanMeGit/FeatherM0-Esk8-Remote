Features:

- 0.96inch OLED with 128x64
- high frequently display update rate to have smooth and accurate inforamtions (every 60ms)
- vibration feature (Alarm on low batttery from Board, Remote)
- extra button to switch between 3 different main Screens 
  - Speed, Disance, Voltage
  - Voltage, Motor amps, Battery amps
  - DEBUG RSSI, cycle time, failed transmissions
- headlight integration
- over 16 updates per seconds from throttle position to PWM output (every 60ms)

Safety Features

- ESTOP automatic slow down until stop - more than 300ms no new values from remote will force the receiver
	into ESTOP and slow down the trigger value every 100ms by 50 until it raches -50 trigger to stop the board safe
- priority transmission handling! First successfully transmissions -> then scree refresh ect.
- breaklight integration with adaptive flash warning
