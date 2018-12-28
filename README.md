High performance remote for ESKATE

Feature list:

- 0.96 inch OLED screen with 60ms update time for smooth and accurate inromations
- Trigger for DEADMAN or CRUISE CONTROL
- Vibration alarm analog to announcments (like low battery)
- Different main pages for individual informations
- headlight control from remote
  - FeatherReceiver Advanced -> headlight connectors up to 24W
- adaptive breaklight control
  - FeatherReceiver Advanced -> breaklight fully controlled via throttle position including force breaklight
- one remote for up to 200 different boards with FeatherReceiver
- ESTOP feature -> on lost connections or invalid received data, the receiver force the board to slow down carefully
  - ESTOP will armed after 2 seconds of validated stable tranmissions
  - soft mode -> after board stopped completely, receiver will recover automaticly from eStop
  - hard mode -> board has to switch off and on to recover from eStop
- priority transmission handling! Deactivating display refresh and UART pull if no valid transmission is received for a certain time
- 3 different riding modes for beginner, intermidiate, pro
  - throttle will be attached to 50% | 75% | 100% of PWM output to ESC
- police mode -> slow... super slow... even not running slow...
  - startup activation -> remote start in police mode if not pressed a button combination on startup
  - manual activation -> remote starts normal, police mode can be activated by button combination (and safe the actual state after restart!)
- throttle center death zone adjustable from settings menu
- overcharge alert -> if voltage is 5 seconds over 42.0 volts announcemnt with vibration is set
- discharge alert -> if board/remote voltage drops under 20/15% announcment with vibration is set

Upcoming features:

...
