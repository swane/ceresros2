2CeresROS2 reads CSV data from Arduino at 57600 baud as the following:
odo, steer, spd, batt, drv_current, current, error,drv_pwm
Where: 
odo=odometry pulses
steer=steer angle
spds=speed m/s
batt=battery voltage
drv_current = motor drive current in A
current=totl current in A
errors=error byte
drv_pwm=PWM value as byte

Publishes the following topics:

/CeresActSpd
/CeresActStr
/CeresBatt
/CeresCurrent
/CeresDrvCrt
/CeresDrvPWM
/CeresError
/CeresOdo
