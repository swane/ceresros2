import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from std_msgs.msg import Float64,Int16,Int8,Bool
import time
from math import tan
port = '/dev/ttyACM0'
serialport=""
#odos, steers, spds, batts, drv_current, current, errors,drv_pwm
L=0.69  #vehicle length in m for kinematics Twist message

class CeresPublisher(Node):

    def __init__(self):
        super().__init__('Ceres_publisher')

        self.pubAspd = self.create_publisher(Float64, 'CeresActSpd', 10)
        self.pubAstr = self.create_publisher(Float64, 'CeresActStr', 10)
        self.pubBatt = self.create_publisher(Float64, 'CeresBatt', 10)
        self.pubDrvCrt = self.create_publisher(Float64, 'CeresDrvCrt', 10)
        self.pubCurrent = self.create_publisher(Float64, 'CeresCurrent', 10)
        self.pubDrvPWM = self.create_publisher(Int16, 'CeresDrvPWM', 10)
        self.pubError = self.create_publisher(Int16, 'CeresError', 10)
        self.pubOdo= self.create_publisher(Int16, 'CeresOdo', 10)

        # Now set a timer
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        print("Bang!")
        msg = Float64()
        msg.data = 36.2
        self.pubAspd.publish(msg)


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        global serialport
        serialport.close()

def read_robot(datastring,p):  # $odom,steerangle,act velocity,lat,lonbearing
    global serialport
    global x
    global y
    global odo
    global vth
    global prevod
    global fwdx
    global L

    splitstring = datastring.split(b',')

    if len(splitstring) < 5:
        print (f"Error invalid input: {datastring}")
        serialport.flushInput()
        serialport.flushOutput()
        serialport.flush()
    else:
        odos, steers, spds, batts, drv_currents, currents,errors,drv_pwms  = splitstring[0:8]
#odos, steers, spds, batts, drv_current, current, errors,drv_pwm
        drv_pwms = drv_pwms[:-2] #strips off \n\r

        try:
            odo = int(odos)
            msg = Int16()
            msg.data = odo
            p.pubOdo.publish(msg)
        except:
            odo = 0
        try:

            actspd = float(spds)
            msg = Float64()
            msg.data = actspd
            #ceres_publisher.pubAspd.publish(actspd)
            p.pubAspd.publish(msg)

            speeds = [0]  # can be for each wheel, so is  list
            speeds[0] = int(actspd * 1000)
                #fixposition_speed.publish(speeds)
        except:
            actspd = 0
        try:
            actstr = float(steers)

            msg = Float64()
            msg.data = actstr
            p.pubAstr.publish(msg)

        except:
            actstr = 0
        try:
            batt = float(batts)

            msg = Float64()
            msg.data = batt
            p.pubBatt.publish(msg)
        except:
            batt = 0
        try:
            error = int(errors)
            msg = Int16()
            msg.data = error
            p.pubError.publish(msg)
        except:
            errors = 0
        try:
            drv_current = float(drv_currents)
            msg = Float64()
            msg.data = drv_current
            p.pubDrvCrt.publish(msg)
        except:
            drv_current = 0
        try:
            current = float(currents)
            msg = Float64()
            msg.data = current
            p.pubCurrent.publish(msg)
        except:
            current = 0
        try:
            drv_pwm = int(drv_pwms)
            msg = Int16()
            msg.data = drv_pwm
            p.pubDrvPWM.publish(msg)
        except:
            drv_pwm = 0


            # calc thetadot from actspd and actstr (degrees, positive to the right)
            #vth = (actspd / L) * tan(-actstr / 57.296)


def wait_USB():
    #ports listed: sudo udevadm monitor -u
    global port
    global serialport
    portOK=False
    print(f"OPENING ON PORT:{port}")
    while not portOK:
        try:
            serialport = serial.Serial(port, 57600, timeout=1, writeTimeout=0)
            portOK = True
            serialport.close()
            serialport.open()
            serialport.flush()
            print("Connected.")
        except:
            print("Port not connected, connect USB Arduino")
            if serialport!="":
                serialport.close()
            portOK = False
            time.sleep(1)


def main(args=None):
    
    rclpy.init(args=args)

    # port=arduino_ports[0]

    ceres_publisher = CeresPublisher()
    portOK = False
    wait_USB()
    print(serialport)

    while True:
        c = ""
        try:
            c = serialport.read(1)
        except serial.SerialException as e:
            portOK = False
            wait_USB()

        if c==b'$':
            readOK=True
            try:
                st = serialport.readline()
                print(st)
            # print st
            except serial.SerialException as e:
                portOK = False
                readOK = False
                wait_USB()
            if readOK:
                #print("Read")
                read_robot(st,ceres_publisher)

            # return none

                #ceres_publisher.pubAspd.publish(msg)
        #rclpy.spin_once(ceres_publisher, timeout_sec=0.1)
    ceres_publisher.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
