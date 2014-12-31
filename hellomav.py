# hellomav.py
#
# Mark Jacobsen
# mark@syriaairlift.org
#
# This is a simple demonstration of how to begin building a ground control station (GCS)
# in Python using a GUI. It allows a user to select a serial port and baud rate, connect
# to a MAVLINK device, then send a command to arm the motor. Note that this does not
# include any exception handling.
#
# The GUI is built using wxPython. MAVLINK communication is done through the pymavlink
# library.

# Acknowledgements:
# Thank you to Andrew Tridgell, the mastermind behind pymavlink and MAVProxy
# Thread code from http://stackoverflow.com/questions/730645/python-wxpython-doing-work-continuously-in-the-background
# Serial port code taken from http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python


from pymavlink import mavutil
import wx
import sys, serial, glob, threading

BUTTON_CONNECT = 10
BUTTON_ARM = 20

#---------------------------------------------------------------------------

# Create a wx.Frame object for the interface
class HelloMAVFrame(wx.Frame):

    def __init__(self, parent, id=-1, title='Hello MAV!',
                 pos=wx.DefaultPosition, size=(500, 500),
                 style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        self.InitUI()



        self.Show()

    def InitUI(self):
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Port combo boxes
        box_ports = wx.BoxSizer(wx.HORIZONTAL)
        baudList = ['57600', '115200']
        label_port = wx.StaticText(self, -1, "Port",size=(80,-1))
        self.cb_port = wx.ComboBox(self, 500, "default value", (90, 50),
                         (160, -1), serial_ports(),
                         wx.CB_DROPDOWN
                         )
        label_baud = wx.StaticText(self, -1, "Baud",size=(50,-1))
        self.cb_baud = wx.ComboBox(self, 500, "115200", (90, 50),
                         (160, -1), baudList,
                         wx.CB_DROPDOWN
                         )

        box_ports.Add(label_port, 0, wx.TOP|wx.RIGHT, 5)
        box_ports.Add(self.cb_port, 1, wx.EXPAND|wx.ALL)
        box_ports.Add(label_baud, 0, wx.TOP|wx.RIGHT, 5)
        box_ports.Add(self.cb_baud, 1, wx.EXPAND|wx.ALL)
        sizer.Add(box_ports, 0, wx.EXPAND|wx.ALL)

        btn_connect = wx.Button(self, BUTTON_CONNECT, "Connect")
        sizer.Add(btn_connect, 0, wx.EXPAND|wx.ALL)
        btn_arm = wx.Button(self, BUTTON_ARM, "Arm")
        sizer.Add(btn_arm, 0, wx.EXPAND|wx.ALL)

        # Divider line
        line = wx.StaticLine(self, -1, size=(20,-1), style=wx.LI_HORIZONTAL)
        sizer.Add(line, 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.RIGHT|wx.TOP, 5)

        # All traffic text box
        self.textOutput = wx.TextCtrl(self, -1, "", size=(125, 300))
        sizer.Add(self.textOutput, 1, wx.EXPAND|wx.ALL)

        # Wire up buttons
        self.Bind(wx.EVT_BUTTON, self.on_click_connect, btn_connect)
        self.Bind(wx.EVT_BUTTON, self.on_click_arm, btn_arm)

        self.SetSizer(sizer)
        sizer.Fit(self)
        return

    def on_click_connect(self,e):
        """
        Process a click on the CONNECT button

        Attempt to connect to the MAV using the specified port and baud rate,
        then subscribe a function called check_heartbeat that will listen for
        a heartbeat message, as well as a function that will print all incoming
        MAVLink messages to the console.
        """

        port = self.cb_port.GetValue()
        baud = int(self.cb_baud.GetValue())
        self.textOutput.AppendText("Connecting to " + port + " at " + str(baud) + " baud\n")

        self.master = mavutil.mavlink_connection(port, baud=baud)
        self.thread = threading.Thread(target=self.process_messages)
        self.thread.setDaemon(True)
        self.thread.start()

        self.master.message_hooks.append(self.check_heartbeat)
        self.master.message_hooks.append(self.log_message)


        print("Connecting to " + port + " at " + str(baud) + "baud")
        self.textOutput.AppendText("Waiting for APM heartbeat\n")
        return

    def on_click_arm(self,e):
        """
        Process a click on the ARM button

        Send an arm message to the MAV, then subscribe a function called
        check_arm_ack that will listen for a positive confirmation of arming.
        """
        self.textOutput.AppendText("Arming motor\n")
        print("******arming motor*********")
        self.master.arducopter_arm()

        self.master.message_hooks.append(self.check_arm_ack)

    def log_message(self,caller,msg):
        if msg.get_type() != 'BAD_DATA':
            print(str(msg))
        return

    def process_messages(self):
        """
        This runs continuously. The mavutil.recv_match() function will call mavutil.post_message()
        any time a new message is received, and will notify all functions in the master.message_hooks list.
        """
        while True:
            msg = self.master.recv_match(blocking=True)
            if not msg:
                return
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()

    def check_heartbeat(self,caller,msg):
        """
        Listens for a heartbeat message

        Once this function is subscribed to the dispatcher, it listens to every
        incoming MAVLINK message and watches for a 'HEARTBEAT' message. Once
        that message is received, the function updates the GUI and then
        unsubscribes itself.
        """

        if msg.get_type() ==  'HEARTBEAT':
            self.textOutput.AppendText("Heartbeat received from APM (system %u component %u)\n" % (self.master.target_system, self.master.target_system))
            self.master.message_hooks.remove(self.check_heartbeat)

    def check_arm_ack(self, caller, msg):
        """
        Listens for confirmation of motor arming

        Once this function is subscribed to the dispatcher, it listens to every
        incomign MAVLINK message and watches for the "Motor armed!" confirmation.
        Once the message is received, teh function updates the GUI and then
        unsubscribes itself.
        """

        if msg.get_type() == 'STATUSTEXT':
            if "Throttle armed" in msg.text:
                self.textOutput.AppendText("Motor armed!")
                self.master.message_hooks.remove(self.check_arm_ack)

    def OnClose(self, e):
        self._mgr.UnInit()
        self.Close()

def serial_ports():
    """Lists all available serial ports

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """

    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


# Create our wxPython application and show our frame
app = wx.App()
frame = HelloMAVFrame(None)
frame.Show()
app.MainLoop()
