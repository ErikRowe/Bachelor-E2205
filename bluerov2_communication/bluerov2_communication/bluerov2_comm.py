from pymavlink import mavutil
from pymavlink import quaternion


class BlueROV2Comm:

    def __init__(self):
        # Disable "Bare exception" warning
        # pylint: disable=W0702

        # Create the connection
        #  If using a companion computer
        #  the default connection is available
        #  at ip 192.168.2.1 and the port 14550
        # Note: The connection is done with 'udpin' and not 'udpout'.
        #  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
        #  uses a 'udpbcast' (client) and not 'udpin' (server).
        #  If you want to use QGroundControl in parallel with your python script,
        #  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
        #  E.g: --out udpbcast:192.168.2.255:yourport
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')

        # Make sure the connection is valid
        self.master.wait_heartbeat()

        self.init_stored_vars()

    def init_stored_vars(self):
        self.attitude = quaternion.QuaternionBase()
        #self.position = [0]*3
        self.velocity = [0]*6
        self.previous_time = None


    def read_data(self):
        msg = self.master.recv_match()
        if not msg:
            return
        if msg.get_type() == 'ATTITUDE':
            msg_dict = msg.to_dict()
            self.velocity[3:] = [msg_dict['rollspeed'], msg_dict['pitchspeed'], msg_dict['yawspeed']]
            self.attitude = quaternion.QuaternionBase([msg_dict['roll'], msg_dict['pitch'], msg_dict['yaw']])
        # if msg.get_type() == 'GLOBAL_POSITION_INT':
        #     msg_dict = msg.to_dict()
        #     self.velocity[:3] = [msg_dict['vx'], msg_dict['vy'], msg_dict['vz']]
        #     self.estimate_position(msg_dict['time_boot_ms'])

    def estimate_position(self, current_time):
        if self.previous_time == None:
            self.previous_time = current_time
        dt = (current_time - self.previous_time) / 10e3
        self.position = [round(self.position[i] + self.velocity[i]*dt, 2) for i in range(3)]
        self.previous_time = current_time


    def set_rc_channel_pwm(self, pwm_list=[1500]*8):
        rc_channel_values = [65535 for _ in range(18)]
        for channel_id, pwm in enumerate(pwm_list):
            rc_channel_values[channel_id - 1] = pwm
            
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            *rc_channel_values)                       # RC channel list, in microseconds.
