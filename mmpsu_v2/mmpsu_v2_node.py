
import rclpy
from rclpy.node import Node
from mmpsu_v2.mmpsu_v2 import mmpsu_v2
from threading import Lock
from std_srvs.srv import SetBool

from mmpsu_v2.srv import SetFloat32

from mmpsu_v2.msg import mmpsu_status

class MmpsuV2Node(Node):
    """MMPSU v2 ROS 2 node"""

    PHASE_MAPPING = {
        0: 'A',
        1: 'B',
        2: 'C',
        3: 'D',
        4: 'E',
        5: 'F'
    }

    STATUS_FIELDS = [
        'OUTPUT_ENABLED',
        'VOUT_MEASURED',
        'VOUT_SETPOINT',
        'PHASES_PRESENT',
        'PHASES_ENABLED',
        'PHASE_A_DUTY_CYCLE',
        'PHASE_A_CURRENT',
        'PHASE_A_CURRENT_LIMIT',
        'PHASE_A_TEMP',
        'PHASE_B_DUTY_CYCLE',
        'PHASE_B_CURRENT',
        'PHASE_B_CURRENT_LIMIT',
        'PHASE_B_TEMP',
        'PHASE_C_DUTY_CYCLE',
        'PHASE_C_CURRENT',
        'PHASE_C_CURRENT_LIMIT',
        'PHASE_C_TEMP',
        'PHASE_D_DUTY_CYCLE',
        'PHASE_D_CURRENT',
        'PHASE_D_CURRENT_LIMIT',
        'PHASE_D_TEMP',
        'PHASE_E_DUTY_CYCLE',
        'PHASE_E_CURRENT',
        'PHASE_E_CURRENT_LIMIT',
        'PHASE_E_TEMP',
        'PHASE_F_DUTY_CYCLE',
        'PHASE_F_CURRENT',
        'PHASE_F_CURRENT_LIMIT',
        'PHASE_F_TEMP',
        'PHASES_IN_OVERTEMP',
        'COMMS_ERROR_COUNT',
        'V_IN',
        'I_IN',
        'V_12P0',
        'I_12P0',
        'V_5P0',
        'I_5P0'
    ]

    def __init__(self):
        super().__init__('mmpsu_v2')
        # publisher
        self.status_pub = self.create_publisher(mmpsu_status, 'mmpsu_status', 10)

        # services
        self.output_enable = self.create_service(SetBool, 'mmpsu/set_output_enabled', self.set_output_enabled_callback)
        self.vout_set_srv = self.create_service(SetFloat32, 'mmpsu/set_vout', self.set_output_voltage_callback)
        self.iout_limit_srv = self.create_service(SetFloat32, 'mmpsu/set_iout_limit')

        self.status_period = self.get_parameter('mmpsu/status_period').get_parameter_value().double_value
        self.uart_path = self.get_parameter('mmpsu/uart_path').get_parameter_value().string_value

        self._mmpsu = mmpsu_v2.MmpsuV2Uart(self.uart_path, self.get_logger())
        self._mmpsu_lock = Lock()

        # status timer
        self.timer = self.create_timer(self.status_period, self.status_timer_callback)


    def status_timer_callback(self):
        msg = mmpsu_status()

        self._mmpsu_lock.acquire()
        fields = self._mmpsu.read_fields(self.STATUS_FIELDS)
        self._mmpsu_lock.release()

        msg.output_enabled = fields['OUTPUT_ENABLED']
        msg.vout_measured = float(fields['VOUT_MEASURED'])/1000.0
        msg.vout_setpoint = float(fields['VOUT_SETPOINT'])/1000.0

        phases_present = fields['PHASES_PRESENT']
        phases_enabled = fields['PHASES_ENABLED']
        phases_overtemped = fields['PHASES_IN_OVERTEMP']
        for phase_ind in range(0, 6):
            present = bool((phases_present >> phase_ind) & 1)
            msg.phase_present[phase_ind] = present
            if present:
                msg.phase_enabled[phase_ind] = bool((phases_enabled >> phase_ind) & 1)
                msg.phase_overtemp[phase_ind] = bool((phases_overtemped >> phase_ind) & 1)
                letter = self.PHASE_MAPPING[phase_ind]
                msg.phase_duty_cycle[phase_ind] = float(fields[f'PHASE_{letter}_DUTY_CYCLE'])/5440.0
                msg.phase_current[phase_ind] = float(fields[f'PHASE_{letter}_CURRENT'])/1000.0
                msg.phase_current_limit[phase_ind] = float(fields[f'PHASE_{letter}_CURRENT_LIMIT'])/1000.0
                msg.phase_temp[phase_ind] = fields[f'PHASE_{letter}_TEMP']
        
        msg.comms_err_count = fields['COMMS_ERROR_COUNT']
        msg.vin_voltage = fields['V_IN']
        msg.vin_current = fields['I_IN']
        msg.v5p0_voltage = fields['V_5P0']
        msg.v5p0_current = fields['I_5P0']
        msg.v12p0_voltage = fields['V_12P0']
        msg.v12p0_current = fields['I_12P0']

        self.status_pub.publish(msg)

    def set_output_enabled_callback(self, request, response):
        self._mmpsu_lock.acquire()
        self._mmpsu.write_field('OUTPUT_ENABLED', request.data)
        self._mmpsu_lock.release()
        self.response.success = True
        return response

    def set_output_voltage_callback(self, request, response):
        setpt = self._clamp(request.setpoint, self._mmpsu.MIN_VOUT, self._mmpsu.MAX_VOUT)
        self._mmpsu_lock.acquire()
        self._mmpsu.write_field('VOUT_SETPOINT', int(setpt*1000.0))
        self._mmpsu_lock.release()
        self.response.success = True
        return response

    def _clamp(self, value, min_val, max_val):
        if value >= min_val:
            if value <= max_val:
                return value
            else:
                return max_val
        else:
            return min_val

def main(args=None):
    rclpy.init(args=args)

    mmpsu_node = MmpsuV2Node()

    rclpy.spin(mmpsu_node)

    mmpsu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()