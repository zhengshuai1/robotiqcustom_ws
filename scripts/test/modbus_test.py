from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import logging
import time

# FORMAT = ('%(asctime)-15s %(threadName)-15s '
#           '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
# logging.basicConfig(format=FORMAT)
# log = logging.getLogger()
# log.setLevel(logging.DEBUG)

UNIT = 0x1
def run_sync_client():
    device_name = "/dev/ttyUSB0"
    client = ModbusClient(method='rtu', port=device_name, timeout=1,
                          baudrate=9600)
    client.connect()

    log.debug("Write to a Coil and read back")
    rq = client.write_coil(3, True, unit=UNIT)
    rr = client.read_coils(3, 1, unit=UNIT)
    assert(not rq.isError())     # test that we are not an error
    assert(not rr.isError())     # test that we are not an error
    assert(rr.bits[0] == True)          # test the expected value

    time.sleep(2)

    log.debug("Write to a Coil and read back")
    rq = client.write_coil(3, False, unit=UNIT)
    rr = client.read_coils(3, 1, unit=UNIT)
    assert(not rq.isError())     # test that we are not an error
    assert(not rr.isError())     # test that we are not an error
    assert(rr.bits[0] == False)          # test the expected value

    # ----------------------------------------------------------------------- #
    # close the client
    # ----------------------------------------------------------------------- #
    client.close()

class Grippercontroller(object):
    UNIT = 0x1

    def __init__(self, device_name):
        super(Grippercontroller, self).__init__()
        self.client = ModbusClient(method='rtu', port=device_name, timeout=1, baudrate=9600)
        self.client.connect()
        print('init modbus connect success')

    def activate_gripper(self):
        rq = self.client.write_coil(0, True, unit=UNIT)
        assert (not rq.isError()), print(rq)  # test that we are not an error


    def release_gripper(self):
        rq = self.client.write_coil(0, False, unit=UNIT)
        assert (not rq.isError()), print(rq)  # test that we are not an error

    def disconnect(self):
        self.client.close()

if __name__ == "__main__":
    device_name = "/dev/ttyUSB0"
    gripper_control = Grippercontroller(device_name="/dev/ttyUSB0")
    i=0
    while i < 2:
        gripper_control.activate_gripper()
        time.sleep(2)
        gripper_control.release_gripper()
        i +=1
    gripper_control.disconnect()
