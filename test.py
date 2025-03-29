from obd.obd import OBD
odb = OBD("/dev/rfcomm0")
from obd.commands import commands

cmd_coolant_temp = commands['COOLANT_TEMP']
odb.supports(cmd_coolant_temp)
ret = odb.query(cmd_coolant_temp)
print(ret.value)

cmd_throttle = commands['THROTTLE_POS']
odb.supports(cmd_throttle)
ret = odb.query(cmd_throttle)
print(ret.value)

hook = 0
