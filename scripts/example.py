'''
Created on Jun 10, 2014

@author: thomasesch
'''
import threespace_api_2 as ts_api

## If the COM port is already known and the device type is known for the 3-Space
## Sensor device, we can just create the appropriate instance without doing a
## search.
com_port = "/dev/ttyACM0"
try:
    device = ts_api.TSUSBSensor(com_port=com_port)
except:
    print("No device on {0}".format(com_port))
## If a connection to the COM port fails, None is returned.
else:
    print(device)

    ## Now close the port.
    if device is not None:
        device.close()
        
device_list = [ts_api.ComInfo(com_port='/dev/ttyACM0', friendly_name='PrioVR Base Station (ttyACM0)', dev_type='BS')]
print device_list

com_port, name, dev_type = device_list[0]

pvr_base = ts_api.PVRSystem(com_port)

print(pvr_base)
print(pvr_base.present_nodes)
print(pvr_base.active_nodes)
#for node in pvr_base.active_nodes:
        #print(pvr_base.getUntaredOrientationAsQuaternion(node))

print(pvr_base.getUntaredOrientationAsQuaternion(0)[0])



