#!/usr/bin/python
import os
from subprocess import Popen, PIPE
import dbus

class USBDevice:
    def __init__(self, device, label, filesys, mount_point):
        self.device = device
        self.label = label
        self.filesys = filesys
        self.mount_point = mount_point

    def mount(self):
        if not os.path.ismount(self.mount_point):
            process = Popen(['udisksctl','mount','-b', self.device], stdout=PIPE,stderr=PIPE)
            stdout, stderr = process.communicate()
            if len(stdout) == 0:
                self.mount_point = 'null'
            else:
                self.mount_point = stdout.split()[3][:-1]
        else:
            print("%s already mounted at %s"%(self.label,self.mount_point))
    
    def unmount(self):
        if os.path.ismount(self.mount_point.strip()):
            process = Popen(['udisksctl','unmount','-b', self.device])
            process.wait()
        else:
            print("%s already unmounted "%(self.label))

    def print_status(self):
        print("device       %s"%self.device)
        print("Label        %s"%self.label)
        print("File system  %s"%self.filesys)
        print("mount_point  %s"%self.mount_point)

    def size_status(self):
        space_st = os.statvfs(self.mount_point)
        # f_bavail: without blocks reserved for super users
        # f_bfree:  with    blocks reserved for super users
        avail = space_st.f_frsize * space_st.f_bavail
        capa = space_st.f_frsize * space_st.f_blocks
        used = capa - avail
        return {
            'total': capa,
            'used': used,
            'available': avail,
            'percent': float(used) / capa,
        }

    def __getitem__(self, item):
        return getattr(self, item)

class USBDevices:
    def __init__(self):
        self.devices = []
        bus = dbus.SystemBus()
        ud_manager_obj = bus.get_object('org.freedesktop.UDisks2', '/org/freedesktop/UDisks2')
        om = dbus.Interface(ud_manager_obj, 'org.freedesktop.DBus.ObjectManager')
        try:
            for k,v in om.GetManagedObjects().items():
                k
                drive_info = v.get('org.freedesktop.UDisks2.Block', {})
                if drive_info.get('IdUsage') == "filesystem" and not drive_info.get('HintSystem') and not drive_info.get('ReadOnly'):
                    device = drive_info.get('Device')
                    device = bytearray(device).replace(b'\x00', b'').decode('utf-8')
                    # print "printing             " + device

                    bd = bus.get_object('org.freedesktop.UDisks2', '/org/freedesktop/UDisks2/block_devices%s'%device[4:])

                    label = bd.Get('org.freedesktop.UDisks2.Block', 'IdLabel', dbus_interface='org.freedesktop.DBus.Properties')
                    # print 'Name od partition is %s'%label

                    file_system =  bd.Get('org.freedesktop.UDisks2.Block', 'IdType', dbus_interface='org.freedesktop.DBus.Properties')
                    # print 'Filesystem is        %s'%file_system

                    mount_byte = bd.Get('org.freedesktop.UDisks2.Filesystem', 'MountPoints', dbus_interface='org.freedesktop.DBus.Properties')
                    mount_point = ''
                    if len(mount_byte) != 0:
                        mount_point = mount_point.join([str(v) for v in mount_byte[0]])
                        mount_point = mount_point[:-1]
                        # print "mount_point         %s"%repr(mount_point)
                    
                    if 'mmcblk' not in device:
                        self.devices.append(USBDevice(device, label, file_system, mount_point))
                    # print
        except:
            print("No device found...")

if __name__ == "__main__":
    USB_devices = USBDevices()
    for device in USB_devices.devices:
        device.print_status()
