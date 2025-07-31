from comet_rpc import (
    exec_kcl,
    IoType,
    iogetpn,
    iovalrd,
    iovalset,
    iodefpn,
    vmip_writeva,
)

# IP address or hostname of the R-30iB(+) controller
server = "192.168.2.151"

exec_kcl(server, "reset")
vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)
# dout1_val = iovalrd(server, IoType.DigitalOut, index=20).value
# iodefpn(server, IoType.DigitalOut, index=20, comment='ArcTool Weld Start')
# dout1_cmt = iogetpn(server, IoType.DigitalOut, index=20).value
# print(dout1_cmt)

# iodefpn(server, IoType.DigitalOut, index=21, comment='ArcTool Weld End')
# dout1_cmt = iogetpn(server, IoType.DigitalOut, index=21).value
# print(dout1_cmt)

dwrite1 = iovalset(server, IoType.DigitalOut,index=21,value=1)
# dout1_val = iovalrd(server, IoType.DigitalOut, index=1).value
# print(dout1_val)