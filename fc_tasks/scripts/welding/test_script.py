from Welding import Welder
import comet_rpc as rpc
import time

def gas_start():
    if rpc.iovalrd('192.168.2.151', rpc.IoType.DigitalIn, index=27) == 1:
        print("Gas Fault Detected")
        return
    else:
        rpc.iovalset('192.168.2.151', rpc.IoType.DigitalOut, index=26, value=1) # GAS START
        time.sleep(0.5)
        print("gas on")

if __name__ == '__main__':
    
    server = '192.168.2.151'
    welder = Welder(server=server)

    rpc.vmip_writeva(server, "*SYSTEM*", "$MCR.$GENOVERRIDE", value=100)

    # Laser Power Watt Setting 
    # This is Register 102. Needs to be done on TP. Can add a check if needed 
    # GOUT 2 seems to be labelled the same. 

    # Enable External Control 
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=47, value=1)
    # Enable Aiming Laser 
    rpc.iovalset(server, rpc.IoType.DigitalOut, index=43, value=0)
    time_s = time.time()
    duration = 20
    # CALL LASER_READY_ARM
    welder.laser_ready_arm()
    time.sleep(2)
    welder.laser_start_emit()
    time.sleep(2.5)

    # while True:
    #     val = rpc.iovalrd(server, rpc.IoType.DigitalOut, 41)
    #     print(val)

    #     if val.value == 0:
    #         rpc.iovalset(server, rpc.IoType.DigitalOut, index=41, value=1) # GATE IN/EMIT

    #     if time.time() - time_s > duration:
    #         break
    
    welder.laser_stop_emit()
    welder.laser_disarm()

    # welder.weld_start()
    # time.sleep(0.5)
    # welder.weld_end()
