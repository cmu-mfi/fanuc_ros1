#!/usr/bin/env python3
'''
@file IOStates.py
@package fc_tasks
@author Ankit Aggarwal 
@contact ankitagg@andrew.cmu.edu
@organization Manufacturing Futures Institute, Carnegie Mellon University
@date 2025-07-28
@license 
@version 1.0
@brief ROS node for interfacing with Fanuc controller IOs using comet_rpc.

This module defines the Fanuc_IO class, which provides a ROS node for reading and writing 
Digital and Analog Inputs/Outputs on a Fanuc robot controller via a network interface. 
The module publishes IO states to ROS topics and offers ROS services to set and read IO values.

@defgroup fc_io_node Fanuc IO Node
@{
'''

import comet_rpc as rpc
import rospy
from fc_msgs.msg import IOState, IOStateArray
from fc_msgs.srv import ReadIO, SetIO, SetIOResponse, ReadIOResponse

class Fanuc_IO():   

    '''
    @class Fanuc_IO
    @brief ROS node for managing Fanuc controller IO states.

    This class initializes as a ROS node, publishing IO states to ROS topics
    and providing services to set or get individual IO values on a Fanuc controller.
    '''

    def __init__(self):

        '''
        @brief Constructor. Initializes ROS node, publishers, and connection parameters.
        '''

        self.server_ip = rospy.get_param('~server', '192.168.2.151')
        rospy.init_node('fc_io_node')
        self.publisher_dout = rospy.Publisher('/io_states_DOUT', IOStateArray, queue_size=10)
        self.publisher_din = rospy.Publisher('/io_states_DIN', IOStateArray, queue_size=10)
        self.publisher_aout = rospy.Publisher('/io_states_AOUT', IOStateArray, queue_size=10)
        self.publisher_ain = rospy.Publisher('/io_states_AIN', IOStateArray, queue_size=10)
        self.rate = rospy.Rate(50)
    
    def checkComment(self, IOType:rpc.IoType, index:int):

        '''
        @brief Check if an IO point has a comment.
        @param IOType IO type (DigitalIn, DigitalOut, AnalogIn, AnalogOut).
        @param index Index of the IO point.
        @return True if the IO has a comment, False otherwise.
        '''

        comment = rpc.iogetpn(server=self.server_ip, typ=IOType, index=index)
        return comment.value != ''
    
    def generate_iostate_array(self, IOType:rpc.IoType, max_list:int):
        
        '''
        @brief Generate an IOStateArray message for a specific IO type.
        @param IOType IO type to query.
        @param max_list Maximum number of IO points to check.
        @return IOStateArray containing active IO points with comments.
        '''

        io_array = IOStateArray()
        io_array.states = []

        for i in range(max_list):
            if self.checkComment(IOType, i):
                state = IOState()
                state.index = i
                state.value = rpc.iovalrd(server = self.server_ip, typ = IOType, index = i)
                state.comment = rpc.iogetpn(server = self.server_ip, typ = IOType, index = i)  
                io_array.states.append(state)
        
        return io_array
    
    def setIOvalue(self, IO_Type, index:int, value:int):

        '''
        @brief Set the value of a given IO point.
        @param IO_Type IO type as string ('Digital_OUT', 'Digital_IN', 'Analog_OUT', 'Analog_IN').
        @param index Index of the IO point.
        @param value Value to set.
        @return True if successful, False otherwise.
        '''

        IOType = None
        if IO_Type == 'Digital_OUT':
            IOType = rpc.IoType.DigitalOut
        elif IO_Type == 'Digital_IN':
            IOType = rpc.IoType.DigitalIn
        elif IO_Type == 'Analog_OUT':
            IOType = rpc.IoType.AnalogOut
        elif IO_Type == 'Analog_IN':
            IOType = rpc.IoType.AnalogIn

        try:
            response = rpc.iovalset(self.server_ip, typ=IOType, index=index, value=value)
            return True
        except:
            print("Cannot set IO Value")
            return False
    
    def readIOvalue(self, IO_Type, index:int, value:int):
        
        '''
        @brief Read the value of a given IO point.
        @param IO_Type IO type as string ('Digital_OUT', 'Digital_IN', 'Analog_OUT', 'Analog_IN').
        @param index Index of the IO point.
        @return IO value on success, False on failure.
        '''

        IOType = None
        if IO_Type == 'Digital_OUT':
            IOType = rpc.IoType.DigitalOut
        elif IO_Type == 'Digital_IN':
            IOType = rpc.IoType.DigitalIn
        elif IO_Type == 'Analog_OUT':
            IOType = rpc.IoType.AnalogOut
        elif IO_Type == 'Analog_IN':
            IOType = rpc.IoType.AnalogIn

        try:
            response = rpc.iovalrd(self.server_ip, typ = IOType, index = index)
            return response.value
        except:
            print("Cannot set IO Value")
            return False
    
    def setIOval_srv(self, request):

        '''
        @brief ROS service handler for setting an IO value.
        @param request ROS service request containing io_type, index, value.
        @return SetIOResponse with status and comment.
        '''

        response = self.setIOvalue(request.io_type, request.index, request.value)
        comment = rpc.iogetpn(server = self.server_ip, typ = request.io_type, index = request.index)
        return SetIOResponse(status = response, comment = comment.value)
    """  """
    def readIOval_srv(self, request):

        '''
        @brief ROS service handler for reading an IO value.
        @param request ROS service request containing io_type and index.
        @return ReadIOResponse with value and comment.
        '''

        response = self.readIOvalue(request.io_type, request.index)
        comment = rpc.iogetpn(server = self.server_ip, typ = request.io_type, index = request.index)
        return ReadIOResponse(value = response.value, comment = comment.value)
    
    def run(self):

        '''
        @brief Main execution loop. Advertises ROS IO services and publishes IO states.
        @details Publishes to /io_states_DOUT, /io_states_DIN, /io_states_AOUT, /io_states_AIN.
        '''
        rospy.loginfo("Starting Fanuc IO Node...")
        while not rospy.is_shutdown():

            # Services 
            rospy.Service('set_io_value', SetIO, self.setIOval_srv)
            rospy.Service('read_io_value', ReadIO, self.readIOval_srv)

            # Digital Outputs
            dout_array = self.generate_iostate_array(rpc.IoType.DigitalOut, 100)
            self.publisher_dout.publish(dout_array)

            # Digital Inputs
            din_array = self.generate_iostate_array(rpc.IoType.DigitalIn, 100)
            self.publisher_din.publish(din_array)

            # Analog Outputs
            aout_array = self.generate_iostate_array(rpc.IoType.AnalogOut, 16)
            self.publisher_aout.publish(aout_array)

            # Analog Inputs
            ain_array = self.generate_iostate_array(rpc.IoType.AnalogIn, 16)
            self.publisher_ain.publish(ain_array)

            self.rate.sleep()

if __name__ == '__main__':

    '''
    @brief Main entry point. Instantiates and runs the Fanuc_IO node.
    '''

    try:
        fanuc_io = Fanuc_IO()
        fanuc_io.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught, shutting down Fanuc_IO.")



'''
@}
'''