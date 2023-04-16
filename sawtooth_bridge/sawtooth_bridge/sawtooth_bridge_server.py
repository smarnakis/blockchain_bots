import rclpy
from rclpy.node import Node
from blockchain_bots_interfaces.srv import ConnectToSawtoothSim, SendCommandToSawtoothSim
from .sawtoothClient import sawtoothClient

class Sawtooth(Node):
    """A class to implement the bridge between ROS2 and Sawtooth.

    ...

    Description
    -----------
    A child class of the Node class. This node acts as the server for the 2 services 
    (namely connect_to_sawtooth and send_command_to_sawtooth) that implement the bridge 
    between the other ros2 nodes and the Sawtooth blockchain.

    Attributes
    ----------
    srv1 : rclpy.service.Service
        Service for connection with the python server in the sawtooth client docker container.
    srv2 : rclpy.service.Service
        Service to send commands to the connected sawtooth client docker container.


    Methods
    -------
    connect_to_sawtooth_callback(request, response):
        The callback that implements srv1
    send_command_to_sawtooth_callback (request, response):
        The callback that implements srv2
    """

    def __init__(self):
        super().__init__('minimal_service')
        self.srv1 = self.create_service(ConnectToSawtoothSim, 'connect_to_sawtooth', self.connect_to_sawtooth_callback)
        self.srv2 = self.create_service(SendCommandToSawtoothSim, 'send_command_to_sawtooth', self.send_command_to_sawtooth_callback)
        
    def connect_to_sawtooth_callback (self, request, response):
        """The callback that implements srv1

        ...

        Description
        -----------
        Given the host IP and the port it connects to the respective docker container 
        using the custom made sawtoothClient class. Now one node handles the services 
        from all the robots, the robot_id is used to create each instance.(TODO: Create
        one server node for each robot.)
        """

        host = request.host
        port = request.port
        robot_id = request.robot_id
        if robot_id == 0:
            try:
                self.clientInstance0 = sawtoothClient(host,port)
            except:
                response.result = False
                return response
        else:
            try:
                self.clientInstance1 = sawtoothClient(host,port)
            except:
                response.result = False
                return response
        response.result = True
        self.get_logger().info('Connected to sawtooth client:\nIP: %s:%d for robot_id %d' % (request.host, request.port, request.robot_id))

        return response
    
    def send_command_to_sawtooth_callback (self, request, response):
        """The callback that implements srv2.

        ...

        Description
        -----------
        Based on the robot_id which calls srv2, a set, inc, show or list command is sent to the
        respective sawtooth-client docker container which in term requests the respective transaction
        from the IntKey TP. 
        """
        robot_id = request.robot_id

        if robot_id == 0: # Case for commands sent by 1st robot
            commandType = request.command_type
            stationNum = request.station_num
            value = request.value
            if commandType == "set":
                result = self.clientInstance0.sendSet(stationNum, value)
            elif commandType == "inc":
                result = self.clientInstance0.sendInc(stationNum, value)
            elif commandType == "show":
                result = self.clientInstance0.sendShow(stationNum)
            elif commandType == "list":
                result = self.clientInstance0.sendList()
            else:
                result = "FALSE COMMAND"
        else: # Case for commands sent by 2nd robot
            commandType = request.command_type
            stationNum = request.station_num
            value = request.value
            if commandType == "set":
                result = self.clientInstance1.sendSet(stationNum, value)
            elif commandType == "inc":
                result = self.clientInstance1.sendInc(stationNum, value)
            elif commandType == "show":
                result = self.clientInstance1.sendShow(stationNum)
            elif commandType == "list":
                result = self.clientInstance1.sendList()
            else:
                result = "FALSE COMMAND"
        self.get_logger().info('Transaction by robot_id: %d for commad:\nCommand Type: %s, station%d, value: %d' % (robot_id, commandType, stationNum, value))
        self.get_logger().info('RESPONSE: %s' %(result))
        response.result = result
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = Sawtooth()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


