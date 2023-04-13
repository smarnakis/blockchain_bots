from socket import socket
import json
import time

# host = '127.0.0.1' 

# Dictionary containing all the possible commands that can be sent in the proper format
commands = {"set_msgs" : [{"action" : "set", "name" : "station0", "value" : 1},
                    {"action" : "set", "name" : "station1", "value" : 0},
                    {"action" : "set", "name" : "station2", "value" : 1},
                    {"action" : "set", "name" : "station3", "value" : 0}
                    ],
            "inc_msgs" : [{"action" : "inc", "name" : "station0", "value" : 1},
                    {"action" : "inc", "name" : "station1", "value" : 1},
                    {"action" : "inc", "name" : "station2", "value" : 1},
                    {"action" : "inc", "name" : "station3", "value" : 1},
                    ],
            "show_msgs" : [{"action" : "show", "name" : "station0", "value" : 1},
                    {"action" : "show", "name" : "station1", "value" : 1},
                    {"action" : "show", "name" : "station2", "value" : 1},
                    {"action" : "show", "name" : "station3", "value" : 1},
                    ],
            "list_msgs" : {"action" : "list", "name" : "station0", "value" : 1}
}

class sawtoothClient():
  """Class for socket connection with sawtooth-client and command farwarding

  ...

  Description
  -----------
  When instantiated, it creates a socket connection with the requested sawtooth-client docker
  container. Then is wraps, encodes and sends the incoming commands from the robot to the 
  connected sawtooth-client.

  Methods
  -------
  _connect(): 
  Perform the socket connection.

  _sendMessage(message):
  Send a message using the socket and return the result.

  initialiseValues():
  Request a set command for all the stations the are to be represented in the blockchain.

  sendSet(stationNum, value=1):
  Request a set command for the requested station and value.

  sendInc(stationNum, value=1):
  Request an inc command for the requested station and value.

  sendShow(stationNum):
  Request a show command for the requested station and value.

  sendList():
  Request a list command for all the stations. 
  """

  def __init__(self, host='127.0.0.1', port=9000):
    self.host = host
    self.port = port
    self.socket = socket()
    self._connect()

  def _connect(self):
    self.socket.connect((self.host, self.port))

  def _sendMessage(self,message):
    self.socket.send(message.encode()) # Encode the message before it is sent.
    feedbackResp = self.socket.recv(1024).decode() 
    return feedbackResp

  def initialiseValues(self):
    messages_raw = commands['set_msgs']
    for message_raw in messages_raw:
      message = json.dumps(message_raw)
      feedbackResp = self._sendMessage(message)
      time.sleep(0.1)

  def sendSet(self, stationNum, value=1):
    message_raw = commands["set_msgs"][stationNum]
    message_raw['value'] = value
    message = json.dumps(message_raw)
    feedbackResp = self._sendMessage(message)
    time.sleep(0.1)
    return feedbackResp

  def sendInc(self, stationNum, value=1):
    message_raw = commands["inc_msgs"][stationNum]
    message_raw['value'] = value
    message = json.dumps(message_raw)
    feedbackResp = self._sendMessage(message)
    time.sleep(0.1)
    return feedbackResp

  def sendShow(self, stationNum):
    message_raw = commands["show_msgs"][stationNum]
    message = json.dumps(message_raw)
    feedbackResp = self._sendMessage(message)
    time.sleep(0.1)
    return feedbackResp

  def sendList(self):
    message_raw = commands["list_msgs"]
    message = json.dumps(message_raw)
    feedbackResp = self._sendMessage(message)
    time.sleep(0.1)
    return feedbackResp