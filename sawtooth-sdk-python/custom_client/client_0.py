from sawtoothClient import sawtoothClient

# host = 'sawtooth-shell'
# host = '192.168.192.4'
host = '127.0.0.1'


def client_program():
  
  port = 9000  # socket server port number

  clientInst = sawtoothClient(host,port)
  clientInst.initialiseValues()
  print(host,port)
  message = input(" -> ")  # take input

  while message.lower().strip() != 'bye':
    data = clientInst.sendInc(int(message))
    print('Received from server: ' + data)  # show in terminal
    data = clientInst.sendList()
    print('Received from server: ' + data) 
    message = input(" -> ")  # again take input

  clientInst.deconnect()  # close the connection


def main():
  client_program()
  return 0

if __name__ == '__main__':
  r = main()