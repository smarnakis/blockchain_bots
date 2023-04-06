import argparse
import getpass
import logging
import os
import sys
import traceback
import pkg_resources
import socket
import cbor
import json

from sawtooth_signing import create_context
from sawtooth_signing import CryptoFactory

from custom_client import IntkeyClient


DEFAULT_URL = "http://sawtooth-rest-api"
DEFAULT_PORT = ":8008"

DEFAULT_HOST = "0.0.0.0"

class Arguements:
    def __init__(self,name,value,wait=None,url=DEFAULT_URL,filename=None):
        self.name = name
        self.value = value
        self.url = url
        self.filename = filename
        self.wait = wait


def server_program():
    # get the hostname
    # host = socket.gethostname()
    print(DEFAULT_HOST)
    robot_num = int(os.getenv('HOSTNAME').split('-')[-1])
    # robot_num = int(_get_node_num("info.txt"))
    # robot_num = 0
    port = 9000+robot_num  # initiate port no above 1024
    
    url = DEFAULT_URL+"-"+str(robot_num)+DEFAULT_PORT
    print("Rest api connection:", url)
    server_socket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    server_socket.bind((DEFAULT_HOST, port))  # bind host address and port together
    print("HOST connection:", DEFAULT_HOST,port)
    # configure how many client the server can listen simultaneously
    server_socket.listen(2)
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))
    while True:
        # receive data stream. it won't accept data packet greater than 1024 bytes
        data_raw = conn.recv(1024).decode()
        # print(data_raw)
        try:
            data = json.loads(data_raw)
            
            if not data:
                # if data is not received break
                break
            try:
                if data['action'] == "set":
                    args = Arguements(data['name'],data['value'],url=url)
                    response = do_set(args)
                elif data['action'] == "inc":
                    args = Arguements(data['name'],data['value'],url=url)
                    response = do_inc(args)
                elif data['action'] == "dec":
                    args = Arguements(data['name'],data['value'],url=url)
                    response = do_dec(args)
                elif data['action'] == "show":
                    args = Arguements(data['name'],data['value'],url=url)
                    response = do_show(args)
                elif data['action'] == "list":
                    args = Arguements(data['name'],data['value'],url=url)
                    response = do_list(args)
                    print(response)
                else:
                    response = "INVALID COMMAND"
                print("from connected user: " + str(data))
            except KeyError:
                response = "INVALID COMMAND"
        except ValueError:
            response = "INVALID COMMAND"

        print("Treated command", data_raw ,"and got result of type:", type(response), "\nand value:", response)
        conn.send(response.encode())  # send data to the client


    conn.close()  # close the connection


def do_set(args):
    name, value, wait = args.name, args.value, args.wait
    client = _get_client(args)
    response = client.set(name, value, wait)
    # print(response)
    return response

def do_inc(args):
    name, value, wait = args.name, args.value, args.wait
    client = _get_client(args)
    response = client.inc(name, value, wait)
    # print(response)
    return response

def do_dec(args):
    name, value, wait = args.name, args.value, args.wait
    client = _get_client(args)
    response = client.dec(name, value, wait)
    # print(response)
    return response

def do_show(args):
    name = args.name
    client = _get_client(args,False)
    value = client.show(name)
    response = '{}: {}'.format(name, value)
    # print(response)
    return response

def do_list(args):
    response = '|'
    client = _get_client(args,False)
    results = client.list()
    # print("Results from client:", results)
    for pair in results:
        for name, value in pair.items():
            response += ' {}: {} |'.format(name, value)
    # print("Response in function:",response)
    return response


def _get_keyfile():
    real_user = getpass.getuser()
    home = os.path.expanduser("~")
    key_dir = os.path.join(home, ".sawtooth", "keys")

    return '{}/{}.priv'.format(key_dir, real_user)

def _get_client(args,read_key_file=True):
    return IntkeyClient(
        url=DEFAULT_URL+":"+DEFAULT_PORT if args.url is None else args.url,
        keyfile=_get_keyfile() if read_key_file else None)

def _read_file(filePath):
    fd = open(filePath)
    data = fd.read().strip()
    fd.close()
    return data

def _get_node_num(filePath):
    num = _read_file(filePath).split("-")[-1]
    return num

if __name__ == '__main__':
    server_program()