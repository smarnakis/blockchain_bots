# sawtooth-sdk-python - Additional custom made files

This directory contains all the additional custom made files created on top of the sawtooth-sdk-python provided by hyperledger [here](https://github.com/hyperledger/sawtooth-sdk-python). 

## Containts:
* **custom_client**
    - **server.py** :  Implements the python server which run in the *sawtooth client* docker container. Thanks to the server, the ros service *sawtooth bridge* can connect and send commands to sawtooth.
    - **custom_client.py** : Contains the wrapper class responsible for organising the IntKey commands into transactions and batches and the to send them to the validator using the REST API. It is used be the *server.py*
    - **client_*.py, sawtoothClient.py** : Implement test clients which should be run EXTERNALY of the docker containers. Each client_*.py uses the sawtoothClient.py cli wrapper to transform the custom user commands to the necessary format for the server. 
* **docker-compose.yaml** : Contains the description of all the necessary docker containers of the sawtooth network that is simulated for this simulation. 

