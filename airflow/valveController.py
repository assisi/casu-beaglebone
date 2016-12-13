########################################################
#########   valve controller BB firmware   #############
#########         created: 11/30/15        #############
#########            D.M. LARICS           #############
######################################################## 


from pymodbus.client.sync import ModbusTcpClient
import zmq

subAddr = 'tcp://*:1555'
valveAddr = '10.42.0.254'
commandRegAddr = [40003, 40004, 40005, 40006]

festoValveBlock = ModbusTcpClient(valveAddr)
zmqContext = zmq.Context(1)
zmqSub = zmqContext.socket(zmq.SUB)
zmqSub.bind(subAddr)
zmqSub.setsockopt(zmq.SUBSCRIBE, '')

# initialize valve states
valveStates = [0, 0, 0, 0];

for i in range(0, 4):
	festoValveBlock.write_register(commandRegAddr[i], valveStates[i])

command2state = {'On': 1, 'Off': 0}

while 1:
	# receive valve message
	[name, device, command, data] = zmqSub.recv_multipart()
	
	# get referenced valve number
	valve = int(name[-3:])
	
	# get valve address byte and number in byte
	[byteIdx, valveIdx] = divmod(valve-1, 8)
	valveIdx = valveIdx + 1
	
	# check valve state ("on" or "off")
	if command2state[command] :

		# set valve bit to "1"
		valveStates[byteIdx] = valveStates[byteIdx] | (1 << (valveIdx-1))

	else:

		# set valve bit to "0"
		valveStates[byteIdx] = valveStates[byteIdx] & ~(1 << (valveIdx-1))

	# send demanded valve states to FESTO valve block
	festoValveBlock.write_register(commandRegAddr[byteIdx], valveStates[byteIdx])



