# Serial Agent

The serial agent connects a microcontroller to a host processor via a serial link.
The host processor is running ROS2 on a desktop operating system (e.g. Linux, MacOS, Windows)
and the code is typically written in either C++ or Python.
The microcontroller code is typically written in C and runs a under a real-time
operating system (e.g. FreeRTOS, Zephyr, etc.)
The serial agent allows microcontroller to behave as a first class member of a ROS2 network.
In short, the serial agent mirrors ROS2 messages between host processor and the microcontroller.

## Terminology

ROS2 implements a message passing system that implements 3 basic protocols:

* Topics:
  Topics are a mulit-cast system where a message is broadcast to multiple recipients.
  The broadcast operation is called a publishing and the receiving operation is called subscribing.

* Servers:
  Servers implement remote procedure call protocol whereby a client sends single request message
  to a server that responds with a single response message.

* Parameters:
  (More here)

Each different topic and server is given a unique string name that looks like Unix style file name
using forward slashes ('/') as the separator.

In ROS terminology:

* Message:
  A message is the packet of information that sent from a publisher to its subscribers.
  Each subscriber gets a duplicate copy of the message.

* Service:
  A service is broken into a client and a server.
  The client creates a request (message), sends it to the server, which processes it,
  and the create a response (message) and returns it back to the client.
  The reason why "message" is in parenthesis is because to avoid confusing them topic messages
  (see immediately above.)

  Service is specifies to messages -- a request message and a response message.
  The service client creates a request, sends it to the server, the server reads the request,
  processes it, creates a response, and 

The host agent marshals message 

There is a duality for Topics and Servers whereby either the host processor or the microcontroller
can initiate topic p


### Microcontroller Subscription:

Host Subscribes to topic:

2. Host agent callback occurs when message is received.
3. Message is encoded and sent to the microcontroller.
4. The microcontroller decodes the message.
5. The message is handed off to the appropriate RTOS task.

### Microcontroller Publisher.

1. Microcontroller creates the message.
2. Message is encoded and sent over to the host.
3. The host decodes the message.
4. The host publishes the message to topic

### Microcontroller Server:

Host becomes a server.

1. Host receives a request and calls appropriate callback.
2. Request is encoded and sent to microcontroller.
3. Request is decoded.
4. Request is processed.
4. Response is generated.

# https://roboticsbackend.com/ros-asyncspinner-example/
# https://stackoverflow.com/questions/39089776/python-read-named-pipe

### Microcontroller Client

1. Microcontroller creates a Request.
2. Request is encoded and sent to host
3. Host decodes Request.
4. Host sends Request out to ROS2 server.
5. A host callback occurs when the ROS2 server responds.
6. The host encodes the Response and sends it to microcontroller
7. The microcontroller decodes the response and returns it to the appropriate RTOS response queue.

