# High level communication benchmark tests

For future implementations, is important to test the hability of using a fast interface for distributing orders.
For that, it is tested here some different approachs in order to help us choose future implementations.

## Purpose

This repository is intended for testing the rate of messages capable of being transmitted by the broker of MQTT and
compare it with the capabilities of ROS.

## MQTT results

Test conditions are as follow:
* 1000 messages sent, with format "Message xxxx" where "xxxx" is the number of message.
* same qos in both ends
* using same PC to run the sender and receiver clients Lenovo T480
* same module implementation of MQTT in both endpoints (paho-mqtt)
* comparison between local MQTT broker (mosquitto) running in raspberryPI 3 online servers
* mean of 5 consecutive tests


Here follows an example of a set of tests printed in the receiver client logged output:
```bash
python -u client_receiver.py --hostname=broker.hivemq.com -p 8000
root                : INFO     Connected to server
root                : INFO     Received message 0
root                : INFO     Received message: "Connected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received 1000 messages
root                : INFO     Rate is 800.09 messages per second
root                : INFO     Received message: "Disconnected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received message 0
root                : INFO     Received message: "Connected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received 1000 messages
root                : INFO     Rate is 813.55 messages per second
root                : INFO     Received message: "Disconnected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received message 0
root                : INFO     Received message: "Connected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received 1000 messages
root                : INFO     Rate is 1033.60 messages per second
root                : INFO     Received message: "Disconnected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received message 0
root                : INFO     Received message: "Connected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received 1000 messages
root                : INFO     Rate is 836.48 messages per second
root                : INFO     Received message: "Disconnected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received message 0
root                : INFO     Received message: "Connected" on topic VIENA/connected/ with QoS 2
root                : INFO     Received 1000 messages
root                : INFO     Rate is 633.22 messages per second
root                : INFO     Received message: "Disconnected" on topic VIENA/connected/ with QoS 2

```
Below is the resume of results performed for **websockets** transport

|Server            |Is local?| QoS | Mean R. Sending | Mean R. Receiving | std | min | max |
|:-----------------|:-------:|:---:|:---------------:|:-----------------:|:---:|:---:|:---:|
|mosquitto         | Yes     |0    | x               | x                 |     |     |     |
|mosquitto         | Yes     |1    | x               | x                 |     |     |     |
|mosquitto         | Yes     |2    | x               | x                 |     |     |     |
|mosquitto.org     | No      |0    | x               | x                 |     |     |     |
|mosquitto.org     | No      |1    | x               | x                 |     |     |     |
|mosquitto.org     | No      |2    | x               | x                 |     |     |     |
|broker.hivemq.com | No      |0    | x               | 823.388           | 142.45 | 633.22 | 1,033.60 |
|broker.hivemq.com | No      |1    | x               | x                 |        |        |          |
|broker.hivemq.com | No      |2    | x               | x                 |        |        |          |

Resume of results performed for **tcp** transport

|Server            |Is local?| QoS | Mean R. Sending | Mean R. Receiving | std | min | max |
|:-----------------|:-------:|:---:|:---------------:|:-----------------:|:---:|:---:|:---:|
|mosquitto         | Yes     |0    | x               | x                 |     |     |     |
|mosquitto         | Yes     |1    | x               | x                 |     |     |     |
|mosquitto         | Yes     |2    | x               | x                 |     |     |     |
|mosquitto.org     | No      |0    | x               | x                 |     |     |     |
|mosquitto.org     | No      |1    | x               | x                 |     |     |     |
|mosquitto.org     | No      |2    | x               | x                 |     |     |     |
|broker.hivemq.com | No      |0    | x               | 14,705.08         |2,659.12| 12,030.96| 18,588.56|
|broker.hivemq.com | No      |1    | x               | x                 |        |          |          |
|broker.hivemq.com | No      |2    | x               | x                 |        |          |          |


#

 