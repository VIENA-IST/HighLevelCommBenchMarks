# High level communication benchmark tests

For future implementations, is important to test the ability of using a fast interface for distributing orders.
For that, it is tested here some different approaches in order to help us choose future implementations.

## Purpose

This repository is intended for testing the rate of messages capable of being transmitted by the broker of MQTT and
compare it with the capabilities of ROS.

## MQTT results

Test conditions are as follow:
* 1000 messages sent, with format "Message xxxx" where "xxxx" is the number of message.
* same qos in both ends
* using same PC to run the sender and receiver clients Lenovo T480
* same module implementation of MQTT in both endpoints (paho-mqtt)
* comparison between local MQTT broker (mosquitto) running in raspberryPI 3 and online servers
* mean of 5 consecutive tests

> :exclamation: In order to use mosquitto with QoS of 1 or 2, we had to increase max_queued_messages to 10000

Here follows an example of a set of tests printed in the receiver client logged output:
```bash
python -u client_receiver.py --hostname=broker.hivemq.com -p 8000 --qos=0
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
Below is the resume of results performed for **websockets** transport in messages per second

|Server            |Is local?| QoS | Mean Receiving | std    | min    | max      |
|:-----------------|:-------:|:---:|:--------------:|:------:|:------:|:--------:|
|mosquitto         | Yes     |0    | 9754.14        |1735.97 |7316.35 |11653.73  |
|mosquitto         | Yes     |1    | 4917.01        |172.89  |4680.69 | 5125.19  |
|mosquitto         | Yes     |2    | 3663.86        |234.97  |3339.15 | 3999.98  |
|mosquitto (Rpi)   | Yes     |0    | x                 |        |        |          |
|mosquitto (Rpi)   | Yes     |1    | x                 |        |        |          |
|mosquitto (Rpi)   | Yes     |2    | x                 |        |        |          |
|mosquitto.org     | No      |0    | 103.71         | 4.47   | 96.85  | 108.64   |
|mosquitto.org     | No      |1    | 107.87         | 9.85   | 90.55  | 113.12   |
|mosquitto.org     | No      |2    | 57.69          | 0.65   | 56.89  | 58.44    |
|broker.hivemq.com | No      |0    | 3209.51        | 2631.14| 555.91 | 6589.80  |
|broker.hivemq.com | No      |1    | x                 |        |        |          |
|broker.hivemq.com | No      |2    | x                 |        |        |          |




|Server            |Is local?| QoS | Mean Transmitting | std    | min      | max      |
|:-----------------|:-------:|:---:|:-----------------:|:------:|:--------:|:--------:|
|mosquitto         | Yes     |0    | 20855.58          |6606.65 |13611.86  |29385.87  |
|mosquitto         | Yes     |1    | 6370.67           |343.24  |5797.45   |6627.05   |
|mosquitto         | Yes     |2    | 4653.94           |189.67  |4374.28   |4883.84   |
|mosquitto (Rpi)   | Yes     |0    | x                 |        |        |          |
|mosquitto (Rpi)   | Yes     |1    | x                 |        |        |          |
|mosquitto (Rpi)   | Yes     |2    | x                 |        |        |          |
|mosquitto.org     | No      |0    | 6863.24           |1031.36 |5768.09   |8451.10   |
|mosquitto.org     | No      |1    | 106.53            | 8.66   |  91.23   | 111.90   |
|mosquitto.org     | No      |2    | 57.64             | 0.59   |  56.92   | 58.43    |
|broker.hivemq.com | No      |0    | 6382.99           |905.09  | 4832.09  | 7004.41  |
|broker.hivemq.com | No      |1    | x                 |        |          |          |
|broker.hivemq.com | No      |2    | x                 |        |          |          |

Resume of results performed for **tcp** transport

