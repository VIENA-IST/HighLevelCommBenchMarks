#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2018 Bruno Tibério
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import logging
import sys
import signal

# import queue
import paho.mqtt.client as mqtt




protocol = mqtt.MQTTv311
# mqtt topics to be used with epos
Topic = 'VIENA/steering/'  # base topic
status = 'VIENA/connected/'  # status topic

# ---------------------------------------------------------------------------
# define signal handlers for systemd signals
# ---------------------------------------------------------------------------


def signal_handler(signum, frame):
    if signum == signal.SIGINT:
        logging.info('Received signal INTERRUPT... exiting now')
    if signum == signal.SIGTERM:
        logging.info('Received signal TERM... exiting now')
    return


def main():
    """Perform steering wheel calibration.

    Ask user to turn the steering wheel to the extremes and finds the max
    """

    import argparse
    import time

    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='MQTT controller')
    parser.add_argument('--channel', '-c', action='store', default='can0',
                        type=str, help='Can channel to be used', dest='channel')
    parser.add_argument('--bus', '-b', action='store',
                        default='socketcan', type=str, help='Bus type', dest='bus')
    parser.add_argument('--rate', '-r', action='store', default=None,
                        type=int, help='bitrate, if applicable', dest='bitrate')
    parser.add_argument('--hostname', action='store', default='localhost', type=str,
                        help='hostname for mqtt broker', dest='hostname')
    parser.add_argument('--port', '-p', action='store', default=8080, type=int,
                        help='port for mqtt broker', dest='port')
    parser.add_argument('--path', action='store', default='/mqtt', type=str,
                        help='path to be used in mqtt broker', dest='path')
    parser.add_argument('--transport', action='store', default='websockets', type=str,
                        help='transport layer used in mqtt broker', dest='transport')
    parser.add_argument('--qos', action='store', default=2, type=int,
                        help='Quality of Service to be used', dest='qos', choices=[0, 1, 2])
    args = parser.parse_args()
    # ---------------------------------------------------------------------------
    # Important constants and definitions to be used
    # ---------------------------------------------------------------------------
    # mqtt constants
    hostname = args.hostname
    port = args.port
    transport = args.transport
    qos = args.qos
    # ---------------------------------------------------------------------------
    # set up logging to file to used debug level saved to disk
    # ---------------------------------------------------------------------------
    logging.basicConfig(level=logging.DEBUG,
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='mqtt_receiver.log',
                        filemode='w')
    # ---------------------------------------------------------------------------
    # define a Handler which writes INFO messages or higher in console
    # ---------------------------------------------------------------------------
    console = logging.StreamHandler()
    console.setLevel(logging.DEBUG)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)
    # ---------------------------------------------------------------------------
    # Defines of callback functions
    # ---------------------------------------------------------------------------

    def on_message(self, userdata, message):
        if message.topic == Topic:
            logging.debug('Received message: {0:04d} on topic {1} with QoS {2}'.format(
                int.from_bytes(message.payload, 'little'), message.topic, message.qos))
            if int.from_bytes(message.payload, 'little') == 0:
                self.t_start=time.time()
                self.index = 0
                logging.info("Received message 0 with timestamp {0}".format(message.timestamp))
            self.index = self.index + 1
            if int.from_bytes(message.payload, 'little') == 999:
                self.t_end = time.time()
                logging.info("Received {1} messages. Last message timestamp {0}".format(message.timestamp, self.index))
                t_elapsed = (self.t_end-self.t_start)
                logging.info('Rate is {0:6.2f} messages per second'.format(1000.0/t_elapsed))
        elif message.topic == status:
            logging.info('Received message: "' + str(message.payload.decode('UTF-8')) + '" on topic '
                         + message.topic + ' with QoS ' + str(message.qos))
        else:
            logging.info("Received message :" + str(message.payload) + " on topic "
                         + message.topic + " with QoS " + str(message.qos))

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            logging.info("Connected to server")
        else:
            logging.info("Failed to connect to server")
        return

    # ---------------------------------------------------------------------------
    # end of callback defines
    # ---------------------------------------------------------------------------
    # ---------------------------------------------------------------------------
    # mqtt client configure
    # ---------------------------------------------------------------------------
    global client
    client = mqtt.Client(protocol=protocol, transport=transport)
    client.on_connect = on_connect
    client.on_message = on_message
    client.t_start = 0
    client.t_end = 0
    client.index = 0
    noFaults = True
    try:
        client.connect(hostname, port=port)
        # client.loop_start()
    except Exception as e:
        logging.info('Connection failed: {0}'.format(str(e)))
        noFaults = False
    finally:
        if not noFaults:
            client.loop_stop(force=True)
            logging.info('Failed to connect to broker...Exiting')
            return
    client.subscribe(Topic, qos)
    client.subscribe(status, 2)
    try:
        client.loop_forever()
    except KeyboardInterrupt as e:
        logging.info('[Main] Got exception {0}... exiting now'.format(e))
    finally:
        client.loop_stop(force=True)
    return


if __name__ == '__main__':
    main()
