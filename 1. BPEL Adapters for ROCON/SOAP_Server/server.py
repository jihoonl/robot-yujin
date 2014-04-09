'''
Adapter Layer, v0.01
Author: Chun Woo Park, Dae Young Kim, Moon Kwon Kim
Date: April 8, 2014
'''

from pysimplesoap.server import SoapDispatcher, SOAPHandler
from BaseHTTPServer import HTTPServer
import logging
import time

from adapter_util import AdapterUtility

logging.basicConfig()

class AdapterSOAPServer:
    def __init__(self):
        self.address = "http://localhost:8008/"
        self.adapter_utility = AdapterUtility()

    def adapt(self, nodeID, resourceName, duration, options={}):
        '''
        Adapt a service request to a ROCON request.
        @ param nodeID: Not used
        @ param resourceName: URI
        @ param duration: Invoke an activity for the duration
        @ param options: Command from BP (i.e. forward or cycle_left)
        '''
        print("Adapt...")
        print("Node ID: %s, Resource Name: %s, Duration: %s, Options: %s" % (nodeID, resourceName, duration, options))

        resource_alloc_flag = False
        command_flag = False

        print "ready to request resources"

        # To publish request to allocate resource
        if resourceName:
            print "===================server.py==================="
            time.sleep(1)
            self.adapter_utility.pub_resource_alloc(resourceName, options)

            while resource_alloc_flag !=True:
                time.sleep(0.5)
                resource_alloc_flag = self.adapter_utility.resource_alloc_flag
            print "Receive reply of the allocation request"

        # To publish command to resource
        else :
            print "===================server.py==================="
            time.sleep(1)
            self.adapter_utility.pub_command(duration, options)

            while command_flag !=True:
                time.sleep(0.5)
                command_flag = self.adapter_utility.command_flag
            print "Receive reply of the command"

        return "Result"

    def run(self):
        dispatcher = SoapDispatcher('op_adapter_soap_disp', location = self.address, action = self.address,
                namespace = "http://smartylab.co.kr/products/op/adapter", prefix="tns", trace = True, ns = True)
        dispatcher.register_function('adapt', self.adapt, returns={'out': str},
                args={'nodeID': str, 'resourceName': str, 'duration': str, 'options': str})
        print("Starting a SOAP server for adapter layer of OP...")
        httpd = HTTPServer(("", 8008), SOAPHandler)
        httpd.dispatcher = dispatcher
        httpd.serve_forever()

if __name__ == "__main__":
    soapServer = AdapterSOAPServer()
    soapServer.run()
