#!/usr/bin/python
'''
Adapter Layer, v0.01
Author: Chun Woo Park, Dae Young Kim, Moon Kwon Kim
Date: April 8, 2014
'''

from pysimplesoap.server import SoapDispatcher, SOAPHandler
from BaseHTTPServer import HTTPServer
import logging
import time
from adapter_util import AdapterUtility, Allocator

logging.basicConfig()


class AdapterSOAPServer:
    def __init__(self):
        self.address = "http://localhost:8008/"
        self.adapter_utility = AdapterUtility()
        self.resource_alloc_flag = False

    def adapt(self, nodeID, resourceName, duration, options={}):
        print resourceName
        resource = {'nodeID': nodeID, 'resourceName': resourceName, 'duration': duration, 'options': options}
        self.adapter_utility.allocate(resource)

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
