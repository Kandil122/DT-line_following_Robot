#!/usr/bin/env python3
"""
Controller
- Reads x, y, theta on IDs 12, 13, 14
- Publishes v, omega on IDs 16, 17
- Implements PID + Feedforward target path follower.
"""
import struct, sys, argparse, math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

DEFAULT_DT = 0.02

class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = args.port if args.port is not None else 50102

        self.Kp_lat = args.Kp_lat
        self.Ki_lat = args.Ki_lat
        self.Kd_lat = args.Kd_lat
        self.Kp_head = args.Kp_head
        self.v_nom = args.v_nom

        self.prev_lat_err = 0.0
        self.int_lat_err = 0.0

        self.simulationStepNs = 0
        self.simulationStep = DEFAULT_DT
        self.totalSimulationTimeNs = 0

        self.path_type = args.path_type

    def reference_path(self, x):
        if self.path_type == "straight":
            return 0.0, 0.0
        elif self.path_type == "sine":
            y = 2.0 * math.sin(0.5 * x)
            dy_dx = 2.0 * 0.5 * math.cos(0.5 * x)
            return y, dy_dx
        elif self.path_type == "curved":
            y = 0.5 * x + 2.0 * math.sin(0.2 * x)
            dy_dx = 0.5 + 2.0 * 0.2 * math.cos(0.2 * x)
            return y, dy_dx
        else:
            return 0.0, 0.0

    def safe_recv_double(self, cid, default=0.0):
        try:
            packed = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, cid)
        except Exception:
            packed = None
        if not packed:
            return default
        n = struct.calcsize('=d')
        return struct.unpack('=d', packed[:n])[0]

    def send_double(self, cid, val):
        vsiCanPythonGateway.setCanId(cid)
        payload = struct.pack('=d', float(val))
        vsiCanPythonGateway.setCanPayloadBits(payload, 0, 64)
        vsiCanPythonGateway.setDataLengthInBits(64)
        vsiCanPythonGateway.sendCanPacket()

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        vsiCommonPythonApi.waitForReset()

        self.totalSimulationTimeNs = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStepNs = vsiCommonPythonApi.getSimulationStep()
        if self.simulationStepNs and self.simulationStepNs > 0:
            self.simulationStep = self.simulationStepNs / 1e9
        else:
            self.simulationStepNs = int(DEFAULT_DT * 1e9)
            self.simulationStep = DEFAULT_DT

        nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

        while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTimeNs:
            # Read state
            x = self.safe_recv_double(12, default=0.0)
            y = self.safe_recv_double(13, default=0.0)
            theta = self.safe_recv_double(14, default=0.0)

            # Control Calculations
            y_ref, dy_dx = self.reference_path(x)
            desired_theta = math.atan2(dy_dx, 1.0)

            lat_err = y_ref - y
            heading_err = (desired_theta - theta + math.pi) % (2 * math.pi) - math.pi
            
            # PID terms
            dt = self.simulationStep if self.simulationStep>0 else DEFAULT_DT
            d_lat = (lat_err - self.prev_lat_err) / dt
            self.int_lat_err += lat_err * dt
            self.prev_lat_err = lat_err

            # Limit integral
            self.int_lat_err = max(-5.0, min(5.0, self.int_lat_err))

            omega_cmd = (self.Kp_lat * lat_err +
                         self.Ki_lat * self.int_lat_err +
                         self.Kd_lat * d_lat +
                         self.Kp_head * heading_err +
                         dy_dx * 0.5)

            v_cmd = self.v_nom * max(0.3, 1.0 - abs(heading_err))

            # Send commands
            self.send_double(16, v_cmd)
            self.send_double(17, omega_cmd)

            nextExpectedTime += self.simulationStepNs
            vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--port', type=int, default=50102)
    p.add_argument('--path-type', choices=['straight','sine','curved'], default='straight')
    p.add_argument('--Kp_lat', type=float, default=1.5)
    p.add_argument('--Ki_lat', type=float, default=0.05)
    p.add_argument('--Kd_lat', type=float, default=0.1)
    # Convenience aliases for single-PID sweeps used in experiments/reports.
    # If provided, these override the lateral PID gains above.
    p.add_argument('--Kp', type=float, default=None)
    p.add_argument('--Ki', type=float, default=None)
    p.add_argument('--Kd', type=float, default=None)
    p.add_argument('--Kp_head', type=float, default=2.5)
    p.add_argument('--v_nom', type=float, default=1.0)
    args = p.parse_args()
    if args.Kp is not None:
        args.Kp_lat = args.Kp
    if args.Ki is not None:
        args.Ki_lat = args.Ki
    if args.Kd is not None:
        args.Kd_lat = args.Kd
    return args

def main():
    args = parse_args()
    Controller(args).mainThread()

if __name__ == '__main__':
    main()
