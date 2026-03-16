#!/usr/bin/env python3
"""
Visualizer
- Reads x, y from IDs 12, 13
- Plots trajectory using Matplotlib
- Dumps KPIs at the end in results.csv
"""
import struct, sys, argparse, math, csv, os
from datetime import datetime, timezone
import numpy as np
import matplotlib.pyplot as plt

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

DEFAULT_DT = 0.02

class Visualizer:
    def __init__(self, args):
        self.componentId = 2
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = args.port if args.port is not None else 50103

        self.traj_x = []
        self.traj_y = []

        self.path_type = args.path_type
        self.path_length = args.path_length
        self.path_x, self.path_y = self.make_reference()

        # Prefer explicit --Kp/--Ki/--Kd, but allow controller-style *_lat aliases.
        self.Kp = args.Kp if args.Kp is not None else args.Kp_lat
        self.Ki = args.Ki if args.Ki is not None else args.Ki_lat
        self.Kd = args.Kd if args.Kd is not None else args.Kd_lat
        self.noise = args.noise; self.disturbance = args.disturbance
        self.save_plot = args.save_plot
        self.out_csv = args.out_csv
        self.run_name = args.run_name
        self.experiment = args.experiment
        self.seed = args.seed
        self.init_x = args.init_x
        self.init_y = args.init_y
        self.init_theta = args.init_theta

        self.simulationStepNs = 0
        self.simulationStep = DEFAULT_DT
        self.totalSimulationTimeNs = 0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10,6))
        self.line_robot, = self.ax.plot([], [], 'b-', linewidth=2, label='Robot Trajectory')
        self.line_path, = self.ax.plot(self.path_x, self.path_y, 'r--', linewidth=2, label='Reference Path')
        self.ax.set_title(f"Line Following Controller - Path: {self.path_type.capitalize()}")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.legend()
        self.ax.grid(True)

    def make_reference(self):
        xs = np.linspace(0, self.path_length, 400)
        if self.path_type == 'straight':
            ys = np.zeros_like(xs)
        elif self.path_type == 'sine':
            ys = np.sin(xs * 0.5) * 2.0
        elif self.path_type == 'curved':
            ys = 0.5 * xs + 2.0 * np.sin(0.2 * xs)
        else:
            ys = np.zeros_like(xs)
        return xs, ys

    def safe_recv_double(self, cid, default=None):
        try:
            packed = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, cid)
        except Exception:
            packed = None
        if not packed:
            return default
        n = struct.calcsize('=d')
        return struct.unpack('=d', packed[:n])[0]

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        vsiCommonPythonApi.waitForReset()

        self.totalSimulationTimeNs = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStepNs = vsiCommonPythonApi.getSimulationStep()
        if self.simulationStepNs and self.simulationStepNs>0:
            self.simulationStep = self.simulationStepNs / 1e9
        else:
            self.simulationStepNs = int(DEFAULT_DT * 1e9)
            self.simulationStep = DEFAULT_DT

        nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
        plot_step = 0

        while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTimeNs:
            x = self.safe_recv_double(12, default=None)
            y = self.safe_recv_double(13, default=None)
            
            if x is not None and y is not None:
                self.traj_x.append(x)
                self.traj_y.append(y)
                
                # Update plot occasionally for speed
                plot_step += 1
                if plot_step % 10 == 0:
                    self.line_robot.set_data(self.traj_x, self.traj_y)
                    self.ax.relim()
                    self.ax.autoscale_view()
                    plt.pause(0.001)

            nextExpectedTime += self.simulationStepNs
            vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

        # Cleanup & compute stats
        plt.ioff()
        if self.save_plot:
            plt.savefig(self.save_plot)
        print("Visualization complete.")
        # plt.show() # Disabled so it doesn't block automated scripts
        self.save_metrics()

    def save_metrics(self):
        if len(self.traj_x) == 0:
            print("Visualizer: no trajectory samples collected!")
            return
            
        # Interpolate actual Y based on X to compute error over the path
        # But if the path is curved, this lateral proxy is an approximation.
        ref_y = np.interp(self.traj_x, self.path_x, self.path_y)
        errors = np.array(self.traj_y) - ref_y

        overshoot = float(np.max(np.abs(errors)))
        tol = 0.05
        settling_idx = len(errors)
        for i in range(len(errors)):
            # Settling time: first time after which the tracking error stays
            # within a fixed tolerance band around zero.
            if np.all(np.abs(errors[i:]) <= tol):
                settling_idx = i
                break
                
        # Steady state error across the last 10%
        idx_10pct = max(1, len(errors) // 10)
        steady_state_error = float(np.mean(np.abs(errors[-idx_10pct:])))
        final_x = float(self.traj_x[-1])
        dt = float(self.simulationStep if self.simulationStep > 0 else DEFAULT_DT)
        settling_time_s = float(settling_idx * dt)
        duration_s = float(len(errors) * dt)

        file_exists = os.path.isfile(self.out_csv)
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
        with open(self.out_csv, 'a', newline='') as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow([
                    'TimestampUtc',
                    'Experiment',
                    'RunName',
                    'Path',
                    'PathLength',
                    'Kp',
                    'Ki',
                    'Kd',
                    'Noise',
                    'Disturbance',
                    'Seed',
                    'InitX',
                    'InitY',
                    'InitTheta',
                    'FinalX',
                    'Samples',
                    'Dt',
                    'DurationS',
                    'Overshoot',
                    'SettlingTimeS',
                    'SteadyStateError',
                ])
            w.writerow([
                datetime.now(timezone.utc).isoformat(),
                self.experiment,
                self.run_name,
                self.path_type,
                self.path_length,
                self.Kp,
                self.Ki,
                self.Kd,
                self.noise,
                self.disturbance,
                self.seed,
                self.init_x,
                self.init_y,
                self.init_theta,
                final_x,
                len(errors),
                dt,
                duration_s,
                overshoot,
                settling_time_s,
                steady_state_error,
            ])
        print(f"Metrics saved to {self.out_csv} -> Overshoot: {overshoot:.3f}, Settling(s): {settling_time_s:.2f}, SSE: {steady_state_error:.3f}, FinalX: {final_x:.2f}")

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--port', type=int, default=50103)
    p.add_argument('--path-type', choices=['straight','sine','curved'], default='straight')
    p.add_argument('--path-length', type=float, default=20.0)
    p.add_argument('--Kp', type=float, default=None)
    p.add_argument('--Ki', type=float, default=None)
    p.add_argument('--Kd', type=float, default=None)
    p.add_argument('--Kp_lat', type=float, default=1.5)
    p.add_argument('--Ki_lat', type=float, default=0.05)
    p.add_argument('--Kd_lat', type=float, default=0.1)
    p.add_argument('--noise', type=float, default=0.02)
    p.add_argument('--disturbance', type=float, default=0.0)
    p.add_argument('--save-plot', type=str, default='')
    p.add_argument('--out-csv', type=str, default='results.csv')
    p.add_argument('--experiment', type=str, default='')
    p.add_argument('--run-name', type=str, default='')
    p.add_argument('--seed', type=int, default=None)
    p.add_argument('--init-x', type=float, default=0.0)
    p.add_argument('--init-y', type=float, default=0.0)
    p.add_argument('--init-theta', type=float, default=0.0)
    return p.parse_args()

def main():
    args = parse_args()
    Visualizer(args).mainThread()

if __name__ == '__main__':
    main()
