#!/usr/bin/env python3
"""Fit the unstable zero-command mode; evaluate an experimental Hall compensator.

Requires numpy/scipy. No robot I/O. Outputs are evidence, not a safety certificate:
the plant is lumped inertia/damping, not an identified full FOC/mechanical model.
"""
import argparse
import json
import math
from pathlib import Path
import struct

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import bilinear

# DC gain 1. Poles -5/-110 rad/s; high-frequency gain 1.2.
NUM = np.array([1.2, 46.6, 550.0])
DEN = np.array([1.0, 115.0, 550.0])


def fit(capture, axis):
    data = np.array([(t, struct.unpack('<ff', bytes.fromhex(h))[1])
                     for t, ident, size, h in capture['frames']
                     if ident >> 5 == axis and ident & 31 == 9 and size == 8])
    data[:, 0] -= data[-1, 0]
    data = data[(data[:, 0] > -.5) & (data[:, 0] < -.015)]
    if len(data) < 25:
        raise ValueError('Need at least 25 samples in final oscillation window')
    t, v = data.T
    result = least_squares(
        lambda x: x[0] * np.exp(x[1] * t) * np.sin(x[2] * t + x[3]) + x[4] - v,
        [4, 6, 44, 1, 0], bounds=([0, 0, 25, -10, -1], [20, 30, 80, 10, 1]))
    sigma, omega = result.x[1:3]
    s = sigma + 1j * omega
    # H_PLL=b^2/(s+b)^2, C=P+I/s, G=1/(2*pi*J*s+B).
    bandwidth, p, i = 30., .265, .3975
    cj, cb = 2*np.pi*s*s*(s+bandwidth)**2, s*(s+bandwidth)**2
    y = -(p*s+i)*bandwidth**2
    inertia, damping = np.linalg.solve([[cj.real, cb.real], [cj.imag, cb.imag]], [y.real, y.imag])
    if inertia <= 0 or damping < 0:
        raise ValueError('Fit incompatible with assumed passive mechanical plant')
    return dict(frequency_hz=omega/(2*np.pi), growth_per_s=sigma,
                fit_rms_rps=float(np.sqrt(np.mean(result.fun**2))),
                equivalent_inertia_kg_m2=inertia, damping_nm_per_rps=damping)


def roots(inertia, damping, compensated):
    plant = np.polymul(np.polymul([2*np.pi*inertia, damping], [1, 0]), [1, 60, 900])
    num, den = (NUM, DEN) if compensated else ([1], [1])
    return np.roots(np.polyadd(np.polymul(plant, den), np.polymul([.265*900, .3975*900], num)))


def simulate(inertia, damping, compensated, friction, duration=8):
    dt = 1/8000
    b, a = bilinear(NUM, DEN, fs=8000)
    position, speed, pll_position, pll_speed = .009, .015, 0., 0.
    integral = torque = state1 = state2 = 0.
    samples = []
    for k in range(int(duration/dt)):
        t = k*dt
        reference = max(0, min(.42, .3*(t-1), .3*(5-t)))
        count = math.floor(position*60)
        pll_position += dt*pll_speed
        error = count-math.floor(pll_position)
        pll_position += dt*60*error
        pll_speed += dt*900*error
        if abs(pll_speed) < .5*dt*900:
            pll_speed = 0.
        estimate = pll_speed/60
        feedback = estimate
        if compensated:
            feedback = b[0]*estimate+state1
            state1 = b[1]*estimate-a[1]*feedback+state2
            state2 = b[2]*estimate-a[2]*feedback
        error = reference-feedback
        raw = .265*error+integral
        desired = max(-.8, min(.8, raw))
        integral = integral*.99 if abs(raw) > .8 else integral+dt*.3975*error
        torque += dt*1000*(desired-torque)
        force = torque-damping*speed
        if abs(speed) < .0001 and abs(force) < friction:
            speed = 0.
        else:
            speed += dt*(force-friction*np.sign(speed if abs(speed) > .0001 else force))/(2*np.pi*inertia)
        position += dt*speed
        if k % 80 == 0:
            samples.append([t, reference, speed, estimate, torque, position])
        if abs(speed) > 20:
            break
    return np.array(samples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--axis', type=int, default=2)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    model = fit(json.loads(args.capture.read_text()), args.axis)
    j, damping = model['equivalent_inertia_kg_m2'], model['damping_nm_per_rps']
    model['linear_max_real_pole'] = {}
    for compensated in (False, True):
        worst = max(max(roots(j*scale, damping*dscale, compensated).real)
                    for scale in (.7, 1, 2, 10, 50) for dscale in (0, 1, 10))
        model['linear_max_real_pole'][str(compensated)] = float(worst)
    args.output.mkdir(parents=True, exist_ok=True)
    cases = []
    for scale, friction in ((.7, .002), (1, .002), (1, .01), (10, .15), (50, .15)):
        for compensated in (False, True):
            data = simulate(j*scale, damping, compensated, friction)
            name = f'J{scale}-friction{friction}-comp{int(compensated)}'
            np.save(args.output/(name+'.npy'), data)
            zero = data[data[:, 0] > 6]
            cases.append(dict(name=name, completed=bool(data[-1, 0] > 7.9),
                              peak_rps=float(max(abs(data[:, 2]))),
                              final_zero_rms_rps=float(np.sqrt(np.mean(zero[:, 2]**2))) if len(zero) else None,
                              final_position_span_turns=float(np.ptp(zero[:, 5])) if len(zero) else None))
    model['nonlinear_cases'] = cases
    model['limitations'] = 'One unstable mode identifies an equivalent plant, not unique physics. Ideal Hall spacing, no FOC phase error, no chassis/tyre coupling. Residual quantization cycles remain; floor validation required.'
    (args.output/'model.json').write_text(json.dumps(model, indent=2, allow_nan=False))
    print(json.dumps(model, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
