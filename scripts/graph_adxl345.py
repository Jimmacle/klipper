#!/usr/bin/env python2
# Generate adxl345 accelerometer graphs
#
# Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse
import matplotlib

def parse_log(logname):
    f = open(logname, 'r')
    out = []
    for line in f:
        if line.startswith('#'):
            continue
        parts = line.split(',')
        if len(parts) != 4:
            continue
        out.append([float(p) for p in parts])
    return out

def plot_accel(data, logname):
    times = [d[0] for d in data]
    x = [d[1] for d in data]
    y = [d[2] for d in data]
    z = [d[3] for d in data]
    fig, ax1 = matplotlib.pyplot.subplots(nrows=1)
    ax1.set_title("Accelerometer data (%s)" % (logname,))
    ax1.set_ylabel('Accel (m/s^2)')
    ax1.plot(times, x, label='x', alpha=0.6)
    ax1.plot(times, y, label='y', alpha=0.6)
    ax1.plot(times, z, label='z', alpha=0.6)
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    ax1.legend(loc='best', prop=fontP)
    ax1.set_xlabel('Time (s)')
    ax1.grid(True)
    fig.tight_layout()
    return fig

def setup_matplotlib(output_to_file):
    global matplotlib
    if output_to_file:
        matplotlib.rcParams.update({'figure.autolayout': True})
        matplotlib.use('Agg')
    import matplotlib.pyplot, matplotlib.dates, matplotlib.font_manager
    import matplotlib.ticker

def main():
    # Parse command-line arguments
    usage = "%prog [options] <log>"
    opts = optparse.OptionParser(usage)
    opts.add_option("-o", "--output", type="string", dest="output",
                    default=None, help="filename of output graph")
    options, args = opts.parse_args()
    if len(args) != 1:
        opts.error("Incorrect number of arguments")

    # Parse data
    data = parse_log(args[0])

    # Draw graph
    setup_matplotlib(options.output is not None)
    fig = plot_accel(data, args[0])

    # Show graph
    if options.output is None:
        matplotlib.pyplot.show()
    else:
        fig.set_size_inches(6, 2.5)
        fig.savefig(options.output)

if __name__ == '__main__':
    main()
