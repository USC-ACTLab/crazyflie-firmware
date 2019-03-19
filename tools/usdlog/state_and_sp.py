# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file', help='input file')
args = parser.parse_args()

# decode binary log data
logData = cff.decode(args.file)

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1;
plotRows = 0;

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k


# get plot config from user
plotPos = 0
if re.search('.x', keys):
    inStr = input("plot pos data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotPos = 1
        plotRows += 3

plotVel = 0
if re.search('.vx', keys):
    inStr = input("plot pos data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotVel = 1
        plotRows += 3

plotAcc = 0
if re.search('.ax', keys):
    inStr = input("plot pos data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotAcc = 1
        plotRows += 3


# current plot for simple subplot usage
plotCurrent = 0;

# new figure
plt.figure(0)

if plotPos:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.x'], '-', label='X (state)')
    plt.plot(logData['tick'], logData['spCompressed.x'], '-', label='X (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('X [mm]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.y'], '-', label='Y (state)')
    plt.plot(logData['tick'], logData['spCompressed.y'], '-', label='Y (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Y [mm]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.z'], '-', label='Z (state)')
    plt.plot(logData['tick'], logData['spCompressed.z'], '-', label='Z (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Z [mm]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

# velocity
if plotVel:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.vx'], '-', label='VX (state)')
    plt.plot(logData['tick'], logData['spCompressed.vx'], '-', label='VX (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('X [mm/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.vy'], '-', label='VY (state)')
    plt.plot(logData['tick'], logData['spCompressed.vy'], '-', label='VY (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Y [mm/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.vz'], '-', label='VZ (state)')
    plt.plot(logData['tick'], logData['spCompressed.vz'], '-', label='VZ (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Z [mm/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

# acc
if plotAcc:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.ax'], '-', label='AX (state)')
    plt.plot(logData['tick'], logData['spCompressed.ax'], '-', label='AX (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('X [mm/s^2]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.ay'], '-', label='AY (state)')
    plt.plot(logData['tick'], logData['spCompressed.ay'], '-', label='AY (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Y [mm/s^2]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stateCompressed.az'], '-', label='AZ (state)')
    plt.plot(logData['tick'], logData['spCompressed.az'], '-', label='AZ (sp)')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Z [mm/s^2]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

plt.show()