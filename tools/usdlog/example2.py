# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re
import numpy as np

# decode binary log data
logData = cff.decode("/media/whoenig/80FB-9983/log17")
# print(logData)
# print(logData['tick'])
# plt.figure(0)
# plt.hist(np.diff(logData['tick']))
# plt.show()

# current plot for simple subplot usage
# plotCurrent = 0;

# new figure
plt.figure(0)


# plotCurrent += 1
# plt.subplot(plotRows, plotCols, plotCurrent)
plt.plot(logData['tick'], logData['gyro.x'], '-', label='X')
plt.plot(logData['tick'], logData['gyro.y'], '-', label='Y')
plt.plot(logData['tick'], logData['gyro.z'], '-', label='Z')
# plt.plot(logData['tick'], logData['ctrlOutput.thrust'], '-', label='Z')
plt.xlabel('RTOS Ticks')
plt.ylabel('Position [m]')
plt.legend(loc=9, ncol=3, borderaxespad=0.)
 

plt.show()