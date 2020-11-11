import matplotlib.pyplot as plt
import numpy as np
D = 5.0

for i in np.arange(0.0, 5.0, 0.01):
    a = 0.5*float((20.0/(D**7))*(i**7) - (70.0/(D**6))*(i**6) + (84.0/(D**5))*(i**5) - (35.0/(D**4))*(i**4)+1)
    plt.plot(i, a, 'x')
for i in np.arange(0.0, 5.0, 0.01):
    b = -.5*float((20.0/(D**7))*(i**7) - (70.0/(D**6))*(i**6) + (84.0/(D**5))*(i**5) - (35.0/(D**4))*(i**4)+1)
    plt.plot(i, b, 'x')
plt.show()
MAX_DIST = 5.0
dist_24 = 7.0

data = long((20.0/(MAX_DIST**7))*(dist_24**7) - (70.0/(MAX_DIST**6))*(dist_24**6) + (84.0/(MAX_DIST**5))*(dist_24**5) - (35.0/(MAX_DIST**4))*(dist_24**4) +1)
print(data)
