from robutils import *
from random import uniform

ax = initPlot(max_range=5, width=8, height=8)

# Define shoulder positions
rightShoulder = np.matrix('[1.22; 1.39; 5.48]')
leftShoulder = np.matrix('[-1.22; 1.34; 5.5]')
# Define vector from one shoulder to another
vectorShoulderToShoulder = rightShoulder - leftShoulder
# Define midpoint between shoulders
centerShoulder = leftShoulder + vectorShoulderToShoulder / 2
# Define hip point
centerHip = np.matrix('[0;-0.8; 5.52]')
# Define hand point
rightHand = np.matrix('[0; 1.8; 3.0]')

# Define vectors
rightShoulderVector = rightShoulder - centerShoulder
leftShoulderVecotor = leftShoulder - centerShoulder
hipVector = centerHip - centerShoulder
handVector = rightHand - rightShoulder

# Define coordinate frame relative to rightShoulder
xAxis = vectorShoulderToShoulder
xAxis /= np.linalg.norm(xAxis)
yAxis = (centerShoulder - centerHip)
yAxis /= np.linalg.norm(yAxis)
zAxis = np.cross(rightShoulderVector, hipVector, axis=0).reshape(3, 1)
zAxis /= np.linalg.norm(zAxis)

# Calculate projections over new axis
xProj = np.matmul(handVector.T, xAxis).item(0) # a.T * b = a inner b
yProj = np.matmul(handVector.T, yAxis).item(0)
zProj = np.matmul(handVector.T, zAxis).item(0)
xProjVector = xProj * xAxis
yProjVector = yProj * yAxis
zProjVector = zProj * zAxis

# Visualize body vectors
plotLine(ax, rightShoulder, centerShoulder, 'yellow')
plotLine(ax, leftShoulder, centerShoulder, 'pink', 'leftShoulder')
plotLine(ax, centerHip, centerShoulder, 'cyan', 'centerHip')
plotLine(ax, rightHand, rightShoulder, 'magenta', 'rightHand')
# Visualize new coordinate frames over rightShoulder
plotLine(ax, xAxis + rightShoulder, rightShoulder,
    color='red', linestyle='--')
plotLine(ax, yAxis + rightShoulder, rightShoulder,
    color='green', linestyle='--')
plotLine(ax, zAxis + rightShoulder, rightShoulder,
    color='blue', linestyle='--')
# Visualize projections over new coordinate frame
plotLine(ax, xProjVector + rightShoulder, rightShoulder, 'red', 'xProj')
plotLine(ax, yProjVector + rightShoulder, rightShoulder, 'green', 'yProj')
plotLine(ax, zProjVector + rightShoulder, rightShoulder, 'blue', 'zProj')

# Show everything
plt.show()
