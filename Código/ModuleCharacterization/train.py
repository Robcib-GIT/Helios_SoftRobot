import numpy as np
from numpy import cos, sin, pi
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Dense
import pandas as pd

# This script trains a new model with a given dataset
model_file = 'models/neural_network/model_4A_V3.h5'
test_data = 'dataset/240510/0x4A_240510_3.csv'
normalization_file = 'models/normalization_params.json'

def normalize(data, min, max):
    return (data - min) / (max - min)

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

# 0x40
h0_range = [-0.1576, 0.0671]
h1_range = [0.1167, 0.3308]

# Load data form csv f6ile
data = pd.read_csv(test_data, header=None)

# Split data into input and output:
# CSV file format: theta | phi | h0 | h1
qx = data.iloc[:, 0]
qy = data.iloc[:, 1]
qz = data.iloc[:, 2]
h0 = data.iloc[:, 3]
h1 = data.iloc[:, 4]

qy = qy*np.pi/180
qz = qz*np.pi/180
theta = np.sqrt(qy**2 + qz**2)
phi = np.arctan2(qz, qy)-np.pi/4

# Postprocess the input data, projecting the bending angle into x and y components.
# Thus, the discontinuity caused by the angle wrapping around 360 degrees is removed,
# smoothing the input data.
theta0 = theta*sin(phi)    
theta1 = theta*cos(phi)

# Normalize the data and concatenate
h0 = normalize(h0, h0_range[0], h0_range[1])
h1 = normalize(h1, h1_range[0], h1_range[1])
theta0 = normalize(theta0, -np.pi/6, np.pi/6)
theta1 = normalize(theta1, -np.pi/6, np.pi/6)

h0 = pd.DataFrame(h0)
h1 = pd.DataFrame(h1)
theta0 = pd.DataFrame(theta0)
theta1 = pd.DataFrame(theta1)

# Plot Input and Output
import matplotlib.pyplot as plt
fig = plt.figure()
sub = fig.add_subplot(121)
plt.plot(h0, theta0, 'o')
plt.xlabel('h0')
plt.ylabel('theta0')

sub = fig.add_subplot(122)
plt.plot(h1, theta1, 'o')
plt.xlabel('h1')
plt.ylabel('theta1')
plt.show()

x = pd.concat([h0, h1], axis=1)
y = pd.concat([theta0, theta1], axis=1)

# Create a model
model = Sequential()
model.add(Dense(8, input_dim=2, activation='relu'))
model.add(Dense(2, activation='tanh'))

# Compile the model
model.compile(loss='mean_squared_error', optimizer='adam')

# Train the model
model.fit(x, y, epochs=20, batch_size=10)

# Model summary
model.summary()

# Save the model
model.save(model_file)