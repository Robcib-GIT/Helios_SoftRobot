from numpy import cos, sin, pi, arctan2, sqrt
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt
from time import perf_counter

# This script tests a pretrained model to predict the output of a given dataset
model_file = 'models/neural_network/model_40_V3.h5'
test_data = 'dataset/240510/0x40_240510_3.csv'
normalization_file = 'models/normalization_params.json'

def normalize(data, min, max):
    return (data - min) / (max - min)

def denormalize(data, min, max):
    return data * (max - min) + min

# Load the trained network
model = tf.keras.models.load_model(model_file)

# Load data form csv file
data = pd.read_csv(test_data, header=None)

# 0x40
h0_range = [-0.2520, 0.1016]
h1_range = [0.1087, 0.4645]

# Split data into input and output:
# CSV file format: theta | phi | h0 | h1
qx = data.iloc[:, 0]
qy = data.iloc[:, 1]
qz = data.iloc[:, 2]
h0 = data.iloc[:, 3]
h1 = data.iloc[:, 4]

qy = qy*pi/180
qz = qz*pi/180
theta = sqrt(qy**2 + qz**2)
phi = arctan2(qz, qy)-pi/4

# Postprocess the input data, projecting the bending angle into x and y components.
# Thus, the discontinuity caused by the angle wrapping around 360 degrees is removed,
# smoothing the input data.
theta0 = theta*sin(phi)    
theta1 = theta*cos(phi)

# Normalize the data and concatenate
h0 = normalize(h0, h0_range[0], h0_range[1])
h1 = normalize(h1, h1_range[0], h1_range[1])

h0 = pd.DataFrame(h0)
h1 = pd.DataFrame(h1)
theta0 = pd.DataFrame(theta0)
theta1 = pd.DataFrame(theta1)

x = pd.concat([h0, h1], axis=1)

# Predict the output using the trained network
print("Predicting output...")
tic = perf_counter()
predicted_output = pd.DataFrame(model.predict(x))
toc = perf_counter()
print(f"Prediction took {toc - tic:0.4f} seconds")

print("Predicting single data...")
x_single = pd.DataFrame([[0.1, 0.2]])
tic = perf_counter()
aux = pd.DataFrame(model.predict(x_single, batch_size=1, verbose=0, use_multiprocessing=True, callbacks=None, max_queue_size=1, steps=None))
toc = perf_counter()
print(f"Prediction took {toc - tic:0.4f} seconds")

# Denormalize the predicted output
h0 = denormalize(h0, h0_range[0], h0_range[1])
h1 = denormalize(h1, h1_range[0], h1_range[1])
theta0_pred = denormalize(predicted_output.iloc[:, 0], -pi/6, pi/6)
theta1_pred = denormalize(predicted_output.iloc[:, 1], -pi/6, pi/6)

# Plot Input and Output
import matplotlib.pyplot as plt
fig = plt.figure()
sub1 = fig.add_subplot(121)
plt.plot(h0, theta0)
plt.xlabel('h0')
plt.ylabel('theta0')

sub1 = fig.add_subplot(122)
plt.plot(h1, theta1)
plt.xlabel('h1')
plt.ylabel('theta1')

# Plot the predicted output
fig = plt.figure()
sub1 = fig.add_subplot(121)
plt.plot(h0, theta0, 'ro')
plt.plot(h0, theta0_pred)
plt.xlabel('h0')
plt.ylabel('theta0')

sub1 = fig.add_subplot(122)
plt.plot(h1, theta1, 'ro')
plt.plot(h1, theta1_pred)
plt.xlabel('h1')
plt.ylabel('theta1')
plt.show()