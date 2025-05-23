import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pathlib import Path
from keras import models as km
from utils import get_data, denormalize, parametric_arc
from helios_kine import iKine, fKine

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 25

def plot_data(l, h, l_avg, h_avg):
    # Plot the TOF distances
    plt.figure(figsize=(10, 6))
    plt.plot(l[0], label='TOF distance l0')
    plt.plot(l[1], label='TOF distance l1')
    plt.plot(l[2], label='TOF distance l2')
    plt.plot(l[3], label='TOF distance l3')
    
    plt.xlabel('Sample Index')
    plt.ylabel('TOF Distance')
    plt.title('Evolution of TOF Distances')
    plt.legend()
    plt.grid(True)

    # Plot the Helios measurements
    plt.figure(figsize=(10, 6))
    plt.plot(h[0], label='Helios measurement 0')
    plt.plot(h[1], label='Helios measurement 1')
    plt.plot(h[2], label='Helios measurement 2')
    plt.plot(h[3], label='Helios measurement 3')

    plt.xlabel('Sample Index')
    plt.ylabel('Helios Measurement')
    plt.title('Evolution of Helios Measurements')
    plt.legend()
    plt.grid(True)

    # Calculate the differential measurements
    diff_l2_l0 = l[2] - l[0]
    diff_l3_l1 = l[3] - l[1]

    # Plot the differential measurements
    plt.figure(figsize=(10, 6))
    plt.plot(diff_l2_l0, label='Differential l2 - l0')
    plt.plot(diff_l3_l1, label='Differential l3 - l1')

    plt.xlabel('Sample Index')
    plt.ylabel('Differential TOF Distance')
    plt.title('Differential TOF Distances')
    plt.legend()
    plt.grid(True)

    # Calculate the differential measurements for Helios
    diff_h2_h0 = h[2] - h[0]
    diff_h3_h1 = h[3] - h[1]

    # Plot the differential measurements for Helios
    plt.figure(figsize=(10, 6))
    plt.plot(diff_h2_h0, label='Differential h2 - h0')
    plt.plot(diff_h3_h1, label='Differential h3 - h1')
    
    plt.xlabel('Sample Index')
    plt.ylabel('Differential Helios Measurement')
    plt.title('Differential Helios Measurements')
    plt.legend()
    plt.grid(True)

if __name__ == '__main__':
    # File path
    file_path = Path('./dataset/250109/0x4A_250109_train.csv')

    # Plot the data
    h, l, h_avg, l_avg, h0, l0 = get_data(file_path)
    #plot_data(l, h, l_avg, h_avg)

    # Load a model and predict the data
    model = km.load_model('models/nn/nn_0x4A_V.keras')

    # Prepare the test data
    x = np.array(h).T
    y = np.array(l).T

    # Predict the output
    predicted_output = pd.DataFrame(model(x))
    predicted_output = np.array([predicted_output[i].tolist() for i in range(len(predicted_output.columns))])

    # Denormalize the output and the test data
    expected_output = denormalize(l, l0, l_avg)
    predicted_output = denormalize(predicted_output, l0, l_avg)

    '''
    # Plot the predicted output vs the expected output
    plt.figure(figsize=(10, 6))

    # Plot the expected output
    plt.subplot(3, 1, 1)
    plt.plot(expected_output[0], label='Expected TOF distance l0')
    plt.plot(expected_output[1], label='Expected TOF distance l1')
    plt.plot(expected_output[2], label='Expected TOF distance l2')
    plt.plot(expected_output[3], label='Expected TOF distance l3')
    plt.ylim(0, 140)
    plt.xlabel('Sample Index')
    plt.ylabel('TOF Distance')
    plt.title('Expected TOF Distances')
    plt.legend()
    plt.grid(True)

    # Plot the predicted output
    plt.subplot(3, 1, 2)
    plt.plot(predicted_output[0], label='Predicted TOF distance l0')
    plt.plot(predicted_output[1], label='Predicted TOF distance l1')
    plt.plot(predicted_output[2], label='Predicted TOF distance l2')
    plt.plot(predicted_output[3], label='Predicted TOF distance l3')
    plt.ylim(0, 140)
    plt.xlabel('Sample Index')
    plt.ylabel('TOF Distance')
    plt.title('Predicted TOF Distances')
    plt.legend()
    plt.grid(True)

    # Plot the error between the expected and the predicted output
    plt.subplot(3, 1, 3)

    for j in range(len(expected_output)):
        e = [abs(expected_output[j][i] - predicted_output[j][i]) for i in range(len(expected_output[0]))]
        plt.plot(e, label='Error l' + str(j))
    
    plt.ylim(0, 140)
    plt.xlabel('Sample Index')
    plt.ylabel('TOF Distance')
    plt.title('Error')
    plt.legend()
    plt.grid(True)
    '''
    
    e = np.zeros((len(expected_output[0]), 1))
    
    theta_expected, phi_expected, length_expected = fKine(np.transpose(expected_output), 100)
    theta_predicted, phi_predicted, length_predicted = fKine(np.transpose(predicted_output), 100)

    # For plotting the 3D points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xxe = []
    yye = []
    zze = []

    xxp = []
    yyp = []
    zzp = []

    e = np.zeros((len(expected_output[0]), 1))

    for i in range(len(expected_output[0])):
        xe, ye, ze = parametric_arc(length_expected[i], theta_expected[i], phi_expected[i])
        xp, yp, zp = parametric_arc(length_predicted[i], theta_predicted[i], phi_predicted[i])
        # ax.plot(xp, yp, zp, 'k', alpha=0)

        xxe.append(xe[-1])
        yye.append(ye[-1])
        zze.append(ze[-1])

        xxp.append(xp[-1])
        yyp.append(yp[-1])
        zzp.append(zp[-1])

        e[i] = np.sqrt((xe[-1] - xp[-1])**2 + (ye[-1] - yp[-1])**2 + (ze[-1] - zp[-1])**2)

    # Plot the 3D points
    # Color points based on error magnitude
    for i in range(len(expected_output[0])):
        color = plt.cm.RdYlGn(1 - e[i] / np.max(e))  # Red for max error, green for min error
        ax.scatter(xxp[i], yyp[i], zzp[i], color=color, s=20)

    ax.plot(xxe, yye, zze, 'b', alpha=0.5)

    ax.set_xlabel('x (mm)', labelpad=10)
    ax.set_ylabel('y (mm)', labelpad=10)
    ax.set_zlabel('z (mm)', labelpad=10)
    #ax.set_title('3D Plot of Points with Error Coloring')
    ax.set_zlim(30, 60)
    ax.set_box_aspect([1, 1, 1/4])  # Aspect ratio is 1:1:1 (equal)

    print('Mean Error:', np.mean(e))
    
    plt.figure(figsize=(10, 6))
    plt.plot(e, label='Error')
    plt.plot([np.mean(e)] * len(e), label='Mean Error', linestyle='--', color='red')
    plt.xlabel('Sample Index')
    plt.ylabel('Error')
    plt.title('Tip Error')
    plt.grid(True)
    # Add a color bar to the 3D plot
    mappable = plt.cm.ScalarMappable(cmap=plt.cm.RdYlGn.reversed())
    mappable.set_array([min(e), max(e)])
    cbar = plt.colorbar(mappable, ax=ax, shrink=0.8, aspect=10)
    cbar.set_label('Error (mm)')

    plt.show()