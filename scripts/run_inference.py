#!/usr/bin/env python

import os
import copy
import time
import torch
import rospy
import rospkg
import numpy as np
import torch.nn as nn
import torchbnn as bnn

from matplotlib import pyplot as plt

#ROS
from robust_fast_navigation.msg import SolverStateArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

states = None


MAX_POLYS = 4
MAX_PLANES = 20

# Spatial Embedding Module:
class SpatialEmbedding(nn.Module):
    def __init__(self, spatial_channels, output_dim, kernel_size, dropout_prob):
        super(SpatialEmbedding, self).__init__()
        self.spatial_channels = spatial_channels
        self.kernel_size = kernel_size

        # Define CNN layers for spatial embedding
        self.cnn_layer = nn.Sequential(
            nn.Conv2d(in_channels=4, out_channels=self.spatial_channels, kernel_size=self.kernel_size),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=(2,2))
        )

        # Get shape of output from CNN layer
        self.dummy_input = torch.randn(1, 4, MAX_POLYS, MAX_PLANES)
        self.dummy_output = self.cnn_layer(self.dummy_input)
        self.fc_in_features = self.dummy_output.size(1)*self.dummy_output.size(2)*self.dummy_output.size(3)

        self.fc1 = nn.Linear(in_features=self.fc_in_features, out_features=output_dim)

    def forward(self, x):
        # x has shape batch_size, MAX_POLYS, MAX_PLANES, 4
        x = self.cnn_layer(x)
        # x has shape batch_size, spatial_channels, 1, 1
        x = x.reshape(x.size(0),-1)
        # x has shape batch_size, spatial_channels
        x = self.fc1(x)
        return x

# Define the Bayesian Neural Network model using torchbnn
class BNNModel(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, dropout_prob, spatial_channels=None, kernel_size=None):
        super(BNNModel, self).__init__()
        self.input_dim = input_dim if spatial_channels is None else 38
        self.hidden_dim = hidden_dim
        self.dropout1 = nn.Dropout(p=dropout_prob)
        self.output_dim = output_dim

        # Define the Bayesian layers using torchbnn
        self.fc1 = bnn.BayesLinear(prior_mu=0, prior_sigma=0.1, in_features=self.input_dim, out_features=self.hidden_dim)
        self.activation1 = nn.ReLU()
        self.fc2 = bnn.BayesLinear(prior_mu=0, prior_sigma=0.1, in_features=self.hidden_dim, out_features=output_dim)

        # Spatial Embedding Module
        self.spatial_embedding = None
        if spatial_channels is not None and kernel_size is not None:
            self.spatial_embedding = SpatialEmbedding(spatial_channels=spatial_channels, output_dim=32, kernel_size=kernel_size, dropout_prob=dropout_prob)

    # Function to initialize random weights for the Bayesian layers
    def init_weights(self):
        for m in self.modules():
            if isinstance(m, bnn.BayesLinear):
                m.weight_mu.data.normal_(0, 0.1)
                m.bias_mu.data.normal_(0, 0.1)

    def forward(self, x):

        if self.spatial_embedding != None:
            x_polys = x[:,:320].reshape(x.shape[0], MAX_POLYS, MAX_PLANES, 4)
            x_polys = x_polys.permute(0, 3, 1, 2)
            x_poly_embed = self.spatial_embedding(x_polys)

            x_other = x[:,320:]
            x = torch.cat((x_poly_embed, x_other), dim=1)

        x = self.fc1(x)
        x = self.dropout1(x)
        x = self.activation1(x)
        x = self.fc2(x)
        return x

# Callback 
def solverStatecb(data):
    global states
    states = data.states
    
    rospy.loginfo("Received solver states")

def extract_polygons_from_pose_array(pose_array_msg):
    polygons = []
    current_polygon = []

    for pose in pose_array_msg.poses:
        if pose.orientation.x == 0.0 and pose.orientation.y == 0.0 and pose.orientation.z == 0.0 and pose.orientation.w == 0.0:
            # Delimiter detected, add the current_polygon to polygons list
            if current_polygon:
                polygons.append(copy.copy(np.array(current_polygon)))
                current_polygon = []
        else:
            # Extract the hyperplane equation coefficients from the orientation part of the pose
            a = pose.orientation.x
            b = pose.orientation.y
            c = pose.orientation.z
            d = pose.orientation.w

            current_polygon.append(np.array([a, b, c, d]))

    return polygons

def states_to_tensor():
    global states

    # convert states to tensor
    list_of_tensors = []
    for i, state in enumerate(states):
        polygons = extract_polygons_from_pose_array(state.polys)

        padded_polygons = torch.zeros((MAX_POLYS, MAX_PLANES, 4), dtype=torch.float32)
        for i, polygon in enumerate(polygons):
            padded_polygons[i, :len(polygon)] = torch.tensor(polygon, dtype=torch.float32)

        initial_pva = torch.tensor(state.initialPVA.velocities[:2] + state.initialPVA.accelerations[:2], dtype=torch.float32)
        final_pva = torch.tensor(state.finalPVA.positions[:2], dtype=torch.float32)

        input_feature = padded_polygons.flatten()
        input_feature = torch.cat((input_feature, initial_pva, final_pva))

        input_feature = input_feature.unsqueeze(0)
        list_of_tensors.append(input_feature)

    return torch.cat(list_of_tensors, dim=0)

def main():
    global states

    # Get dataset
    rospack = rospkg.RosPack()

    model_path = rospack.get_path('robust_fast_navigation')
    model_path = os.path.join(model_path, 'bags', 'learning_solver', 'train_val')


    input_dim = 4*20*4 + 4 + 2
    hidden_dim = 64  # Set the number of hidden units in the BNN
    output_dim = 1  # For binary classification
    dropout_prob = .6
    spatial_channels = 16
    kernel_size = (3,3)

    model = BNNModel(input_dim, hidden_dim, output_dim, dropout_prob, spatial_channels, kernel_size)

    # load model to cpu
    device = torch.device('cpu')
    model.load_state_dict(torch.load(os.path.join(model_path, 'model.pt'), map_location=device))

    # Run inference on validation and measure inference time
    model.to(device)
    model.eval()

    # initialize ros node
    rospy.init_node('run_inference', anonymous=True)
    rate = rospy.Rate(2) # 1hz

    # Subscriber for SolverState message
    rospy.Subscriber("/candidateRecoveryPoints", SolverStateArray, solverStatecb)

    # initialize publisher for Float32MultiArray
    pub = rospy.Publisher('/inference', Float32MultiArray, queue_size=10, latch=True)

    while not rospy.is_shutdown():
        rate.sleep()

        if states == None:
            continue

        rospy.loginfo("states received!")

        # convert states to tensor
        input_tensor = states_to_tensor()

        # Run inference
        start = time.time()
        output = torch.round(torch.sigmoid(model(input_tensor)))
        end = time.time()
        print('Inference time: ', end - start)

        # get output and package into Float32MultiArray message
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "predictions"
        dim.size = output.shape[0]
        dim.stride = 1

        msg.layout.dim.append(dim)
        msg.data = output.detach().numpy().flatten()

        # publish message
        pub.publish(msg)

        states = None


if __name__ == "__main__":
    main()
