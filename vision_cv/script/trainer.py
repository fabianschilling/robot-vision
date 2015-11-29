#!/usr/bin/env python

import numpy as np
import cv2
import matplotlib.pyplot as plt

from sklearn.ensemble import RandomForestClassifier
from sklearn import svm
from sklearn import cross_validation
from sklearn.externals import joblib

datapath = '/home/fabian/catkin_ws/src/ras_vision/vision_cv/script/data/'

def get_color_data():
	
	color_names = ['orange', 'yellow', 'lightgreen', 'green', 'lightblue', 'blue', 'purple', 'red']


	# Get training data
	orange = np.loadtxt(datapath + 'orange.txt')
	yellow = np.loadtxt(datapath + 'yellow.txt')
	lightgreen = np.loadtxt(datapath + 'lightgreen.txt')
	green = np.loadtxt(datapath + 'green.txt')
	lightblue = np.loadtxt(datapath + 'lightblue.txt')
	blue = np.loadtxt(datapath + 'blue.txt')
	purple = np.loadtxt(datapath + 'purple.txt')
	red = np.loadtxt(datapath + 'red.txt')

	# Get number of samples of every dataset
	oragnge_n_samples, _ = orange.shape
	yellow_n_samples, _ = yellow.shape
	lightgreen_n_samples, _ = lightgreen.shape
	green_n_samples, _ = green.shape
	lightblue_n_samples, _ = lightblue.shape
	blue_n_samples, _ = blue.shape
	purple_n_samples, _ = purple.shape
	red_n_samples, _ = red.shape


	# Generate targets
	orange_y = np.zeros(oragnge_n_samples)
	yellow_y = np.ones(yellow_n_samples)
	lightgreen_y = np.ones(lightgreen_n_samples) * 2
	green_y = np.ones(green_n_samples) * 3
	lightblue_y = np.ones(lightblue_n_samples) * 4
	blue_y = np.ones(blue_n_samples) * 5
	purple_y = np.ones(purple_n_samples) * 6
	red_y = np.ones(red_n_samples) * 7

	X = np.vstack((orange, yellow, lightgreen, green, lightblue, blue, purple, red))
	y = np.hstack((orange_y, yellow_y, lightgreen_y, green_y, lightblue_y, blue_y, purple_y, red_y))

	return (X, y)

def get_material_data():

	wood = np.loadtxt(datapath + 'wood.txt')
	plastic = np.loadtxt(datapath + 'plastic.txt')

	n_samples_wood, _ = wood.shape
	n_samples_plastic, _ = plastic.shape

	wood_y = np.zeros(n_samples_wood)
	plastic_y = np.ones(n_samples_plastic)

	X = np.vstack((wood, plastic))
	y = np.hstack((wood_y, plastic_y))

	return (X, y)


# X, y = get_color_data()
X, y = get_material_data()

clf = RandomForestClassifier(n_estimators=100)
#clf.fit(X, y)
scores = cross_validation.cross_val_score(clf, X, y)
print(scores)

#joblib.dump(clf, datapath + 'color_clf.pkl')