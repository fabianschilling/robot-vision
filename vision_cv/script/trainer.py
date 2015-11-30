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

	color_data = []
	color_targets = []

	for i, col in enumerate(color_names):
		data = np.loadtxt(datapath + color_names[i] + '.txt')
		color_data.append(data)
		n_samples, _ = data.shape
		color_targets.append(np.zeros(n_samples) + i)

	X = np.vstack(tuple(color_data))
	y = np.hstack(tuple(color_targets))

	return (X, y)

def get_material_data():

	material_names = ['wood', 'plastic']
	material_data = []
	material_targets = []

	for i, mat in enumerate(material_names):
		data = np.loadtxt(datapath + material_names[i] + '.txt')
		material_data.append(data)
		n_samples, _ = data.shape
		material_targets.append(np.zeros(n_samples) + i)

	X = np.vstack(tuple(material_data))
	y = np.hstack(tuple(material_targets))

	return (X, y)

def get_shape_data():

	shape_names = ['ball', 'cube']
	shape_data = []
	shape_targets = []

	for i, sh in enumerate(shape_names):
		data = np.loadtxt(datapath + shape_names[i] + '.txt')
		shape_data.append(data)
		n_samples, _ = data.shape
		shape_targets.append(np.zeros(n_samples) + i)

	X = np.vstack(tuple(shape_data))
	y = np.hstack(tuple(shape_targets))

	return (X, y)


# X, y = get_color_data()
# X, y = get_material_data()
X, y = get_shape_data()

clf = RandomForestClassifier(n_estimators=100)
clf.fit(X, y)
# scores = cross_validation.cross_val_score(clf, X, y)
# print(scores)

joblib.dump(clf, datapath + 'shape_clf.pkl')