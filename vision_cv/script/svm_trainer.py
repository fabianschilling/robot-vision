#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from sklearn import svm
from sklearn.cross_validation import train_test_split
from sklearn.externals import joblib
from sklearn.ensemble import RandomForestClassifier

def main():

	greendata = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/svm/green.txt')
	reddata = np.loadtxt('/home/fabian/catkin_ws/src/ras_vision/vision_data/color/svm/red.txt')

	n_samples, n_features = greendata.shape

	X = np.vstack((greendata, reddata))

	greentargets = np.zeros(n_samples)
	redtargets = np.ones(n_samples)

	y = np.hstack((greentargets, redtargets))

	#X_train, X_test, y_train, y_test = train_test_split(X, y)

	#clf = svm.SVC(probability=True)
	clf = RandomForestClassifier()

	clf.fit(X, y)

	joblib.dump(clf, 'redvsgrn.pkl')


if __name__ == '__main__':
	main()