#!/usr/bin/env python

import os
import numpy as np
from scipy import misc
import cv2
from sklearn.preprocessing import scale
from sklearn.svm import SVC
from sklearn.utils import shuffle
from sklearn.cross_validation import train_test_split
from sklearn.externals import joblib

def load_images(directory):
	images = []
	count = 0
	for filename in os.listdir(directory):
		image = cv2.imread(directory + filename, cv2.CV_LOAD_IMAGE_GRAYSCALE)
		if image is not None:
			data = np.array(image)
			flattened = data.flatten()
			images.append(flattened)
			count += 1
	print('Read ' + str(count) + ' images from ' + directory)
	return (np.array(images, dtype=np.float32), count)

def main():
	cube_dir = '/home/fabian/Desktop/data/cube/'
	sphere_dir = '/home/fabian/Desktop/data/sphere/'

	cubes, n_cubes = load_images(cube_dir)
	spheres, n_speres = load_images(sphere_dir)

	cubes_labels = np.zeros(n_cubes)
	spheres_labels = np.ones(n_speres)

	X_raw = np.concatenate((cubes, spheres))
	X = scale(X_raw)
	y = np.concatenate((cubes_labels, spheres_labels))

	X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.1)

	print(cubes.shape)

	clf = SVC(probability=True)
	clf.fit(X, y)

	print(clf.score(X_test, y_test))

	joblib.dump(clf, '/home/fabian/Desktop/svm/svm.pkl')


if __name__ == '__main__':
	main()