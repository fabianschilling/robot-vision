import matplotlib.pyplot as plt
import numpy as np
from sklearn import svm
from sklearn import cross_validation
from sklearn.ensemble import RandomForestClassifier

wood = np.loadtxt('wood.txt')
plas = np.loadtxt('plastic.txt')

n_samples, n_features = wood.shape

wood_targets = np.zeros(n_samples)
plas_targets = np.ones(n_samples)

X = np.vstack((wood, plas))
y = np.hstack((wood_targets, plas_targets))

#clf = svm.SVC(probability=True)
clf = RandomForestClassifier(n_estimators=100, verbose=1)

scores = cross_validation.cross_val_score(clf, X, y, cv=5)

print(scores)

# plt.scatter(wood[:, 0], wood[:, 1], color='green')
# plt.scatter(plas[:, 0], plas[:, 1], color='blue')
# plt.show()