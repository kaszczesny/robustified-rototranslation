from glob import glob
import numpy as np

pngs_ = glob("../../../rgbd_dataset_freiburg3_long_office_household/rgb/*.png")
pngs = []
for png in pngs_:
	pngs.append( float(png.split("/")[-1][:-4]) )

pngs = sorted(pngs)

csv = np.loadtxt("../../../rgbd_dataset_freiburg3_long_office_household/groundtruth.txt", delimiter=" ",
	skiprows=3)


correct = np.zeros([len(pngs), 7])

for i in range(0, len(pngs)):
	idx = np.where(csv[:, 0] > pngs[i])[0][0]
	correct[i,:] = csv[idx,1:]

np.savetxt("groundtruth_corrected.txt", correct, delimiter=",")
