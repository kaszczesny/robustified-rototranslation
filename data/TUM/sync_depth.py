from glob import glob
import numpy as np

pngs_ = glob("../../../rgbd_dataset_freiburg3_long_office_household/rgb/*.png")
pngs = []
for png in pngs_:
	pngs.append( float(png.split("/")[-1][:-4]) )

pngs = np.array(sorted(pngs))


depths_ = glob("../../../rgbd_dataset_freiburg3_long_office_household/depth/*.png")
depths = []
for depth in depths_:
	depths.append( float(depth.split("/")[-1][:-4]) )

depths = np.array(sorted(depths))



correct = np.zeros([len(pngs)])

for i in range(0, len(pngs)):
	idx = np.where(depths > pngs[i])[0][0]
	correct[i] = idx + 1

np.savetxt("depth_idx.txt", correct, "%d", delimiter=",")
