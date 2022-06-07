import cv2
import numpy as np

pntsCam = np.array([[1470.0,752],[246, 790],[243, 228],[1229, 152],[936,617]])
pntsTable = np.array([[0.4286,-0.1616],[0.3975, 0.3216],[0.6201, 0.3208],[0.6589, -0.0594],[ 0.4754, 0.0486]])
matrix, mask = cv2.findHomography(pntsCam, pntsTable)
print(matrix)
dst = cv2.perspectiveTransform(pntsCam.reshape(-1,1,2), matrix)
dst = dst.reshape((-1,2))
#print(dst.shape)
#print(pntsTable.shape)
#diff = np.mean(np.abs(pntsTable-dst),axis=0)
#print('********')
#print(diff)