import cv2
import numpy as np

pntsCam = np.array([[1429.0,  441.0],
					[1090.0, 607.0],
					[920.0, 360.0],
					[602.0, 326.0],
					[1148.0, 403.0], 
					[549.0, 559.0 ],
					[802.0, 548.0 ],
					[754.0, 256.0 ]])

pntsTable = np.array([[0.5561, 0.3529 ],
					  [0.4875, -0.0136 ],
					  [0.5819, 0.0535 ],
					  [0.5924, 0.1751 ],
					  [0.5740, -0.0415],
					  [0.5083, 0.1907],
					  [0.5159, 0.9196],
					  [0.6290, 0.1107]])

matrix, mask = cv2.findHomography(pntsCam, pntsTable)
print(matrix)
dst = cv2.perspectiveTransform(pntsCam.reshape(-1,1,2), matrix)
dst = dst.reshape((-1,2))
print(dst.shape)
print(pntsTable.shape)
diff = np.mean(np.abs(pntsTable-dst),axis=0)
print('********')
print(diff)




