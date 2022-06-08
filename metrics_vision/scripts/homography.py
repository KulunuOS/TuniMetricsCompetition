import cv2
import numpy as np

pntsCam = np.array([[1516.0,746],[301, 765],[315, 205],[1288, 151],[991,613]])
pntsTable = np.array([[0.4381,-0.1896],[0.4186, 0.2945],[0.6411, 0.2905],[0.6706, -0.0920],[0.4907, 0.0192]])
matrix, mask = cv2.findHomography(pntsCam, pntsTable)
print(matrix)
dst = cv2.perspectiveTransform(pntsCam.reshape(-1,1,2), matrix)
dst = dst.reshape((-1,2))
print(dst.shape)
print(pntsTable.shape)
diff = np.mean(np.abs(pntsTable-dst),axis=0)
print('********')
print(diff)