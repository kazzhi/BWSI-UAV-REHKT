import cv2 
import numpy as np
# corners of the square blocks (vertical and horizontal)
Ch_Dim = (7, 7)
Sq_size = 25  #milimeters
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 


obj_3D = np.zeros((Ch_Dim[0] * Ch_Dim[1], 3), np.float32)
index = 0
for i in range(Ch_Dim[0]):
    for j in range(Ch_Dim[1]):
        obj_3D[index][0] = i * Sq_size
        obj_3D[index][1] = j * Sq_size
        index += 1
#print(obj_3D)
obj_points_3D = []  # 3d point in real world space
img_points_2D = []  # 2d points in image plane.



import glob
image_files = glob.glob("imgs/*.png")

print(len(image_files))

for image in image_files:
    #print(image)
    
    img = cv2.imread(image)
    if img is None:
        print("No Image Found")
    image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

     
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, Ch_Dim, None)
    #print(f"Found corners in {image}") if ret else print(f"No corners found in {image}")

    if ret == True:
        obj_points_3D.append(obj_3D.copy())
        corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
        img_points_2D.append(corners2)

        img = cv2.drawChessboardCorners(image, Ch_Dim, corners2, ret)
#print(f"Images with successful detections: {len(obj_points_3D)}")
if len(obj_points_3D) == 0:
    #print("No corners were detected. Check your chessboard dimensions or image quality.")
    exit()

ret, mtx, dist_coeff, R_vecs, T_vecs = cv2.calibrateCamera(obj_points_3D, img_points_2D, gray.shape[::-1], None, None)
print("calibrated")

print(f"mtx: {mtx}")
print(f"coeff: {dist_coeff}")
print(f"ret: {ret}")
print()
# print(R_vecs)
# print(T_vecs)