import numpy as np
import sys
import cv2
# You should replace these 3 lines with the output in calibration step
DIM=(320, 240)
K=np.array([[157.48126107325643, 0.0, 155.54100584796126], [0.0, 157.4460985081828, 124.1673880384651], [0.0, 0.0, 1.0]])
D=np.array([[-0.016100807266528787], [-0.0660261841490366], [0.12071322102537432], [-0.07516938046185537]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w,h), 1, (w,h))
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)