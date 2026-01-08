# Calibration Guide (Pixel → Ground XY)

1. Place 4+ markers on a flat ground plane and measure their (X,Y) in meters.
2. Click corresponding (u,v) pixels in the camera image.
3. Compute homography H (image→ground) and save it:
```python
import numpy as np, cv2
img_pts = np.array([[u1,v1],[u2,v2],[u3,v3],[u4,v4]], dtype=np.float32)
gnd_pts = np.array([[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]], dtype=np.float32)
H,_ = cv2.findHomography(img_pts, gnd_pts, cv2.RANSAC)
np.save("calibration/H_img_to_ground.npy", H)
