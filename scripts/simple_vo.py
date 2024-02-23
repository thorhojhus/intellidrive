import cv2
import numpy as np

def process_frame(prev_frame, current_frame, prev_kp, prev_des):
    orb = cv2.ORB_create()

    kp2, des2 = orb.detectAndCompute(current_frame, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(prev_des, des2, k=2)

    good = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
            
    src_pts = np.float32([ prev_kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    matchesMask = mask.ravel().tolist()
    
    filtered_matches = [good[i] for i in range(len(matchesMask)) if matchesMask[i]]

    return filtered_matches, kp2, des2

cap = cv2.VideoCapture('Tunnel.mp4')

ret, prev_frame = cap.read()
orb = cv2.ORB_create()
prev_kp, prev_des = orb.detectAndCompute(prev_frame, None)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    matches, prev_kp, prev_des = process_frame(prev_frame, frame, prev_kp, prev_des)
    prev_frame = frame

    # Display results if necessary
    out_img = cv2.drawMatches(prev_frame, prev_kp, frame, prev_kp, matches, None)
    cv2.imshow('Matches', out_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()