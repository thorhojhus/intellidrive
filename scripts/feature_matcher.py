import cv2
import numpy as np

def feature_matching(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Cannot open video.")
        return
    
    orb = cv2.ORB_create()
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    ret, prev_frame = cap.read()
    if not ret:
        print("Error: Cannot read frame.")
        return
    
    # Convert to grayscale
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect and compute features
        kp1, des1 = orb.detectAndCompute(prev_gray, None)
        kp2, des2 = orb.detectAndCompute(gray, None)
        
        # Match features using KNN
        matches = bf.match(des1, des2)
        
        # Sort matches based on distance
        matches = sorted(matches, key = lambda x:x.distance)
        
        # Draw matches
        matched_frame = cv2.drawMatches(prev_frame, kp1, frame, kp2, matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        
        # Display the matched features
        cv2.imshow('Matched Features', matched_frame)
        
        # Wait for key press or for 30ms, whichever is faster
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
        
        prev_gray = gray.copy()
        prev_frame = frame.copy()
    
    cap.release()
    cv2.destroyAllWindows()

# Call the function with the path to your video file
#feature_matching('Tunnel.mp4')
feature_matching(0)
