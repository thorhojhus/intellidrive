import torch
import cv2 

device = "cuda" if torch.has_cuda else "cpu"
device = "mps" if torch.has_mps else "cpu"


model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True).to(device)

orb = cv2.ORB_create()

cap = cv2.VideoCapture(0)

def draw_yolo_predictions(frame, results):
    labels, cord = results.names, results.pred[0]
    for *xyxy, conf, cls in cord:
        label = f'{labels[int(cls)]} {conf:.2f}'
        x1, y1, x2, y2 = map(int, xyxy)  # Ensuring the coordinates are integers
        cv2.rectangle(frame, (x1, y1), (x2, y2), color=(255, 0, 0), thickness=2)
        cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX , 0.5, (255,255,255), 2, cv2.LINE_AA)
    return frame


while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        
        # Run YOLO inference
        results = model(frame)
        
        # Get keypoints using ORB
        kp, des = orb.detectAndCompute(frame, None)
        
        # Draw keypoints
        frame = cv2.drawKeypoints(frame, kp, None, color=(0,255,0), flags=0)
        
        # Draw YOLO predictions
        frame = draw_yolo_predictions(frame, results)
        
        # Display frame
        cv2.imshow('Frame', frame)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()