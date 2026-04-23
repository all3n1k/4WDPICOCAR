import cv2
import requests
import base64
import time
import json
import threading

# Configuration
STREAM_URL = 'http://192.168.1.216:81/stream'
OLLAMA_URL = 'http://localhost:11434/api/generate'
MODEL = 'moondream'

# Global state
latest_frame = None
vlm_status = "Initializing..."
robot_action = "STOP"

def vlm_worker():
    global latest_frame, vlm_status, robot_action
    
    while True:
        if latest_frame is not None:
            # Encode frame to jpeg and then to base64
            _, buffer = cv2.imencode('.jpg', latest_frame)
            img_b64 = base64.b64encode(buffer).decode('utf-8')
            
            prompt = """
Analyze this image for a robot pet. 
1. Is there a cat, dog, or person in the image?
2. If yes, are they on the LEFT side, RIGHT side, or in the CENTER of the image?
Respond ONLY with a JSON object in this exact format:
{"target_detected": true/false, "location": "left" or "right" or "center" or "none", "reasoning": "brief explanation"}
"""
            
            payload = {
                "model": MODEL,
                "prompt": prompt,
                "images": [img_b64],
                "stream": False,
                "format": "json"
            }
            
            start_time = time.time()
            vlm_status = "Inferring..."
            try:
                response = requests.post(OLLAMA_URL, json=payload, timeout=5)
                if response.status_code == 200:
                    data = response.json()
                    result = json.loads(data.get("response", "{}"))
                    
                    latency = time.time() - start_time
                    vlm_status = f"VLM OK ({latency:.2f}s): {result.get('reasoning', '')}"
                    
                    if result.get("target_detected"):
                        loc = result.get("location", "none").lower()
                        if loc == "left":
                            robot_action = "TURN LEFT"
                        elif loc == "right":
                            robot_action = "TURN RIGHT"
                        elif loc == "center":
                            robot_action = "FORWARD (CHASE)"
                        else:
                            robot_action = "STOP (CONFUSED)"
                    else:
                        robot_action = "SEARCHING..."
                        
                else:
                    vlm_status = f"VLM Error: {response.status_code}"
            except Exception as e:
                vlm_status = f"API Error: {str(e)}"
                
        time.sleep(0.1) # Small pause before next inference

def main():
    global latest_frame, vlm_status, robot_action
    
    print("Agentic Pet OS - VLM Node Initializing...")
    
    # Start VLM thread
    threading.Thread(target=vlm_worker, daemon=True).start()
    
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print("Failed to open the camera stream.")
        return

    print("Vision Stream Connected! Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Retrying in 1 second...")
            time.sleep(1)
            cap.release()
            cap = cv2.VideoCapture(STREAM_URL)
            continue

        # Update global frame for VLM thread (resize to save processing time if needed)
        latest_frame = cv2.resize(frame, (640, 480))
        
        # Draw status on frame
        cv2.putText(frame, f"VLM: {vlm_status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Draw Action
        color = (0, 255, 0) if "FORWARD" in robot_action else (0, 0, 255) if "TURN" in robot_action else (255, 0, 0)
        cv2.putText(frame, f"ACTION: {robot_action}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.imshow('Agentic Pet OS', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
