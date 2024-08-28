from bot import *
import time
import math
from camera.camera import Camera
from gpiozero import LED,PWMLED,PWMOutputDevice,DigitalOutputDevice




def calculate_distance(point1, point2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def detect_ball(robot, cam):
    """Detects a tennis ball and returns its position."""
    attempt = 0 
    detected = 0
    while True:
        if attempt > 3:
            print("Rotating to search for the ball...")
            robot.rotate(0.174533)  # Rotate ~10 degrees
            attempt = 0
        
        # Capture frame
        frame = cam.get_frame()

        # Detect balls
        detections = cam.ball_detector(robot.state)

        if len(detections) == 0:
            print("No ball detected, increasing attempt count...")
            attempt += 1
        else:
            if detected > 3:
                ball = detections[0]
                point = [ball["y"], ball["x"]]
                print(f"Detected ball at X: {ball['y']}, Y: {ball['x']}")
                turn_on()
                return point  # Return the detected ball's position
            detected +=1

def turn_on(duration=2):
    """
    Turn the speaker on for a specified duration.
    
    :param duration: Time in seconds to keep the speaker on.
    """
    speaker = PWMOutputDevice(24)
    speaker.frequency = 4000 
    speaker.value = 0.5  # 50% duty cycle for sound

    # Sleep for the duration of the beep
    time.sleep(duration)
    
    # Turn off the speaker after the beep duration
    speaker.off()

def main():
    # Initialize Classes
    cam = Camera()
    robot = Robot()
    
    #Drive to center
    robot.drive_to_location([2.5,2]) #[2.5,2]


    # Detect the ball
    target_point = detect_ball(robot, cam)

    #Drive to ball
    robot.drive_to_location(target_point) 
    
    time.sleep(2)

    # Return to origin after reaching the ball
    print("Returning to origin...")
    robot.drive_to_location([0, 0])

# Run the main function
if __name__ == "__main__":
    main()
    

