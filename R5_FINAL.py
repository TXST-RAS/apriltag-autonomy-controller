import math
import time
import cv2
import numpy as np
import serial
from pupil_apriltags import Detector
from picamera2 import Picamera2

#-------------------------
# MOTOR COMMANDS
#-------------------------

beacon_dropoff = [
    #start
    "F 2000",
    "WEST 1",
    "FTOF 100",
    "NORTH 1",
    "F 5000",
    #"ARM 1", #Beacon Dropoff
]

rock_pickup = [
    "FTOF 100",
    "B 1000",
    "EAST 1",
    "F 10000",
    "NORTH 1",
    "FTOF 90",
    "B 400",
    "EAST 1",
    "FTOF 100",
    "SOUTH 1",
    "F 4500",
    "EAST 1",

    #enter cave
    "FTOF 100",
    "NORTH 1",
    "FTOF 60",
    #"B 500",
    "WEST 1",
    "RTOF 60",
    "WEST 1",
    "FTOF 100",
    "SOUTH 1",
    "FTOF 100",
    "EAST 1",
    "FTOF 100",
    "NORTH 1",
    "F 5000",
    "WEST 1",

    #exit cave
    "F 10000",
    "WEST 1",
    "FTOF 150",
    "WEST 1",
    "B 15000",
    "SOUTH 1",
    #"L 1000",
    "SOUTH 1",
    "FTOF 75",
    "B 500",
    #"R 4000",
    "WEST 1",
    "F 2000",
    "SOUTH 1",
    "FTOF 75"
    #end
]

box1_pickup = [
    "B 500",
    "WEST 1",
    "LTOF 50",
    "WEST 1",
    "B 3750",
    "RS 1"
]

box2_pickup = [
    "F 7000",
    "WEST 1",
    "F 7000",
    "NORTH 1",
    "FTOF 125",
    "WEST 1",
    "RTOF 50",
    "WEST 1",
    "B 3000",
    "LS 1"
]

dropoff_setup1 = [
    "L 8000",
    "SOUTH 1",
    "F 5000"
]

dropoff_setup2 = [
    "EAST 1",
    "B 1000"
]

#dropoff paths
dropoff0 = [
    "NORTH 1",
    "B 1000",
    "L 500",
    "DROP 1"
]

dropoff1 = [
    "NORTH 1",
    "B 500",
    "L 500",
    "DROP 1"
]

dropoff2 = [
    "NORTH 1",
    "L 500",
    "DROP 1"
]

dropoff3 = [
    "SOUTH 1",
    "B 1000",
    "L 500",
    "DROP 1"
]

dropoff4 = [
    "SOUTH 1",
    "B 1000",
    "L 500",
    "DROP 1"
]

#Serial Stuff
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# CAMERA / DETECTOR CONFIG
WIDTH = 640
HEIGHT = 480
TAG_FAMILY = "tag36h11"
 
# Speed/accuracy knobs:
DECIMATE = 1.0        # 1.0 = best range/accuracy, 2.0+ faster
SIGMA = 0.0
THREADS = 2
REFINE_EDGES = True

# TAG SIZE
TAG_SIZE_METERS = 0.08
 
# CAMERA INTRINSICS
FX = 785
FY = 785
CX = WIDTH/2
CY = HEIGHT/2
 

#-------------------------
#Function Definitions
#-------------------------

#ROBOT STUFF
class MegaController:
    def __init__(self, port="/dev/ttyACM0", baud=115200, read_timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=read_timeout)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
 
    def send_and_wait(self, command, timeout=10.0):
        self.ser.reset_input_buffer()
        self.ser.write((command.strip() + "\n").encode())
        self.ser.flush()
 
        start = time.time()
 
        while time.time() - start < timeout:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
 
            print(f"[Mega] {line}")
 
            if line.lower() == "done":
                return True
 
        return False
 
    def close(self):
        self.ser.close()

#Serial commands execution
def command_exectution(commands):
    for cmd in commands:
            print(f"Running: {cmd}")
            success = robot.send_and_wait(cmd, timeout=15.0)
 
            if not success:
                print(f"Command failed or timed out: {cmd}")
                break
 
            print(f"Completed: {cmd}\n")

def main():
    # -----------------------
    # ROBOT INIT
    # ------------------------
    robot = MegaController(SERIAL_PORT, BAUD_RATE)
    # -----------------------
    # INIT CAMERA (CSI)
    # -----------------------
    picam2 = Picamera2()
    cams = Picamera2.global_camera_info()
    print("Detected cameras:", cams)

    if not cams:
        raise SystemExit(
            "No CSI camera detected by libcamera.\n"
            "Fix camera detection first (ribbon/enable camera) and verify:\n"
            "  rpicam-hello --list-cameras"
        )
 
    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
    )

    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)
 
    # -----------------------
    # INTRINSICS
    # -----------------------
    global FX, FY, CX, CY
    camera_params = (float(FX), float(FY), float(CX), float(CY))

    # -----------------------
    # DETECTOR
    # -----------------------
    detector = Detector(
        families=TAG_FAMILY,
        nthreads=THREADS,
        quad_decimate=DECIMATE,
        quad_sigma=SIGMA,
        refine_edges=REFINE_EDGES,
    )
 
    command_list = rock_pickup
    command_list += box1_pickup
    command_list += box2_pickup
    command_list += dropoff_setup1
    command_execution(command_list, robot)

    try:
        while True:
            rgb = picam2.capture_array()
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=TAG_SIZE_METERS
            )
 
            for d in detections:
                match d.tag_id:
                    case 0:
                        command_list = dropoff_setup2
                        command_list += dropoff0    
                    case 1:
                        command_list = dropoff_setup2
                        command_list += dropoff1
                    case 2:
                        command_list = dropoff_setup2
                        command_list += dropoff2
                    case 3:
                        command_list = dropoff_setup2
                        command_list += dropoff3
                    case 4:
                        command_list = dropoff_setup2
                        command_list += dropoff4
    except KeyboardInterrupt:
        picam2.stop()
        cv2.destroyAllWindows()
        robot.close()
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        robot.close()
 
if __name__ == "__main__":
    main() 
