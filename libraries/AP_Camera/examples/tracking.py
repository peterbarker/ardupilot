import cv2
import socket
import struct
import threading
import math
from pymavlink import mavutil
import time
import imutils

stop_thread_lock = threading.Lock()
stop_thread = False

#rtsp_url = "rtsp://localhost:8554/camera"
rtsp_url = "rtsp://192.168.144.25:8554/main.264"
#pymavlink_connection_url = '/dev/ttyTHS0'
pymavlink_connection_url = 'localhost:7000'

class GimbalControl:
    def __init__(self, connection_str):
        print("Connecting")
        self.master = mavutil.mavlink_connection(connection_str, baud=921600, source_system=20)
        print("Connected")
        self.master.wait_heartbeat()
        print("Heartbeat from the system (system %u component %u)" % 
              (self.master.target_system, self.master.target_component))
        self.center_x = 0
        self.center_y = 0
        self.object_detected = False
        self.lock = threading.Lock()  # Initialize a lock for shared variables

        self.control_thread = threading.Thread(target=self.send_command)
        self.tracking_status_thread = threading.Thread(target=self.send_tracking_status)  # Thread for sending camera tracking status
        self.control_thread.start()
        self.tracking_status_thread.start()  # Start tracking status thread

    def send_tracking_status(self):
        """Send tracking status in a separate thread at 50Hz."""
        global stop_thread
        while True:
            start_time = time.time()
            with self.lock:
                # Send 1 if object is detected, 2 if not
                tracking_status = 0
                with stop_thread_lock:
                    if stop_thread == True:
                        tracking_status = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS_IDLE
                    elif self.object_detected == True:
                        tracking_status = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS_ACTIVE
                    else:
                        tracking_status = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS_ERROR

                self.send_camera_tracking_image_status(
                    tracking_status,  # Example: Active tracking
                    tracking_mode=mavutil.mavlink.CAMERA_TRACKING_MODE_POINT,
                    target_data=mavutil.mavlink.CAMERA_TRACKING_TARGET_DATA_IN_STATUS,
                    point_x=0,
                    point_y=0,
                    radius=float("NaN"),
                    rec_top_x=float("NaN"),
                    rec_top_y=float("NaN"),
                    rec_bottom_x=float("NaN"),
                    rec_bottom_y=float("NaN"),
                    camera_device_id=0,
                )

            # 50Hz loop
            elapsed_time = time.time() - start_time
            sleep_time = max(0, 0.02 - elapsed_time)  # Ensure no negative sleep time
            time.sleep(sleep_time)

    def send_gimbal_manager_pitch_yaw_angles(self, pitch, yaw, pitch_rate, yaw_rate):
        msg = self.master.mav.gimbal_manager_set_pitchyaw_encode(
            self.master.target_system,
            self.master.target_component,
            0,
            0,
            pitch,
            yaw,
            pitch_rate,
            yaw_rate
        )
        self.master.mav.send(msg)

    def send_camera_tracking_image_status(self, tracking_status, tracking_mode, target_data, point_x, point_y, radius, rec_top_x, rec_top_y, rec_bottom_x, rec_bottom_y, camera_device_id):
        msg = self.master.mav.camera_tracking_image_status_encode(
            #self.master.target_system,  # Target system
            #self.master.target_component,  # Target component
            tracking_status,
            tracking_mode,
            target_data,
            point_x,
            point_y,
            radius,
            rec_top_x,
            rec_top_y,
            rec_bottom_x,
            rec_bottom_y,
            #camera_device_id
        )
        self.master.mav.send(msg)

    def send_command(self):
        global stop_thread
        while True:
            start_time = time.time()  # Record the start time of the loop
            with stop_thread_lock:
                if stop_thread == True:
                    elapsed_time = time.time() - start_time
                    sleep_time = max(0, 0.02 - elapsed_time)  # Ensure no negative sleep time
                    time.sleep(sleep_time)
                    continue

            with self.lock:  # Lock when accessing shared variables
                centre_x_copy = int(self.center_x)
                centre_y_copy = int(self.center_y)

            if (centre_x_copy == 0 and centre_y_copy == 0):
                diff_x = 0
                diff_y = 0
            else:
                diff_x = (centre_x_copy - (640 / 2)) / 2
                diff_y = -(centre_y_copy - (480 / 2)) / 2

            pitch_rate = math.radians(diff_y)
            yaw_rate = math.radians(diff_x)
            if abs(pitch_rate) > 0.1:
                pitch_rate = pitch_rate/abs(pitch_rate) * 0.1
            if abs(yaw_rate) > 0.1:
                yaw_rate = yaw_rate/abs(yaw_rate) * 0.1
            self.send_gimbal_manager_pitch_yaw_angles(float("NaN"), float("NaN"), pitch_rate, yaw_rate)

            # 50Hz
            elapsed_time = time.time() - start_time
            sleep_time = max(0, 0.02 - elapsed_time)  # Ensure no negative sleep time
            time.sleep(sleep_time)

    def update_center(self, x, y):
        with self.lock:  # Lock when updating shared variables
            self.center_x = x
            self.center_y = y


class VideoStreamer:
    def __init__(self, input_port, output_port):
        self.input_port = input_port
        self.output_port = output_port
        self.cap = self.initialize_video_capture()
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # self.out = self.initialize_video_writer()
        self.frame = None

        self.tracker = cv2.legacy.TrackerCSRT_create()  # Change to legacy namespace
        self.initBB = None
        self.last_change_time = 0

    def initialize_video_capture(self):
        print("Opening VideoCapture")
#            f'rtspsrc location={rtsp_url} latency=50 ! decodebin ! videoconvert ! appsink',
        cap = cv2.VideoCapture(
            f'rtspsrc location={rtsp_url} latency=0 buffer-mode=auto ! rtph265depay !  tee name=tee1 tee1. ! queue ! h265parse ! avdec_h265  ! videoconvert ! video/x-raw,format=BGRx ! appsink tee1. ! queue ! h265parse config-interval=15 ! video/x-h265 ! mpegtsmux ! appsink',
            cv2.CAP_GSTREAMER
        )

        

        print("Created videocapture")
        if not cap.isOpened():
            print("Error: Could not open video stream.")
            exit()
        print("returning cap")
        return cap

    def update_tracker(self, new_roi):
        self.initBB = new_roi
        self.tracker = cv2.legacy.TrackerCSRT_create()  # Change to legacy namespace
        if self.frame is not None:
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
            self.tracker.init(self.frame, new_roi)

    def grab_frame(self):
        ret, frame = self.cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#        frame = imutils.resize(frame, 500)
        return ret, frame

    def process_frame(self, gimbal_control):
        global stop_thread
        i = 0
        while True:
#            if i % 10 == 0:
#                print(f"Frame {i}")
            i += 1
            start_time = time.time()  # Record the start time of the loop
            with stop_thread_lock:
                if stop_thread == True:
                    elapsed_time = time.time() - start_time
                    sleep_time = max(0, 0.02 - elapsed_time)  # Ensure no negative sleep time
                    time.sleep(sleep_time)
                    ret, self.frame = self.grab_frame()
                    continue

            ret, self.frame = self.grab_frame()
            if not ret:
                print("Error: Unable to read frame from video stream.")
                break

            # If a bounding box exists, update the tracker and draw the rectangle
            if self.initBB is not None:
                (success, box) = self.tracker.update(self.frame)
                if success:
                    print("Success")
                    (x, y, w, h) = [int(v) for v in box]
                    center_x = x + (w // 2)
                    center_y = y + (h // 2)
                    gimbal_control.update_center(center_x, center_y)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.last_change_time = time.time()
                    with gimbal_control.lock:
                        gimbal_control.object_detected = True
                else:
                    print("Tracking failure detected, reinitializing tracker.")
                    with gimbal_control.lock:
                        gimbal_control.object_detected = False
                    self.reinitialize_tracker(gimbal_control)

            # Display the frame
#            cv2.imshow("Frame", self.frame)
#            # print(self.frame.shape)
#            key = cv2.waitKey(1) & 0xFF
#            if key == ord("q"):
#                break

        self.cleanup()

    def reinitialize_tracker(self, gimbal_control):
        """Reinitialize tracker on loss."""
        gimbal_control.update_center(0, 0)
        self.initBB = None  # Reset the bounding box
        # Optionally, use object detection to find the new location of the object and update tracker

        # Example: Use an object detection method to reinitialize the bounding box
        # detection_box = self.object_detection_method(self.frame)
        # if detection_box:
        #     self.update_tracker(detection_box)

    def cleanup(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()


class UDPReceiver:
    def __init__(self, gimbal_control, video_streamer, port=14580):
        self.gimbal_control = gimbal_control
        self.video_streamer = video_streamer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock2.bind(('127.0.0.1', 14590))
        self.udp_thread = threading.Thread(target=self.receive_udp_data)
        self.udp_thread2 = threading.Thread(target=self.receive_udp_data2)
        self.udp_thread2.daemon = True
        self.udp_thread2.start()
        self.udp_thread.daemon = True
        self.udp_thread.start()

#    def receive_udp_data(self):
#        while True:
#            global stop_thread
#            data, addr = self.sock.recvfrom(16)
#            x1, y1, x2, y2 = struct.unpack('!ffff', data)
#            print(f"Received new tracking coordinates: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
#            with stop_thread_lock:
#                stop_thread = False
            # Convert normalized coordinates to pixel values
 #           x1, y1, x2, y2 = int(x1 * self.video_streamer.frame_width), int(y1 * self.video_streamer.frame_height), int(x2 * self.video_streamer.frame_width), int(y2 * self.video_streamer.frame_height)
 #           self.video_streamer.update_tracker((x1, y1, x1 + x2, y1 + y2))

    def receive_udp_data(self):
        while True:
            global stop_thread
            data, addr = self.sock.recvfrom(16)
            x, y, x2, y2 = struct.unpack('!ffff', data)
            print(f"Received new tracking coordinates: x={x}, y={y}, x2={x2}, y2={y2}")
            with stop_thread_lock:
                stop_thread = False
           # Convert normalized coordinates to pixel values
            x, y, x2, y2 = int(x * self.video_streamer.frame_width), int(y * self.video_streamer.frame_height), int(x2 * self.video_streamer.frame_width), int(y2 * self.video_streamer.frame_height)
            rec = (x, y, x2-x, y2-y)
            rec = (x, y, 50, 50)
            print(f"{rec=}")
            self.video_streamer.update_tracker(rec)

    def receive_udp_data2(self):
        global stop_thread
        while True:
            data, addr = self.sock2.recvfrom(1)
            with stop_thread_lock:
                stop_thread = True
            print("Stop tracking called")


def main():
    gimbal_control = GimbalControl(pymavlink_connection_url)
    video_streamer = VideoStreamer(input_port=5600, output_port=5700)
    udp_receiver = UDPReceiver(gimbal_control, video_streamer)

    video_streamer.process_frame(gimbal_control)

if __name__ == "__main__":
    main()
