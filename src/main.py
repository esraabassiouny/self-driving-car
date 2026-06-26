from object_detection import (
    init_detector,
    detect_objects,
    stop_camera
)

import cv2
import numpy as np
import time
import os
from datetime import datetime
from real_lane import *

# Import our modular components
import config
import hardware
import states
import vision_utils

# Create debug folders
os.makedirs("debug/lane_following", exist_ok=True)
os.makedirs("debug/roi", exist_ok=True)
os.makedirs("debug/mask", exist_ok=True)
os.makedirs("debug/sliding", exist_ok=True)
os.makedirs("frames", exist_ok=True)


class CarController:
    def __init__(self):
        # 1. Initialize hardware controller
        self.hardware = hardware.SerialController()
        
        # 2. State Machine Registration
        self.state_instances = {
            'LANE_FOLLOW': states.LaneFollowState(),
            'LANE_CHANGE_RIGHT': states.LaneChangeRightState(),
            'STOP': states.StopState(),
            'STOP_SIGN_WAIT': states.StopSignWaitState(),
            'UTURN_STOP1': states.UTurnStop1State(),
            'UTURN_FORWARD': states.UTurnForwardState(),
            'UTURN_STOP2': states.UTurnStop2State(),
            'UTURN_STEER': states.UTurnSteerState(),
            'UTURN_STOP_FINAL': states.UTurnStopFinalState(),
            'REACH_P1': states.ReachP1State(),
            'FORWARD': states.ForwardState(),
            'PAUSE': states.PauseState(),
            'STEER_RIGHT': states.SteerRightState(),
            'PAUSE_AFTER_STEER': states.PauseAfterSteerState(),
            'FORWARD_AFTER_STEER': states.ForwardAfterSteerState()
        }
        
        # Active state tracking
        self.current_state_name = 'LANE_FOLLOW'
        self.active_state = self.state_instances[self.current_state_name]
        
        self.current_lane = 'RIGHT'
        self.first_lane_follow_frame = True
        self.state_start_time = time.time()
        
        # Cooldowns and Timing-based limits
        self.uturn_cooldown_until = 0.0
        self.ignore_stop_until = 0.0
        self.skip_detection_until = 0.0
        self.stop_until = 0.0
        self.yellow_slow_until = 0.0
        
        # Telemetry & Camera Inputs
        self.min_dist = 999.0
        self.distance = 999.0
        
        # Obstacle detection flags
        self.has_obstacle = False
        self.both_blocked_slow_down = False
        self.left_lane_has_obstacle = False
        self.right_lane_has_obstacle = False
        
        # Traffic elements
        self.stop_sign_inside = False
        self.stop_sign_outside = False
        self.red_light_outside = False
        
        # Output speeds and display info
        self.left_pwm = 0
        self.right_pwm = 0
        self.command = "STOPPED"
        self.error = 0.0
        self.lane_center = 0.0
        self.car_center = 0.0
        
        # Start initial state
        self.active_state.on_enter(self)

    def change_state(self, new_state_name):
        if new_state_name not in self.state_instances:
            print(f"⚠️ Warning: state {new_state_name} does not exist!")
            return
            
        print(f"🔄 State Transition: {self.current_state_name} -> {new_state_name}")
        self.active_state.on_exit(self)
        self.current_state_name = new_state_name
        self.active_state = self.state_instances[new_state_name]
        self.state_start_time = time.time()
        self.active_state.on_enter(self)

    def update(self, frame, left_fit, right_fit, left_valid, right_valid, mask):
        # Reset dynamic flags calculated per-frame
        self.both_blocked_slow_down = False
        
        # Delegate update to active state
        next_state_name = self.active_state.update(
            self, frame, left_fit, right_fit, left_valid, right_valid, mask
        )
        
        if next_state_name is not None:
            self.change_state(next_state_name)

    def send_controls(self):
        # Clip motor PWM speeds safely
        self.left_pwm = int(np.clip(self.left_pwm, 0, 255))
        self.right_pwm = int(np.clip(self.right_pwm, 0, 255))
        self.hardware.send_motor_speeds(self.left_pwm, self.right_pwm, self.current_state_name, self.min_dist)


def main():
    print("🚗 Modular Control Orchestrator started (CTRL+C to stop)")
    
    # Initialize object detector (YOLO)
    init_detector()
    
    # Initialize video recording
    video_writer = None
    if config.SAVE_VIDEO:
        os.makedirs("videos", exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_name = datetime.now().strftime("videos/drive_%Y%m%d_%H%M%S.avi")
        video_writer = cv2.VideoWriter(video_name, fourcc, 5.0, (800, 600))
        print(f"📹 Recording video: {video_name}")

    # Instantiate the unified controller
    car = CarController()
    
    # Start Arduino ultrasonic distance sweeping
    car.hardware.start_sweep()
    
    frame_id = 0
    
    try:
        while True:
            # 1. Update distance sensor telemetry
            car.min_dist = car.hardware.read_distance(default_dist=car.min_dist)
            
            # Reset visual distance
            car.distance = 999.0
            
            # 2. Run object detection (using YOLO)
            time_detect = time.time()
            frame, detections = detect_objects(skip_yolo=True)
            time_after = time.time() - time_detect
            print("time_detection", time_after)
            
            # 3. Process Lane Detection
            warped, Minv, M, roi_debug = perspective_transform(frame)
            warped_change, Minv_change, M_change = vision_utils.perspective_transform_lane_change(frame)
            
            cv2.imwrite(f"debug/roi/frame_{frame_id:05d}.jpg", roi_debug)
            
            mask = threshold_white(warped)
            mask_change = threshold_white(warped_change)
            cv2.imshow("Mask", mask)
            
            if config.SAVE_MASK:
                cv2.imwrite(f"debug/mask/frame_{frame_id:05d}.jpg", mask)
                
            histogram = np.sum(
                mask_change[mask_change.shape[0]//2:, :],
                axis=0
            )
            
            peaks = []
            for i in range(50, len(histogram) - 50):
                if histogram[i] > 1000:
                    if histogram[i] == np.max(histogram[i-50:i+50]):
                        peaks.append(i)
            print("Lane change peaks:", peaks)
            
            hist_img = cv2.cvtColor(mask_change, cv2.COLOR_GRAY2BGR)
            for p in peaks:
                cv2.line(hist_img, (p, 0), (p, hist_img.shape[0]), (0, 0, 255), 3)
            cv2.imshow("3 Lane Peaks", hist_img)
            
            # Fit polynomial to get left and right lanes
            left_fit, right_fit, left_valid, right_valid = fit_polynomial(mask)
            
            # 4. Process YOLO Detections relative to lanes
            car.stop_sign_inside = False
            car.stop_sign_outside = False
            car.red_light_outside = False
            car.left_lane_has_obstacle = False
            car.right_lane_has_obstacle = False
            car.has_obstacle = False
            
            if len(detections) == 2:
                for detection in detections:
                    conf = detection["conf"]
                    x1, y1, x2, y2 = detection["box"]
                    name = detection["name"]
                    area = detection["area"]
                    det_dist = vision_utils.estimate_distance(area)
                    car.distance = min(car.distance, det_dist)
                    
                    print(f"Detected {name} (conf: {conf:.2f}, dist: {det_dist:.2f} cm)")
                    
                    in_lane = vision_utils.is_inside_lane((x1, y1, x2, y2), left_fit, right_fit, M)
                    color = (0, 0, 255) if in_lane else (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    label = f"{name} {conf:.2f}"
                    dist_label = f"dist: {det_dist:.2f} cm"
                    cv2.putText(frame, label, (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(frame, dist_label, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                    # Lane localization
                    if left_fit is not None and right_fit is not None:
                        x_center = (x1 + x2) / 2.0
                        y_bottom = float(y2)
                        pts = np.array([[[x_center, y_bottom]]], dtype=np.float32)
                        pts_warped = cv2.perspectiveTransform(pts, M)
                        xw, yw = pts_warped[0][0]
                        
                        x_left = np.polyval(left_fit, yw)
                        x_right = np.polyval(right_fit, yw)
                        lane_width_pixels = x_right - x_left
                        
                        print(f"xw: {xw}, x_left+100: {x_left+100}")
                        
                        if x_left + 100 <= xw <= x_right:
                            car.has_obstacle = True
                            if car.current_lane == 'RIGHT':
                                car.right_lane_has_obstacle = True
                            else:
                                car.left_lane_has_obstacle = True
                        elif xw < x_left + 100:
                            if car.current_lane == 'RIGHT':
                                car.left_lane_has_obstacle = True
                        elif x_right < xw <= (x_right + lane_width_pixels):
                            if car.current_lane == 'LEFT':
                                car.right_lane_has_obstacle = True
                                
                    if name == "stop-sign":
                        if in_lane:
                            car.stop_sign_inside = True
                        else:
                            car.stop_sign_outside = True
                    elif name == "red":
                        if not in_lane:
                            car.red_light_outside = True
                    elif name == "yellow":
                        if in_lane:
                            car.yellow_slow_until = time.time() + config.YELLOW_SLOW_DURATION
            
            # 5. Draw Lane overlays and debug path
            if left_fit is not None:
                result = draw_lane(frame, mask, left_fit, right_fit, Minv)
                if config.SAVE_LANE:
                    cv2.imwrite(f"debug/lane_following/frame_{frame_id:05d}.jpg", result)
            else:
                result = frame.copy()
                
            # 6. Update state machine
            car.update(frame, left_fit, right_fit, left_valid, right_valid, mask)
            
            # 7. Draw OSD/OSD center lines
            if left_fit is not None and car.current_state_name in ['LANE_FOLLOW', 'REACH_P1']:
                # Green = Detected Lane Center
                cv2.line(result, (int(car.lane_center), 0), (int(car.lane_center), result.shape[0]), (0, 255, 0), 3)
                # Red = Desired Car Center
                cv2.line(result, (int(car.car_center), 0), (int(car.car_center), result.shape[0]), (0, 0, 255), 3)
                
            # Compute text for current state
            if "UTURN" in car.current_state_name:
                state_text = "LEFT U-TURN"
            elif time.time() < car.yellow_slow_until:
                state_text = "SLOW DOWN"
            elif car.current_state_name in ['REACH_P1', 'FORWARD', 'PAUSE', 'STEER_RIGHT', 'PAUSE_AFTER_STEER']:
                state_text = "CHANGE LANE"
            else:
                state_text = car.current_state_name
                
            # Lane visualization text
            if not left_valid and not right_valid:
                display_lane = "NO LANE"
            else:
                display_lane = car.current_lane
                
            lane_color = (0, 0, 255) if display_lane == "NO LANE" else (255, 255, 255)
            
            # Put basic OSD info
            cv2.putText(result, f"State: {state_text}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(result, f"Lane: {display_lane}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, lane_color, 2)
            cv2.putText(result, f"Speed: L{car.left_pwm} R{car.right_pwm}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Put extra details only if in LANE_FOLLOW
            if car.current_state_name == 'LANE_FOLLOW':
                cv2.putText(result, f"Curve: {car.command}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(result, f"Error: {int(car.error)}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
            # Adjacent lane occupancy indicators
            left_lane_status = "OBSTACLE" if car.left_lane_has_obstacle else ("UNKNOWN" if left_fit is None or right_fit is None else "EMPTY")
            right_lane_status = "OBSTACLE" if car.right_lane_has_obstacle else ("UNKNOWN" if left_fit is None or right_fit is None else "EMPTY")
            
            left_color = (0, 255, 0) if left_lane_status == "EMPTY" else ((0, 0, 255) if left_lane_status == "OBSTACLE" else (0, 255, 255))
            right_color = (0, 255, 0) if right_lane_status == "EMPTY" else ((0, 0, 255) if right_lane_status == "OBSTACLE" else (0, 255, 255))
            
            cv2.putText(result, f"LEFT Lane: {left_lane_status}", (560, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, left_color, 2)
            cv2.putText(result, f"RIGHT Lane: {right_lane_status}", (560, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, right_color, 2)
            
            # 8. Send motor commands to Arduino
            car.send_controls()
            
            # Show processed output window
            cv2.imshow("Lane Following & Obstacle Avoidance", result)
            
            if config.SAVE_LANE:
                cv2.imwrite(f"debug/lane_following/frame_{frame_id:05d}.jpg", result)
                
            if video_writer is not None:
                video_writer.write(result)
                
            cv2.imwrite(f"frames/frame_{frame_id:06d}.jpg", result)
            
            # Check keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC to exit
                break
            
            frame_id += 1
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping Car...")
        car.hardware.stop_car()
    finally:
        # Cleanup
        try:
            car.hardware.stop_car()
        except:
            pass
        if video_writer is not None:
            video_writer.release()
        cv2.destroyAllWindows()
        car.hardware.close()
        print("✅ Stopped safely")


if __name__ == "__main__":
    main()
