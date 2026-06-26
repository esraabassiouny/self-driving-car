import time
import config
from real_lane import compute_steering, compute_pwm, get_command, detect_lane_end

class State:
    def on_enter(self, car):
        pass

    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        """Called every frame. Returns string name of the next state to transition to, or None."""
        raise NotImplementedError

    def on_exit(self, car):
        pass


class LaneFollowState(State):
    def on_enter(self, car):
        car.first_lane_follow_frame = True

    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        now = time.time()
        
        # 1. Check Lane End -> U-Turn
        if detect_lane_end(mask) and (now > car.uturn_cooldown_until):
            print("🛑 Horizontal line (lane end) detected! Starting U-Turn sequence...")
            return 'UTURN_FORWARD'
            
        # 2. Check lane boundary disappear -> Stop
        if left_fit is None and (now > car.uturn_cooldown_until):
            print("🛑 Lane boundaries disappeared! Stopping.")
            return 'STOP'
            
        # 3. Check Stop Sign
        if (car.stop_sign_inside or car.stop_sign_outside) and (car.min_dist <= 40.0 or car.distance <= 40.0) and (now > car.ignore_stop_until):
            if car.stop_sign_inside:
                print(f"🛑 Stop sign detected inside lane boundaries at {car.min_dist:.1f}cm! Stopping for 3 seconds...")
            else:
                print(f"🛑 Stop sign detected outside lane boundaries at {car.min_dist:.1f}cm! Stopping for 3 seconds...")
            car.stop_until = now + 3.0
            car.ignore_stop_until = now + 8.0
            car.skip_detection_until = now + 8.0
            print("⚡ Skipping object detection for 8 seconds (during stop + resuming) to avoid duplicate detection.")
            return 'STOP_SIGN_WAIT'
            
        # 4. Check Red Light
        if car.red_light_outside and (now > car.ignore_stop_until):
            print("🛑 Traffic red light detected outside lane boundaries! Stopping for 3 seconds...")
            car.stop_until = now + 3.0
            car.ignore_stop_until = now + 8.0
            car.skip_detection_until = now + 8.0
            return 'STOP_SIGN_WAIT'
            
        # 5. Check Obstacles
        if car.has_obstacle:
            current_lane_blocked = False
            other_lane_blocked = False
            
            if car.current_lane == 'RIGHT':
                current_lane_blocked = car.right_lane_has_obstacle
                other_lane_blocked = car.left_lane_has_obstacle
            else:
                current_lane_blocked = car.left_lane_has_obstacle
                other_lane_blocked = car.right_lane_has_obstacle
                
            if current_lane_blocked:
                if other_lane_blocked:
                    # Both lanes blocked
                    if car.min_dist <= 10.0 or car.distance <= 10.0:
                        print(f" Both lanes blocked and distance <= 30cm (Radar: {car.min_dist:.1f}cm, Camera: {car.distance:.1f}cm)! Stopping.")
                        return 'STOP'
                    elif car.min_dist <= 50.0 or car.distance <= 50.0:
                        print(f" Both lanes blocked and distance <= 60cm (Radar: {car.min_dist:.1f}cm, Camera: {car.distance:.1f}cm). Slowing down.")
                        car.both_blocked_slow_down = True
                else:
                    # Lane change trigger
                    if car.current_lane == 'RIGHT':
                        print(f" Obstacle in RIGHT lane. LEFT lane is empty. Changing lane LEFT!")
                        if car.min_dist <= 35.0 or car.distance <= 35.0:
                            return 'REACH_P1'
                    else:
                        print(f" Obstacle in LEFT lane. RIGHT lane is empty. Changing lane RIGHT!")
                        return 'LANE_CHANGE_RIGHT'
                        
        # Standard Steering Control
        if left_fit is not None:
            error, lane_center, car_center = compute_steering(left_fit, right_fit, left_valid, right_valid, frame.shape, 'LANE_FOLLOW')
            car.error = error
            car.lane_center = lane_center
            car.car_center = car_center
            car.command = get_command(error)
            
            if car.first_lane_follow_frame:
                car.left_pwm = 255
                car.right_pwm = 255
                car.first_lane_follow_frame = False
            else:
                # Check for yellow light slow down
                slow_mode = now < car.yellow_slow_until
                if (slow_mode or car.both_blocked_slow_down) and (car.distance <= 60.0 or car.min_dist <= 60.0):
                    car.left_pwm, car.right_pwm = compute_pwm(error, base_speed=100)
                    car.command = get_command(error) + " (SLOW)"
                else:
                    car.left_pwm, car.right_pwm = compute_pwm(error)
        else:
            car.left_pwm = 0
            car.right_pwm = 0
            car.command = "STOPPED"
            
        return None


class LaneChangeRightState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        if left_fit is not None:
            error, lane_center, car_center = compute_steering(left_fit, right_fit, left_valid, right_valid, frame.shape, 'LANE_CHANGE_RIGHT')
            car.error = error
            car.lane_center = lane_center
            car.car_center = car_center
            car.command = "LANE_CHANGE_RIGHT"
            car.left_pwm, car.right_pwm = compute_pwm(error)
        else:
            car.left_pwm = 0
            car.right_pwm = 0
            car.command = "STOPPED"
        return None


class StopState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "STOPPED"
        if left_valid and right_valid and car.min_dist > 60.0 and car.distance > 60.0:
            print("Lanes clear and distance > 60cm. Resuming lane follow.")
            return 'LANE_FOLLOW'
        return None


class StopSignWaitState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "STOPPED"
        if time.time() >= car.stop_until:
            print(" Stop complete. Resuming lane following.")
            return 'LANE_FOLLOW'
        return None


class UTurnStop1State(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "UTURN_STOP"
        if time.time() - car.state_start_time >= 0.3:
            print(" UTURN: Moving forward...")
            return 'UTURN_FORWARD'
        return None


class UTurnForwardState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 180
        car.right_pwm = 180
        car.command = "UTURN_FORWARD"
        if time.time() - car.state_start_time >= 1.1:
            print(" UTURN: Stopping before steering...")
            return 'UTURN_STOP2'
        return None


class UTurnStop2State(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "UTURN_STOP"
        if time.time() - car.state_start_time >= 0.3:
            print("🔄 UTURN: Steering left to turn around...")
            return 'UTURN_STEER'
        return None


class UTurnSteerState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 190
        car.command = "UTURN_STEER"
        elapsed = time.time() - car.state_start_time
        if elapsed >= config.UTURN_MIN_STEER_DURATION:
            if left_valid and right_valid:
                print(" UTURN: Lane boundaries detected! Stopping...")
                return 'UTURN_STOP_FINAL'
            elif elapsed >= config.UTURN_TIMEOUT:
                print(" UTURN: Steering timeout! Following Lane.")
                return 'LANE_FOLLOW'
        return None


class UTurnStopFinalState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "UTURN_STOP"
        now = time.time()
        if now - car.state_start_time >= 1.0:
            car.current_lane = 'RIGHT'
            car.uturn_cooldown_until = now + 8.0
            print(" UTURN complete. Resuming lane following.")
            return 'LANE_FOLLOW'
        return None


class ReachP1State(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        error, lane_center, car_center = compute_steering(left_fit, right_fit, left_valid, right_valid, frame.shape, 'REACH_P1')
        car.error = error
        car.lane_center = lane_center
        car.car_center = car_center
        car.command = get_command(error)
        car.left_pwm, car.right_pwm = compute_pwm(error)
        if time.time() - car.state_start_time >= config.P1_DURATION:
            print(f"🎯 P1 duration reached ({config.P1_DURATION}s)! Transitioning to STATE_FORWARD.")
            return 'FORWARD'
        return None


class ForwardState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 200
        car.right_pwm = 200
        car.command = "FORWARD"
        if time.time() - car.state_start_time >= config.FORWARD_DURATION:
            print(f"⏭ Forward duration reached ({config.FORWARD_DURATION}s)! Transitioning to STATE_PAUSE.")
            return 'PAUSE'
        return None


class PauseState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "PAUSED"
        if time.time() - car.state_start_time >= config.PAUSE_DURATION:
            print(f"🛑 Pause duration reached ({config.PAUSE_DURATION}s)! Transitioning to STATE_STEER_RIGHT.")
            return 'STEER_RIGHT'
        return None


class SteerRightState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 195
        car.right_pwm = 0
        car.command = "STEER_RIGHT"
        if left_valid and right_valid:
            print("🎯 Both boundaries of the new lane detected! Transitioning to STATE_PAUSE_AFTER_STEER.")
            return 'PAUSE_AFTER_STEER'
        return None


class PauseAfterSteerState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 0
        car.right_pwm = 0
        car.command = "PAUSED"
        if time.time() - car.state_start_time >= config.PAUSE_AFTER_STEER_DURATION:
            print("🎯 Pause after steer complete! Transitioning to STATE_FORWARD_AFTER_STEER.")
            return 'FORWARD_AFTER_STEER'
        return None


class ForwardAfterSteerState(State):
    def update(self, car, frame, left_fit, right_fit, left_valid, right_valid, mask):
        car.left_pwm = 180
        car.right_pwm = 180
        car.command = "FORWARD"
        if time.time() - car.state_start_time >= config.FORWARD_AFTER_STEER_DURATION:
            car.current_lane = 'LEFT'
            print("🎯 Forward after steer complete! Resuming STATE_LANE_FOLLOW in LEFT lane.")
            return 'LANE_FOLLOW'
        return None
