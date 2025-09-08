from . import (
    json,
    utime,
    ext
)


class DistanceScanner:
    def __init__(self, servo:ext.ServoMotor, dist_obj:object, *, 
                 min_angle:float=0.0, max_angle:float=180.0, angle_step:float=2.0, 
                 max_distance:int=200, scan_delay_ms:int=80, measurement_delay_ms:int=20):
        self.__servo = servo
        self.__dobj = dist_obj

        self.__min_angle = min_angle
        self.__max_angle = max_angle
        self.__angle_step = angle_step
        self.__current_angle = self.__min_angle
        self.__direction = 1  # 1: increasing, -1: decreasing

        self.__max_distance = max_distance                    # ignore distances over 200cm
        self.__scan_delay_ms = scan_delay_ms                  # increase delay between measurements
        self.__measurement_delay_ms = measurement_delay_ms    # measurement stabilization time

        print("Moving to initial position...")
        self.__servo[0].duration_ms = 100 
        self.__servo[0].angle = self.__min_angle
        self.__servo[0].wait_completion(timeout_ms=2000)

    def get_distance(self) -> int | None:
        distances = []
        for i in range(5):
            distance = self.__dobj[0].value[0]
            if distance is not None and distance <= self.__max_distance:
                distances.append(distance)
            utime.sleep_ms(self.__measurement_delay_ms)

        if distances:
            return sum(distances) // len(distances)
        else:
            return None
    
    def move_to_next_angle(self):
        next_angle = self.__current_angle + (self.__angle_step * self.__direction)

        if next_angle >= self.__max_angle:
            next_angle = self.__max_angle
            self.__direction = -1
        elif next_angle <= self.__min_angle:
            next_angle = self.__min_angle
            self.__direction = 1

        self.__current_angle = next_angle
        self.__servo[0].angle = self.__current_angle

        utime.sleep_ms(self.__scan_delay_ms)

    def scan_once(self) -> dict:
        distance = self.get_distance()

        scan_data = {
            'angle': self.__current_angle,
            'distance': distance,
            'timestamp': utime.ticks_ms(),
            'direction': 'CW' if self.__direction == 1 else 'CCW'
        }

        return scan_data

    def continuous_scan(self):
        print("Continuous scan started...")
        print("Angle range: {}° ~ {}°".format(self.__min_angle, self.__max_angle))
        print("Angle step: {}°".format(self.__angle_step))
        print()

        scan_count = 0
        last_direction_change = 0

        try:
            while True:
                scan_data = self.scan_once()
                
                json_data = json.dumps(scan_data)

                print(json_data)

                if self.__direction != (1 if scan_count == 0 else (1 if scan_data['direction'] == 'CW' else -1)):
                    cycle_count = (scan_count - last_direction_change) // 2
                    if cycle_count > 0:
                        print(f"# Scan cycle {cycle_count} complete", end='\r')
                    last_direction_change = scan_count
                
                self.move_to_next_angle()
                
                scan_count += 1
                
                if scan_count % 50 == 0:
                    print(f"# Total scans: {scan_count}", end='\r')

        except KeyboardInterrupt:
            print("\nScan interrupted")
            self.stop()
    
    def stop(self):
        print("Stopping scanner...")
        self.__dobj.deinit()
        
        self.__servo[0].angle = 90.0
        self.__servo[0].wait_completion(timeout_ms=2000)
        self.__servo.deinit()
    
    def calibrate_servo(self):
        print("Servo motor calibration started...")
        angles = [0, 45, 90, 135, 180]
        
        for angle in angles:
            print(f"Moving to angle {angle}...")
            self.__servo[0].angle = angle
            utime.sleep_ms(1000)
            
            distance = self.get_distance()
            print(f"Distance at angle {angle}: {distance}cm")
        
        self.__servo[0].angle = 90.0
        print("Calibration complete")