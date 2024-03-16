import time
import math
import logging

class PositionEstimator:
    '''
    A PositionEstimator that uses accelerometer and gyroscope readings of an imu to 
    predict position in-between readings from a gps, due to their difference in update
    frequencies. 

    Should be used unthreaded if there is not a significant difference
    between the gps update frequency and donkey loop frequency.

    Should be used threaded only when testing estimator effectiveness;
    the estimated position is only useful when the autopilot (an unthreaded part)
    needs to calculate error (CTE, also unthreaded) for steering values.

    In order to allow the autopilot to use the estimations at a higher frequency,
    the donkey loop frequency needs to be increased.
    '''

    def __init__(self):
        # position, velocity, and orientation values
        self.x = 0.
        self.y = 0.
        self.vx = 0.
        self.vy = 0.
        self.yaw = 0.

        # last gps values received
        self.last_pos_x = 0.
        self.last_pos_y = 0.

        # timestamps for calculating time difference
        self.last_run_time = time.time()
        self.last_update_time = time.time()
        self.last_gps_time = time.time()

        self.start_time = time.time()

        # create file for analysis purposes
        self.file = open('estimations.csv', 'w')
        # write column headers
        self.file.write('(est_x, est_y), (act_x, act_y), timestamp\n')

    def update(self):
        # runs as frequently as possible; update stored position using stored velocity and time difference

        # update current time to find time difference since last update
        curr_time = time.time()
        dt = curr_time - self.last_update_time

        # update stored position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # update time for next update
        self.last_update_time = curr_time



    def run_threaded(self, acl_x: float, acl_y: float, gyr_x: float, pos_x: float, pos_y: float):
        # updates stored velocity and orientation using accelerometer and gyroscope readings

        curr_time = time.time()
        dt = curr_time - self.last_run_time

        # update orientation
        self.yaw += float(gyr_x) * dt

        # rotate accerlation vectors
        ax = float(acl_x) * math.cos(-self.yaw) - float(acl_y) * math.sin(-self.yaw)
        ay = float(acl_x) * math.sin(-self.yaw) - float(acl_y) * math.cos(-self.yaw)

        # update velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # update time for next run
        self.last_run_time = curr_time

        # checks if the pos/x and pos/y from gps have recently been updated
        # if true, reset stored position and velocity to match actual values
        if pos_x != self.last_pos_x or pos_y != self.last_pos_y:
            #print('resetting with gps instruciton')
            # calculate time difference since last gps update
            gps_dt = curr_time - self.last_gps_time
            # reset position based on gps values
            pos_x = float(pos_x)
            pos_y = float(pos_y)
            self.x = pos_x
            self.y = pos_y
            self.z = 0
            # reset velocity based on gps delta
            self.vx = (pos_x - self.last_pos_x) / gps_dt
            self.vy = (pos_y - self.last_pos_y) / gps_dt
            self.vz = 0
            # reset yaw
            a_mag = math.sqrt((pos_x - self.last_pos_x)**2 + (pos_y - self.last_pos_y)**2)
            b_mag = 1
            self.yaw = math.acos((pos_y - self.last_pos_y) / (a_mag * b_mag))
            # update last gps time to current
            self.last_gps_time = curr_time

        # updates last known gps position to current gps position
        self.last_pos_x = pos_x
        self.last_pos_y = pos_y

        # write predicted pos and last known true pos to file
        self.file.write(f'({self.x},{self.y}), ({pos_x}, {pos_y}), {curr_time - self.start_time}\n')

        return self.x, self.y, self.yaw


    def run(self, acl_x: float, acl_y: float, gyr_x: float, pos_x: float, pos_y: float):
        # essentially a combination of update and run_threaded, for testing purposes in __main__

        self.update()
        self.run_threaded(acl_x, acl_y, gyr_x, pos_x, pos_y)

        return self.x, self.y, self.yaw

if __name__ == "__main__":
    # outputs position estimates without gps signal; very prone to drifting away because of the lack of gps/velocity/position resets,
    # but should still output an accurate orientation
    print('starting main')
    from imu_oakd import IMU
    p = IMU(sensor="oakd_BNO086")
    print('imu set')
    est = PositionEstimator()
    print('estimator set')
    while True:
        imu_data = p.run()
        data = est.run(imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5], 0, 0)
        print(data)
        time.sleep(0.05)
