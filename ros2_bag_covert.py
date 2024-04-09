import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs_py import point_cloud2
import os
import sys

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        if topic_name not in self.topic_id:
            print(f"Topic {topic_name} does not exist.")
            return []
    
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]
    
    def save_nebula_message(self, topic_name, save_path):
        msg = self.get_messages(topic_name)
        if not msg: 
            return
        #nebula_msg_path = os.path.join(save_path, 'nebula_msg')
        #os.makedirs(nebula_msg_path, exist_ok=True) 
        for i, (timestamp, point_cloud) in enumerate(msg):
            cloud = list(point_cloud2.read_points(point_cloud)) 
            filename = os.path.join(save_path, f"{i}.txt")
            with open(filename, 'w') as f:
                t = point_cloud.header.stamp.sec + point_cloud.header.stamp.nanosec * 1e-9
                f.write(f"{t:.10f}\n")
                f.write(f"{len(cloud)}\n")
                for pt in cloud:
                    f.write(f"{pt[0]:.20f} {pt[1]:.20f} {pt[2]:.20f}\n")

    def save_wheel_message(self, topic_name, save_path):
        msg = self.get_messages(topic_name)
        if not msg: 
            return
        filename = os.path.join(save_path, f"wheel_mea.txt")
        with open(filename, 'w') as f:
            for i, (timestamp, measurement) in enumerate(msg):
                t = measurement.header.stamp.sec + measurement.header.stamp.nanosec * 1e-9
                f.write(f"{measurement.velocity_left:.20f} {measurement.velocity_right:.20f} {t:.10f}\n")

    def save_imu_message(self, topic_name, save_path):
        msg = self.get_messages(topic_name)
        if not msg: 
            return
        filename = os.path.join(save_path, f"imu_mea.txt")
        with open(filename, 'w') as f:
            for i, (timestamp, measurement) in enumerate(msg):
                t = measurement.header.stamp.sec + measurement.header.stamp.nanosec * 1e-9
                f.write(f"{t:.10f} {measurement.linear_acceleration.x:.20f} {measurement.linear_acceleration.y:.20f} {measurement.linear_acceleration.z:.20f} {measurement.angular_velocity.x:.20f} {measurement.angular_velocity.y:.20f} {measurement.angular_velocity.z:.20f}\n") 

    def save_livox_message(self, topic_name, save_path):
        msg = self.get_messages(topic_name)
        if not msg: 
            return
        livox_msg_path = os.path.join(save_path, 'livox_msg')
        os.makedirs(livox_msg_path, exist_ok=True)
        for i, (timestamp, point_cloud) in enumerate(msg):
            filename = os.path.join(livox_msg_path, f"{i}.txt")
            with open(filename, 'w') as f:
                t = point_cloud.header.stamp.sec + point_cloud.header.stamp.nanosec * 1e-9
                f.write(f"{t:.10f}\n")
                f.write(f"{len(point_cloud.points)}\n")
                for pt in point_cloud.points:
                    f.write(f"{pt.x:.20f} {pt.y:.20f} {pt.z:.20f}\n")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py bag_file save_path")
        sys.exit(1)

    bag_file = sys.argv[1]
    save_path = sys.argv[2]

    parser = BagFileParser(bag_file)

    #parser.save_nebula_message("/nebula200/mtof_points2",save_path)

    parser.save_wheel_message("/wheel_vels", save_path)

    parser.save_imu_message("/livox/imu", save_path)

    #parser.save_livox_message("/livox/lidar",save_path)
