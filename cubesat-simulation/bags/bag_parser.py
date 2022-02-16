import rclpy.serialization
import sqlite3
import pandas as pd
from rosidl_runtime_py.utilities import get_message

def exchange_by_zero(element, type):
    return str(element) + "++"

def deserialize(messages, topics):
    for i in range(messages['data'].size):
        msg_type = topics[topics['id']==messages['topic_id'][i]]['type'].values[0]
        messages['data'][i] = rclpy.serialization.deserialize_message(messages['data'][i],get_message(msg_type))

bag_file = 'rosbag2_2022_02_04-12_31_28.db3'

cnx = sqlite3.connect(bag_file)

topics = pd.read_sql_query("SELECT * FROM topics", cnx)
messages = pd.read_sql_query("SELECT * FROM messages", cnx)

deserialize(messages, topics)

topics.to_csv('topics.csv')
messages.to_csv('messages.csv')