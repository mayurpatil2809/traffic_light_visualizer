#!/usr/bin/env python
import rospy
import tkinter as tk
from spat_profile_pub.msg import SpaT

class TrafficLightVisualizer(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.geometry("600x450")
        self.title("Traffic Light Visualizer")

        self.light_canvas = tk.Canvas(self, height=450, width=600)
        self.light_canvas.pack()

        self.route_text = self.light_canvas.create_text(300, 420, text="Current States: Mixed Route 1", font=("Arial", 14))
        self.traffic_lights = {}
        self.labels = {}
        self.state_labels = {}

    def update_light(self, data):
        tl_id = data.id
        state = data.current_state

        columns = 5  # No. of TLs in a row
        row = (tl_id - 1) // columns
        col = (tl_id - 1) % columns

        x_position = col * 100 + 30
        y_position = row * 100 + 10

        if tl_id not in self.traffic_lights:
            self.traffic_lights[tl_id] = self.light_canvas.create_oval(x_position, y_position, x_position + 30, y_position + 30, fill="gray")
            self.labels[tl_id] = self.light_canvas.create_text(x_position + 15, y_position + 50, text=f"TL: {tl_id}")
            self.state_labels[tl_id] = self.light_canvas.create_text(x_position + 15, y_position + 70, text="State: ?", font=("Arial", 10))

        color = "red" if state == 3 else ("yellow" if state == 8 else "green")
        state_text = "Red" if state == 3 else ("Yellow" if state == 8 else "Green")
        self.light_canvas.itemconfig(self.traffic_lights[tl_id], fill=color)
        self.light_canvas.itemconfig(self.state_labels[tl_id], text=f"State: {state_text}")

def listener():
    vis = TrafficLightVisualizer()

    def callback(data):
        vis.update_light(data)

    rospy.init_node('traffic_light_visualizer', anonymous=True)
    rospy.Subscriber('spat_data', SpaT, callback)

    vis.mainloop()

if __name__ == '__main__':
    listener()


