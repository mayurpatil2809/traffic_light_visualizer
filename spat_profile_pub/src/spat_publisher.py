#!/usr/bin/env python
import rospy
from spat_profile_pub.msg import SpaT
import yaml

def publish_tl_state(pub, tl_id, current_state, current_duration, next_state):
    msg = SpaT()
    msg.id = int(tl_id)
    msg.current_state = current_state
    msg.current_duration = max(0, int(current_duration))
    msg.next_state = next_state
    pub.publish(msg)

def load_traffic_lights_config():
    # Load the TL configuration from YAML file
    config_path = rospy.get_param("/traffic_lights_config_path")
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config["traffic_lights"]

def get_initial_state(states, durations, elapsed_time):
    """
    Calculate the initial state and remaining time for a given elapsed time.
    """
    time_accumulated = 0
    for index, duration in enumerate(durations):
        if time_accumulated + duration > elapsed_time:
            return index, duration - (elapsed_time - time_accumulated)
        time_accumulated += duration
    return 0, durations[0]  # Default to the first state

if __name__ == "__main__":
    rospy.init_node('spat_publisher')
    pub = rospy.Publisher('spat_data', SpaT, queue_size=10)
    rate = rospy.Rate(10)

    profile_number = rospy.get_param('profile_number', 1)
#    profile_number = 10

    tl_config = load_traffic_lights_config()

    original_durations = {tl_id: tl_info["durations"][:] for tl_id, tl_info in tl_config.items()}
    current_index = {tl_id: 0 for tl_id in tl_config}
    last_time_updated = rospy.get_time()

    elapsed_time = profile_number - 1 
    print(f"Profile number: {profile_number}, Elapsed time: {elapsed_time}")

    for tl_id, info in tl_config.items():
        total_duration = sum(info['durations'])
        time_passed = elapsed_time % total_duration
        current_index[tl_id], info['durations'][current_index[tl_id]] = get_initial_state(info['states'], info['durations'], time_passed)
        print(f"Initial state and duration for TL ID {tl_id}: State {current_index[tl_id]}, Duration {info['durations'][current_index[tl_id]]}")

    try:
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            if current_time - last_time_updated >= 1:
                last_time_updated = current_time
                for tl_id, info in tl_config.items():
                    info['durations'][current_index[tl_id]] -= 1
                    if info['durations'][current_index[tl_id]] <= 0:
                        current_index[tl_id] = (current_index[tl_id] + 1) % len(info['states'])
                        info['durations'][current_index[tl_id]] = original_durations[tl_id][current_index[tl_id]]

            for tl_id, info in tl_config.items():
                current_state = info['states'][current_index[tl_id]]
                current_duration = info['durations'][current_index[tl_id]]
                next_state_index = (current_index[tl_id] + 1) % len(info['states'])
                next_state = info['states'][next_state_index]

                publish_tl_state(pub, tl_id, current_state, current_duration, next_state)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

