import time
from io_port_toggle import TimeKeeper

# Start the timer
TimeKeeper.start()
print("Started the timer.")

# Wait for 5 seconds
time.sleep(5)

# Stop the timer
TimeKeeper.stop()
print("Stopped the timer.")

# Print the total time
print(f"Total time: {TimeKeeper.get_total_time()} seconds")