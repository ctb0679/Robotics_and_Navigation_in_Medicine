import shelve
import rnm

# Open the .shelve file in read mode
shelf = shelve.open('/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording/session.shelve')

# Access the contents of the .shelve file
keys = list(shelf.keys())  # Get a list of all keys in the dictionary
values = [shelf[key] for key in keys]  # Get the corresponding values for each key

# Process the contents as needed
for key, value in zip(keys, values):
    print("Key:", key)
    print("Value:", value)
    print()

# Close the .shelve file
shelf.close()

