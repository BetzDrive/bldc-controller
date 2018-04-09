import matplotlib.pyplot as plt
import pickle
import sys

num_channels = 9
if len(sys.argv) < 2:
    print("give me a data file")
    exit(0)

data = []
with open(sys.argv[1], 'rb') as file:
    data = pickle.load(file)

length = len(data) // num_channels

ia = [data[i * num_channels] for i in range(length)]
ib = [data[i * num_channels + 1] for i in range(length)]
ic = [data[i * num_channels + 2] for i in range(length)]
va = [data[i * num_channels + 3] for i in range(length)]
vb = [data[i * num_channels + 4] for i in range(length)]
vc = [data[i * num_channels + 5] for i in range(length)]
vin = [data[i * num_channels + 6] for i in range(length)]
angle = [data[i * num_channels + 7] for i in range(length)]
vel = [data[i * num_channels + 8] for i in range(length)]

plt.plot(ia)
plt.xlabel('sample')
plt.ylabel('Current A')
plt.show()

plt.plot(ib)
plt.xlabel('sample')
plt.ylabel('Current B')
plt.show()

plt.plot(ic)
plt.xlabel('sample')
plt.ylabel('Current C')
plt.show()

plt.plot(va)
plt.xlabel('sample')
plt.ylabel('Voltage A')
plt.show()

plt.plot(vb)
plt.xlabel('sample')
plt.ylabel('Voltage B')
plt.show()

plt.plot(vc)
plt.xlabel('sample')
plt.ylabel('Voltage C')
plt.show()

plt.plot(vin)
plt.xlabel('sample')
plt.ylabel('Vin')
plt.show()

plt.plot(angle)
plt.xlabel('sample')
plt.ylabel('Encoder Angle')
plt.show()

plt.plot(vel)
plt.xlabel('sample')
plt.ylabel('Encoder Velocity')
plt.show()
