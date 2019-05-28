import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator
from matplotlib import style
import threading, time

#########################################################################################
#                       Live Graph Plotting - By: Greg Balke                            #
#########################################################################################
# This graph spawns a secondary thread to update your data while the main thread plots  #
# Pressing 'p' during operation will toggle the plotting and display the entire dataset #
# Arguments:                                                                            #
#       update_func: user defined function called at the update interval                #
#                           Expect data to look like (x, [y1, y2, ..., yn])             #
#       labels: labels matching the dim of the y axis of returned data                  #
#       sample_interval: interval at which the update function is run (in milliseconds) #
#       plot_interval: interval at which the plot is updated (in milliseconds)          #
#       window_size: number of samples to show when plotting is not paused              #
#########################################################################################
class livegraph():
    def __init__(self, update_func, labels, sample_interval=1, plot_interval=100, window_size=1000):
        self.labels = labels
        self.sample_interval = sample_interval/1000.0
        self.plot_interval = plot_interval
        self.window_size = window_size
        self.update_func = update_func
        
        self.data = livegraph.data_object()
        self.graph = livegraph.live_graph(self.data, self.labels, interval=self.plot_interval, window_size=self.window_size)
        self.update_thread = livegraph.data_update(self.data, self.update_func, interval=self.sample_interval)

    def start(self):
        self.update_thread.start()
        plt.show()
        
        self.data.running = False
        self.update_thread.join()

    # Sub class to manage running the graph animation update interface
    class live_graph():
        def __init__(self, data, labels, interval=10, window_size=100):
            self.data = data
            self.num_items = len(labels)
            self.labels=labels
            self.window_size = window_size
    
            style.use('fivethirtyeight')
            
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(1,1,1)
    
            self.pause = False
            self.drawn = False
    
            self.fig.canvas.mpl_connect('key_press_event', self.onPress)
            self.ani = animation.FuncAnimation(self.fig, self.redraw, interval=interval, repeat=True)
    
        def redraw(self, i):
            # When paused, draw entire plot
            if self.pause:
                ys = self.data.ys
                xs = self.data.xs
            # When not paused, draw only the window
            else:
                ys = self.data.ys[-self.window_size:]
                xs = self.data.xs[-self.window_size:]
                            
            if not self.drawn:
                self.ax.clear()
                self.ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        
                for item in range(self.num_items):
                    dat = [y[item] for y in ys]
                    self.ax.plot(xs, dat, label=self.labels[item])
                
                self.ax.legend(loc='upper left')
    
            if self.pause:
                self.drawn = True
            else:
                self.drawn = False
    
        def onPress(self, event):
            if event.key == 'p':
                self.pause ^= True
    
    # Object to handle the coordinate data such that it is passed by reference to
    #   the update and graph sub-classes.
    # This also monitors the state of the graph such that when the graph closes,
    #   the updater stops.
    class data_object():
        def __init__(self):
            self.xs = []
            self.ys = []
            self.running = True
    
    # Thread class to handle updating user data from the referenced function!
    class data_update(threading.Thread):
        def __init__(self, data, update_func, interval=1):
            threading.Thread.__init__(self)
    
            self.data = data
            self.interval = interval
            self.nextCall = time.time()
            self.update_func = update_func
    
        def run(self):
            count = 0
            while self.data.running:
                x, y = self.update_func(count) 
                self.data.xs.append(x)
                self.data.ys.append(y)
                self.nextCall = self.nextCall + self.interval
                time.sleep(max(0, self.nextCall - time.time()))
                count += 1
