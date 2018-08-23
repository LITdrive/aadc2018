import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import pandas as pd

# CONFIG this Suff here ;) ######################################
MAP_LENGTH = 5020 #in mm
MAP_HIGHT = 4020 #in mm
mapImg = 'ourMap.jpg'
#################################################################

points = []

# function to draw lines - from matplotlib examples.  Note you don't need
# to keep a reference to the lines drawn, so I've removed the class as it
# is overkill for your purposes
def draw_line(startx,starty):
        ax = plt.gca()
        xy = plt.ginput(1)
        x = [startx,xy[0][0]]
        y = [starty,xy[0][1]]
        line = ax.plot(x,y, picker=5) # note that picker=5 means a click within 5 pixels will "pick" the Line2D object
        #txt = ax.text(x+5,y+5,str(len(points)),color='r')
        ax.figure.canvas.draw()     

def onclick(event):
    """
    This implements click functionality.  If it's a double click do something,
    else ignore.
    Once in the double click block, if its a left click, wait for a further 
    click and draw a line between the double click co-ordinates and that click
    (using ginput(1) - the 1 means wait for one mouse input - a higher number
    is used to get multiple clicks to define a polyline)
    If the double click was a right click, draw the fixed radius circle

    """
    if event.dblclick:
        if event.button == 1:
            # Draw line
            draw_line(event.xdata,event.ydata) # here you click on the plot
            points.append([event.xdata,event.ydata])
        elif event.button == 3:
            # Write to figure
            plt.figtext(3, 8, 'boxed italics text in data coords', style='italic', bbox={'facecolor':'red', 'alpha':0.5, 'pad':10})
            circ = plt.Circle((event.xdata, event.ydata), radius=0.07, color='g', picker = True)
            ax.add_patch(circ)
            ax.figure.canvas.draw()
        else:
            pass # Do nothing


def onpick(event):    
    """
    Handles the pick event - if an object has been picked, store a
    reference to it.  We do this by simply adding a reference to it
    named 'stored_pick' to the axes object.  Note that in python we
    can dynamically add an attribute variable (stored_pick) to an 
    existing object - even one that is produced by a library as in this
    case
    """
    this_artist = event.artist #the picked object is available as event.artist
    # print(this_artist) #For debug just to show you which object is picked
    plt.gca().picked_object = this_artist

def on_key(event):
    """
    Function to be bound to the key press event
    If the key pressed is delete and there is a picked object,
    remove that object from the canvas
    """
    if event.key == u'delete':
        ax = plt.gca()
        if ax.picked_object:
            ax.picked_object.remove()
            ax.picked_object = None
            ax.figure.canvas.draw()
    if event.key == 's':
        print('Saving points')
        print(points)
        df = pd.DataFrame()
        df['x']=[p[0] for p in points]
        df['y']=[p[1] for p in points]
        df.to_csv('points.csv',header=None,index=False) 

fig, ax = plt.subplots()

img = Image.open(mapImg)
ax.imshow(np.rot90(np.rot90(img)),origin='lower', extent=[0,MAP_LENGTH,0,MAP_HIGHT])

#First we need to catch three types of event, clicks, "picks" (a specialised
#type of click to select an object on a matplotlib canvas) and key presses.
#The logic is - if it's a right double click, wait for the next click and draw
#a line, if its a right double click draw a fixed radius circle.  If it's a
#pick, store a reference to the picked item until the next keypress.  If it's
#a keypress - test if it's delete and if so, remove the picked object.
#The functions (defined above) bound to the events implement this logic
connection_id = fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('pick_event', onpick)
cid = fig.canvas.mpl_connect('key_press_event', on_key)

#set the size of the matplotlib figure in data units, so that it doesn't
#auto-resize (which it will be default on the first drawn item)
ax.set_xlim([0,MAP_LENGTH])
ax.set_ylim([0,MAP_HIGHT])
ax.aspect = 1
plt.tight_layout()

plt.show()