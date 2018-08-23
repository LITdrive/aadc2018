import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import pandas as pd

# CONFIG this Suff here ;) ######################################
MAP_LENGTH = 5.020 #in m
MAP_HIGHT = 4.020 #in m
mapImg = 'ourMap.jpg'
#################################################################

points = []

# function to draw lines - from matplotlib examples.  Note you don't need
# to keep a reference to the lines drawn, so I've removed the class as it
# is overkill for your purposes
def draw_line(startx,starty):
        ax = plt.gca()
        #xy = plt.ginput(1)
        #draw_point(xy[0][0],xy[0][1])
        if len(points)>1:
            x = [startx,points[-2][0]]
            y = [starty,points[-2][1]]
            line = ax.plot(x,y,c='b', picker=5) # note that picker=5 means a click within 5 pixels will "pick" the Line2D object
        ax.figure.canvas.draw()     

def draw_point(x,y):
    if (x>0)and (y>0) and (x <= MAP_LENGTH) and (y <= MAP_HIGHT	):
         points.append([x,y])
    poi = ax.plot(x,y,'.y')
    txt = ax.text(x+MAP_LENGTH*0.005,y,str(len(points)),color='r')
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
            draw_point(event.xdata,event.ydata)
            draw_line(event.xdata,event.ydata) # here you click on the plot
        else:
            pass # Do nothing

def on_key(event):
    """
    Function to be bound to the key press event
    """
    if event.key == 's':
        print('Saving points')
        print(points)
        df = pd.DataFrame()
        df['x']=[p[0] for p in points]
        df['y']=[p[1] for p in points]
        df.to_csv('points.csv',header=None,index=True,sep=',') 

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

cid = fig.canvas.mpl_connect('key_press_event', on_key)

#set the size of the matplotlib figure in data units, so that it doesn't
#auto-resize (which it will be default on the first drawn item)
ax.set_xlim([0,MAP_LENGTH])
ax.set_ylim([0,MAP_HIGHT])
ax.aspect = 1
plt.tight_layout()

plt.show()