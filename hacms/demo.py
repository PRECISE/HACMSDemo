#!/usr/bin/env python

# HACMS modules
import hacms
import ui.demo_ui

        #TODO: Try to look into flushing queue as it gets backlogged (CPU overloaded...)
        #TODO: Layout widgets so that the console and plots will resize with the window
        #TODO: Add legend for plot lines, fit titles and axes labels, too
        #TODO: Use timestamps for x-axis, data will be plotted accordingly
        #TODO: Add navigation tab with Google Maps
        #TODO: Combine three plots into a single plot (using subplots)?
        #TODO: Change big buttons to be darker or colored when checked
        #TODO: Put all main widgets into a list
        #TODO: put red line to mark attack moment or attack region
        #TODO: Make separate debug and demo UIs (make parent window class, subclass it for each type)
        #TODO: Ability to save plots showing the entire data set, save data in Matlab-friendly format (write to a file in real-time? file name and location?)
        #TODO: Ability to save/transfer .bag files (for video)
        #TODO: Fix plots so that the titles and axes labels are shown completely
     
class HACMSDemoWindow(hacms.HACMSWindow):
    def __init__(self):
        super(HACMSDemoWindow, self).__init__()
        self.ui = ui.demo_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.init_window()

