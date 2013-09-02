#!/usr/bin/env python

import hacms
import ui.demo_ui
     
class HACMSDemoWindow(hacms.HACMSWindow):
    def __init__(self):
        super(HACMSDemoWindow, self).__init__()
        self.ui = ui.demo_ui.Ui_MainWindow()
        self.init_window()
        #self.widgets.append
