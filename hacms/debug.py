import hacms
import ui.debug_ui

class HACMSDebugWindow(hacms.HACMSWindow):
    def __init__(self):
        super(HACMSDebugWindow, self).__init__()
        self.ui = ui.debug_ui.Ui_MainWindow()
        self.init_window()
        self.widgets.append(
            self.ui.trimPlot,
            self.ui.trimPlotLabel)
        
    def init_plots(self):
        super(HACMSDebugWindow, self).init_plots()
        self.trimFig = Figure((3.31, 2.01), dpi=self.dpi)
        self.trimCanvas = FigureCanvas(self.trimFig)
        self.trimCanvas.setParent(self.ui.trimPlot)
        self.trimAxes = self.trimFig.add_subplot(111)
        self.trimAxes.grid(True)
        self.trimAxes.set_title('Trim')
            
    def draw_trimPlot(self):
        """ Redraws the trim plot
        """
        # clear the axes and redraw the plot anew
        #
        self.trimAxes.clear()
        self.trimAxes.grid(True)
        self.trimAxes.set_title('Trim')
        self.trimAxes.plot(self.out_Trim)
        self.trimCanvas.draw()
