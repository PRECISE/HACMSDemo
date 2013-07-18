from PyQt4.QtCore import *
from PyQt4.QtWebKit import *

class MapView(QWebView):
    def __init__(self, parent=None): 
        super(MapView, self).__init__(parent)
        self.setHtml(open('map.html').read())
#         config = ConfigParser.SafeConfigParser()
#         config.read('hacms.cfg')
#         self.api_key=config.get('MAP','api_key')

