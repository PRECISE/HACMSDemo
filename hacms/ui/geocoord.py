from PyQt4.QtGui import *
import ui.geocoord_ui

class GeoCoordDialog(QDialog):
    def __init__(self, parent = None):
        super(GeoCoordDialog, self).__init__(parent)
        self.ui = ui.geocoord_ui.Ui_GeoCoordinateDialog()
        self.ui.setupUi(self)
        
        # Set Validator for parameter fields
        self.validator = QDoubleValidator()
        self.validator.setNotation(QDoubleValidator.StandardNotation)
        self.ui.latitudeLineEdit.setValidator(self.validator)
        self.ui.longitudeLineEdit.setValidator(self.validator)

    def latitude(self):
        return float(self.ui.latitudeLineEdit.text())
        
    def longitude(self):
        return float(self.ui.longitudeLineEdit.text())

    # static method to create the dialog and return (lat, long, accepted)
    @staticmethod
    def getLatLong(parent = None):
        dialog = GeoCoordDialog(parent)
        result = dialog.exec_()
        return (dialog.latitude(), dialog.longitude(), result == QDialog.Accepted)

