import traceback, sys
import paramiko
import ConfigParser

class Remote(object):
    def __init__(self, output):
        self.output = output
        self.client = None
        self.isConnected = False
        self.landsharkRunning = False
        self.rcRunning = False
        self.attackRunning = False
    
    def connect(self):
        if self.isConnected:
            return True
        
        # Connect and use paramiko Client to negotiate SSH2 across the connection
        try:
            self.output.appendPlainText('*** Connecting...')
            self.client = paramiko.SSHClient()
            self.client.load_system_host_keys()
            self.client.set_missing_host_key_policy(paramiko.WarningPolicy())
            config = ConfigParser.SafeConfigParser()
            config.read('ssh.cfg')
            self.client.connect(config.get('SSH','hostname'), int(config.get('SSH','port')), config.get('SSH','username'), config.get('SSH','password'))
            self.isConnected = True
            self.output.appendPlainText('*** Connected!')

        except Exception, e:
            self.output.appendPlainText('*** Caught exception: %s: %s' % (e.__class__, e))
            traceback.print_exc()
            try:
                self.client.close()
            except:
                pass
            self.isConnected = False
        
        return self.isConnected
            
    def startLandshark(self):
        if self.landsharkRunning:
            return True        
        
        if self.connect():
            try:
                self.output.appendPlainText('*** Starting Landshark...')
                self.client.exec_command('roslaunch landshark_launch black_box.launch &')
                self.output.appendPlainText('*** Started Landshark.')
                self.landsharkRunning = True
            except:
                pass
        
        return self.landsharkRunning
        
    def stopLandshark(self):
        if not self.landsharkRunning:
            return False
        
        if self.connect():
            try:
                self.output.appendPlainText('*** Stopping Landshark...')
                self.client.exec_command('killall roslaunch')
                self.output.appendPlainText('*** Stopped Landshark.')
                self.landsharkRunning = False
            except:
                pass
        
        return self.landsharkRunning
        
    def startRC(self):
        if self.rcRunning:
            return True
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        
        try:
            self.output.appendPlainText('*** Starting Resilient Controller...')
            #self.writeLinesToOutput(self.client.exec_command('ls')[1]) 
            self.rcRunning = True
        except:
            pass
        
        return self.rcRunning
        
    def stopRC(self):
        if not self.rcRunning:
            return False
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        
        try:
            self.output.appendPlainText('*** Stopping Resilient Controller...')
            #self.writeLinesToOutput(self.client.exec_command('ls')[1]) 
            self.rcRunning = False
        except:
            pass
        
        return self.rcRunning
        
    def startAttack(self):
        if self.attackRunning:
            return True
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        
        try:
            self.output.appendPlainText('*** Starting Attack...')
            #self.writeLinesToOutput(self.client.exec_command('ls')[1]) 
            self.attackRunning = True
        except:
            pass
        
        return self.attackRunning
        
    def stopAttack(self):
        if not self.attackRunning:
            return False
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        
        try:
            self.output.appendPlainText('*** Stopping Attack...')
            #self.writeLinesToOutput(self.client.exec_command('ls')[1]) 
            self.attackRunning = False
        except:
            pass
        
        return self.attackRunning
    
    def writeLinesToOutput(self, lines):
        for line in lines:
            self.output.appendPlainText(line)
        