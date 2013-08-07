import traceback, sys, signal, time
import paramiko
import ConfigParser

class Remote(object):
    def __init__(self, output):
        self.output = output
        self.client = None
        self.throttle1_shell = None
        self.throttle2_shell = None
        self.throttle3_shell = None
        self.throttle4_shell = None
        self.throttle5_shell = None
        self.throttle6_shell = None
        self.throttle7_shell = None
        self.cc_shell = None
        self.saveData_shell = None
        self.isConnected = False
        self.landsharkRunning = False
        self.ccRunning = False
        self.saveDataRunning = False

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
            config.read('hacms.cfg')
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
            	self.black_shell = self.client.invoke_shell()
            	self.throttle1_shell = self.client.invoke_shell()
            	self.throttle2_shell = self.client.invoke_shell()
            	self.throttle3_shell = self.client.invoke_shell()
            	self.throttle4_shell = self.client.invoke_shell()
            	self.throttle5_shell = self.client.invoke_shell()
            	self.throttle6_shell = self.client.invoke_shell()
            	self.throttle7_shell = self.client.invoke_shell()
                self.black_shell.send('source ~/.bashrc\nroslaunch landshark_launch black_box.launch\n')
                time.sleep(2) # Sleep for a bit while the ROS master node is starting up...
                self.throttle1_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark_control/base_velocity 2 /landshark_demo/base_vel\n')
                self.throttle2_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark_control/reference_velocity 2 /landshark_demo/ref_vel\n')
                self.throttle3_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark_control/estimated_velocity 2 /landshark_demo/est_vel\n')
                self.throttle4_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark/odom 2 /landshark_demo/odom\n')
                self.throttle5_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark/left_encoder_velocity 2 /landshark_demo/left_enc_vel\n')
                self.throttle6_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark/right_encoder_velocity 2 /landshark_demo/right_enc_vel\n')
                self.throttle7_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark/gps_velocity 2 /landshark_demo/gps_vel\n')
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
                self.black_shell.close()
                self.throttle1_shell.close()
                self.throttle2_shell.close()
                self.throttle3_shell.close()
                self.throttle4_shell.close()
                self.throttle5_shell.close()
                self.throttle6_shell.close()
                self.throttle7_shell.close()
                self.output.appendPlainText('*** Stopped Landshark.')
                self.landsharkRunning = False
            except:
                pass

        return self.landsharkRunning

    def startCC(self):
        if self.ccRunning:
            return True
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False

        try:
            self.output.appendPlainText('*** Starting Cruise Controller...')
            self.cc_shell = self.client.invoke_shell()
            self.cc_shell.send('source ~/.bashrc\nroslaunch Controller controller.launch\n')
            time.sleep(2) # Sleep for a bit while the Controller is starting up...
            self.output.appendPlainText('*** Started Cruise Controller.')
            self.ccRunning = True
        except:
            pass

        return self.ccRunning

    def stopCC(self):
        if not self.ccRunning:
            return False
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False

        try:
            self.output.appendPlainText('*** Stopping Cruise Controller...')
            self.cc_shell.close()
            self.output.appendPlainText('*** Stopped Cruise Controller.')
            self.ccRunning = False
        except:
            pass

        return self.ccRunning
        
    def startSaveData(self):
        if self.saveDataRunning:
            return True
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.ccRunning:
            self.output.appendPlainText('*** You must first start Cruise Control.')
            return False

        try:
            self.output.appendPlainText('*** Starting Save Data...')
            self.saveData_shell = self.client.invoke_shell()
            self.saveData_shell.send('source ~/.bashrc\nroslaunch log_data_demo.launch\n')
            self.output.appendPlainText('*** Started Save Data.')
            self.saveDataRunning = True
        except:
            pass

        return self.saveDataRunning
        
    def stopSaveData(self):
        if not self.saveDataRunning:
            return False
        if not self.isConnected:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.landsharkRunning:
            self.output.appendPlainText('*** You must first start Landshark.')
            return False
        if not self.ccRunning:
            self.output.appendPlainText('*** You must first start Cruise Control.')
            return False

        try:
            self.output.appendPlainText('*** Stopping Save Data...')
            self.saveData_shell.close()
            self.output.appendPlainText('*** Stopped Save Data.')
            self.saveDataRunning = False
        except:
            pass

        return self.saveDataRunning
        

    def writeLinesToOutput(self, lines):
        for line in lines:
            self.output.appendPlainText(line)

