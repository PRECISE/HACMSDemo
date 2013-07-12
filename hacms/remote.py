import traceback, sys, signal
import paramiko
import ConfigParser

class Remote(object):
    def __init__(self, output):
        self.output = output
        self.client = None
        self.main_shell = None
        self.throttle1_shell = None
        self.throttle2_shell = None
        self.cc_shell = None
        self.isConnected = False
        self.landsharkRunning = False
        self.ccRunning = False

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
            self.main_shell = self.client.invoke_shell()
            self.black_shell = self.client.invoke_shell()
            self.throttle1_shell = self.client.invoke_shell()
            self.throttle2_shell = self.client.invoke_shell()
            self.cc_shell = self.client.invoke_shell()
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
                self.black_shell.send('source ~/.bashrc\nroslaunch landshark_launch black_box.launch\n')
                self.throttle1_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark/odom 2 /landshark_demo/odom\n')
                self.throttle2_shell.send('source ~/.bashrc\nrosrun topic_tools throttle messages /landshark/gps_velocity 2 /landshark_demo/gps_velocity\n')
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
                #TODO: These don't seem to work remotely!
                self.client.exec_command("ps ax | awk '/roslaunch landshark_launch black_box.launch/ {print $1}' | xargs kill -2\n")
                self.client.exec_command("ps ax | awk '/messages \/landshark\/odom 2 \/landshark_demo\/odom/ {print $1}' | xargs kill -2\n")
                self.client.exec_command("ps ax | awk '/messages \/landshark\/gps_velocity 2 \/landshark_demo\/gps_velocity/ {print $1}' | xargs kill -2\n")
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
            #self.cc_shell.send('source ~/.bashrc\nroslaunch Controller controller.launch\n')
            self.cc_shell.send('source ~/.bashrc\nrosrun landshark_cruise landshark_cruise\n')
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
            #self.client.exec_command("ps ax | awk '/roslaunch Controller controller.launch/ {print $1}' | xargs kill -2\n")
            self.client.exec_command("ps ax | awk '/rosrun landshark_cruise landshark_cruise/ {print $1}' | xargs kill -2\n")
            self.output.appendPlainText('*** Stopped Cruise Controller.')
            self.ccRunning = False
        except:
            pass

        return self.ccRunning

    def writeLinesToOutput(self, lines):
        for line in lines:
            self.output.appendPlainText(line)

