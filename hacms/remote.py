import paramiko
import ConfigParser

class Remote(object):
    def __init__(self):
        self.channel = None
    
    def connect(self):
        # Connect and use paramiko Client to negotiate SSH2 across the connection
        try:
            client = paramiko.SSHClient()
            client.load_system_host_keys()
            client.set_missing_host_key_policy(paramiko.WarningPolicy)
            print '*** Connecting...'
            config = ConfigParser.SafeConfigParser()
            config.read('ssh.cfg')
            client.connect(config.get('SSH','hostname'), int(config.get('SSH','port')), config.get('SSH','username'), config.get('SSH','password'))
            chan = client.invoke_shell()
            print repr(client.get_transport())
            print '*** Here we go!'
            print
            #interactive.interactive_shell(chan)
            chan.close()
            client.close()

        except Exception, e:
            print '*** Caught exception: %s: %s' % (e.__class__, e)
            traceback.print_exc()
            try:
                client.close()
            except:
                pass
            sys.exit(1)
            
    def startROS(self):
        return