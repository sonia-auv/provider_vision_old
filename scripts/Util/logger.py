import os
import time
import sys

class log():

    def __init__(self):
        self.HEADER = '\033[95m'
        self.OKGREEN = '\033[92m'
        self.WARNING = '\033[93m'
        self.FAIL = '\033[91m'
        self.ENDC = '\033[0m'
        self.BOLD = "\033[1m"
        self.isQuiet = False
        self.log = None
        print os.getcwd()+"/TestResult_"+str(time.time())+".log"
        try:
            try:
                os.makedirs(os.getcwd()+"/log/")
            except:
                pass
            self.log = open(os.getcwd()+"/log/TestResult_"+str(time.time())+".log", 'w')
        except Exception, e:
            print "Cannot open log file..."
            print e.message


    def quiet(self):
        self.isQuiet = True
    def verbose(self):
        self.isQuiet = False

    def info(self,msg):
        if self.log :
            self.log.write("INFO <"+str(time.time())+"> :: "+msg+"\n")
        if not self.isQuiet:
            print self.OKGREEN + ("INFO <"+str(time.time())+"> :: "+msg) + self.ENDC

    def warn(self,msg):
        if self.log :
            self.log.write("WARN <"+str(time.time())+"> :: "+msg+"\n")
        if not self.isQuiet:
            print self.WARNING + ("WARN <"+str(time.time())+"> :: "+msg) + self.ENDC

    def error(self,msg):
        if self.log :
            self.log.write("ERROR <"+str(time.time())+"> :: "+msg+"\n")
        if not self.isQuiet:
            print self.FAIL + ("ERROR <"+str(time.time())+"> :: "+msg) + self.ENDC