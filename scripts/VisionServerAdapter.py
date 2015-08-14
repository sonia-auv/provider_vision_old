
import rospy
import Util.logger
import sonia_msgs.srv

class VisionServerAdapter():

    def __init__(self):
        self.SAVE_FILTERCHAIN = 1
        self.RESTORE_FILTERCHAIN = 2

        self.ADD_FILTERCHAIN_FILTER = 1
        self.DELETE_FILTERCHAIN_FILTER = 2

        self.ADD_FILTERCHAIN = 1
        self.DELETE_FILTERCHAIN = 2

        self.START_EXECUTION = 1
        self.STOP_EXECUTION = 2

        self.EXECUTION_LIST_ID = 1
        self.MEDIA_LIST_ID = 2
        self.FILTERCHAIN_LIST_ID = 3
        self.FILTERS_LIST_ID = 4

        self.log = Util.logger.log()
        self.initServices()

    def initServices(self):

        self.log.info("Initiating services")
        try:
            self.get_information_list = rospy.ServiceProxy('/vision_server/vision_server_get_information_list', sonia_msgs.srv.vision_server_get_information_list)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.copy_filterchain = rospy.ServiceProxy('/vision_server/vision_server_copy_filterchain', sonia_msgs.srv.vision_server_copy_filterchain)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.execute_cmd = rospy.ServiceProxy('/vision_server/vision_server_execute_cmd', sonia_msgs.srv.vision_server_execute_cmd)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_filterchain_filter = rospy.ServiceProxy('/vision_server/vision_server_get_filterchain_filter', sonia_msgs.srv.vision_server_get_filterchain_filter)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_filterchain_filter_param = rospy.ServiceProxy('/vision_server/vision_server_get_filterchain_filter_param',sonia_msgs.srv.vision_server_get_filterchain_filter_param)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_media_param = rospy.ServiceProxy('/vision_server/vision_server_get_media_param',sonia_msgs.srv.vision_server_get_media_param)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_media_param_list = rospy.ServiceProxy('/vision_server/vision_server_get_media_param_list',sonia_msgs.srv.vision_server_get_media_param_list)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.manage_filterchain = rospy.ServiceProxy('/vision_server/vision_server_manage_filterchain',sonia_msgs.srv.vision_server_manage_filterchain)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.manage_filterchain_filter = rospy.ServiceProxy('/vision_server/vision_server_manage_filterchain_filter',sonia_msgs.srv.vision_server_manage_filterchain_filter)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.save_filterchain = rospy.ServiceProxy('/vision_server/vision_server_save_filterchain', sonia_msgs.srv.vision_server_save_filterchain)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.set_filterchain_filter_order = rospy.ServiceProxy('/vision_server/vision_server_set_filterchain_filter_order', sonia_msgs.srv.vision_server_set_filterchain_filter_order)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.set_filterchain_filter_param = rospy.ServiceProxy('/vision_server/vision_server_set_filterchain_filter_param', sonia_msgs.srv.vision_server_set_filterchain_filter_param)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.set_media_param = rospy.ServiceProxy('/vision_server/vision_server_set_media_param', sonia_msgs.srv.vision_server_set_media_param)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_filterchain_filter_all_param = rospy.ServiceProxy('/vision_server/vision_server_get_filterchain_filter_all_param',sonia_msgs.srv.vision_server_get_filterchain_filter_all_param)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_filterchain_from_execution = rospy.ServiceProxy('/vision_server/vision_server_get_filterchain_from_execution', sonia_msgs.srv.vision_server_get_filterchain_from_execution)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        try:
            self.get_media_from_execution = rospy.ServiceProxy('/vision_server/vision_server_get_media_from_execution', sonia_msgs.srv.vision_server_get_media_from_execution)
        except rospy.ServiceException, e:
            self.log.error("initServices")
            self.log.error(e.message)

        self.log.info("Services are initiated")


    def getLogger(self):
        return self.log

    def getMediaFromExecution(self, exec_name):
        try:
            return self.get_media_from_execution(exec_name)
        except Exception, e:
            self.log.error("getMediaFromExecution")
            self.log.error(e.message)

    def getFilterchainFromExecution(self, exec_name):
        try:
            return self.get_filterchain_from_execution(exec_name)
        except Exception, e:
            self.log.error("getFilterchainFromExecution")
            self.log.error(e.message)

    def getFilterchainFilterParams(self, exec_name, filterchain, filter):
        try:
            return self.get_filterchain_filter_all_param(exec_name,filterchain,filter)
        except Exception, e:
            self.log.error("getFilterchainFilterParams")
            self.log.error(e.message)

    def getMediaList(self):
        try:
            return self.get_information_list(self.MEDIA_LIST_ID)
        except Exception, e:
            self.log.error("getMediaList")
            self.log.error(e.message)

    def getExecutionList(self):
        try:
            return self.get_information_list(self.EXECUTION_LIST_ID)
        except Exception, e:
            self.log.error("getExecutionList")
            self.log.error(e.message)

    def getFilterchainList(self):
        try:
            return self.get_information_list(self.FILTERCHAIN_LIST_ID)
        except Exception, e:
            self.log.error("getFitlerchainList")
            self.log.error(e.message)

    def getFilterList(self):
        try:
            return self.get_information_list(self.FILTERS_LIST_ID)
        except Exception, e:
            self.log.error("getFilterList")
            self.log.error(e.message)

    def copyFilterchain(self,filterchain_to_copy, filterchain_new_name):
        try:
            return self.copy_filterchain(filterchain_to_copy, filterchain_new_name)
        except Exception, e:
            self.log.error("copyFilterchain")
            self.log.error(e.message)

    def startMedia(self, nodeName, filterchainName, mediaName):
        try:
            return self.execute_cmd(nodeName,filterchainName,mediaName,self.START_EXECUTION)
        except Exception, e:
            self.log.error("startMedia")
            self.log.error(e.message)

    def stopMedia(self, nodeName, filterchainName, mediaName):
        try:
            return self.execute_cmd(nodeName,filterchainName,mediaName,self.STOP_EXECUTION)
        except Exception, e:
            self.log.error("stopMedia")
            self.log.error(e.message)

    def getFilterchainFilter(self, exec_name, filterchain):
        try:
            return self.get_filterchain_filter(exec_name,filterchain)
        except Exception, e:
            self.log.error("getFilterchainFilter")
            self.log.error(e.message)

    def getFilterchainFilterParam(self, exec_name, filterchain, filter):
        try:
            return self.get_filterchain_filter_param(exec_name, filterchain, filter)
        except Exception, e:
            self.log.error("getFilterchainFilterParam")
            self.log.error(e.message)

    def getMediaParam(self, media_name, param_name):
        try:
            return self.get_media_param(media_name, param_name)
        except Exception, e:
            self.log.error("getMediaParam")
            self.log.error(e.message)

    def getMediaParamList(self, media_name):
        try:
            return self.get_media_param_list(media_name)
        except Exception, e:
            self.log.error("getMediaParamList")
            self.log.error(e.message)

    def addFilterchain(self, filterchain):
        try:
            return self.manage_filterchain(filterchain, self.ADD_FILTERCHAIN)
        except Exception, e:
            self.log.error("addFilterchain")
            self.log.error(e.message)

    def deleteFilterchain(self, filterchain):
        try:
            return self.manage_filterchain(filterchain, self.DELETE_FILTERCHAIN)
        except Exception, e:
            self.log.error("deleteFilterchain")
            self.log.error(e.message)

    def addFilterchainFilter(self, exec_name, filterchain, filter):
        try:
            return self.manage_filterchain_filter(exec_name, filterchain, filter, self.ADD_FILTERCHAIN_FILTER)
        except Exception, e:
            self.log.error("addFilterchainFilter")
            self.log.error(e.message)

    def deleteFilterchainFilter(self, exec_name, filterchain, filter):
        try:
            return self.manage_filterchain_filter(exec_name, filterchain, filter, self.DELETE_FILTERCHAIN_FILTER)
        except Exception, e:
            self.log.error("addFilterchainFilter")
            self.log.error(e.message)

    def saveFilterchain(self, exec_name, filterchain):
        try:
            return self.save_filterchain(exec_name, filterchain, self.SAVE_FILTERCHAIN)
        except Exception, e:
            self.log.error("saveFilterchain")
            self.log.error(e.message)

    def restoreFilterchain(self, exec_name, filterchain):
        try:
            return self.save_filterchain(exec_name, filterchain, self.RESTORE_FILTERCHAIN)
        except Exception, e:
            self.log.error("restoreFilterchain")
            self.log.error(e.message)

    def setFilterchainFilterOrder(self, exec_name, filterchain, orderlist):
        try:
            return self.set_filterchain_filter_order(exec_name,filterchain,orderlist)
        except Exception, e:
            self.log.error("setFilterchainFilterOrder")
            self.log.error(e.message)

    def setFilterchainFilterParam(self, exec_name, filterchain, filter, parameter, value):
        try:
            return self.set_filterchain_filter_param(exec_name, filterchain, filter, parameter, value)
        except Exception, e:
            self.log.error("setFilterchainFilterParam")
            self.log.error(e.message)

    def setMediaParam(self, media_name, param_name, value):
        try:
            return self.set_media_param(media_name, param_name, value)
        except Exception, e:
            self.log.error("setMediaParam")
            self.log.error(e.message)