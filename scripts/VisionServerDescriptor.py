


class VisionServerDescriptor(object):
    def __init__(self, logger):

        self._logger = logger

        #### EXISTING VALUES ####
        self._executionsReal = []
        self._filtersReal = []
        self._filterchainsReal = []
        self._mediasReal = []
        #### FAKE VALUES ####
        self._executionsFake = []
        self._filtersFake = []
        self._filterchainsFake = []
        self._mediasFake = []
        self._paramsFake = []

    def generateFakeValues(self):
        self._logger.info("Generating fake values inside VisionServerDescriptor...")

        # Params
        self._logger.info("Params...")
        self._paramsFake.append(self.Param(name="Dolphin", value=2015))
        self._paramsFake.append(self.Param(name="Starfish", value="Seaturtle"))
        self._paramsFake.append(self.Param(name="\"/$%?&*()~{­¶[]½¾¼¢³±£¤³±¼", value="int"))
        self._paramsFake.append(self.Param(name="Blur", value="ok"))
        self._paramsFake.append(self.Param(name="shouldFail", value="\"/$%?&*()~{­¶[]½¾¼¢³±£¤³±¼"))
        self._logger.info("OK")

        # Filters
        self._logger.info("Filters...")
        self._filtersFake.append(self.Filter(name="AFakeFilter"))
        self._filtersFake.append(self.Filter(name="AnotherFakeFilter"))
        for filterFake in self._filtersFake:
            filterFake._params = self._paramsFake
        self._logger.info("OK")

        # Filterchains
        self._logger.info("Filterchains...")
        self._filterchainsFake.append(self.Filterchain(name="AFakeFilterchain"))
        self._filterchainsFake.append(self.Filterchain(name="AnotherFakeFilterchain"))
        for filterchainFake in self._filterchainsFake:
            filterchainFake._filters = self._filtersFake
            filterchainFake._params = self._paramsFake
        self._logger.info("OK")

        # Medias
        self._logger.info("Medias...")
        self._mediasFake.append(self.Media(name="AFakeMedia"))
        self._mediasFake.append(self.Media(name="AnotherFakeMedia"))
        for mediaFake in self._mediasFake:
            mediaFake._params = self._paramsFake
        self._logger.info("OK")

        # Executions
        self._logger.info("Executions...")
        self._executionsFake.append(self.DetectionTask(name="AFakeExecution"))
        self._executionsFake.append(self.DetectionTask(name="AnotherFakeExecution"))
        for i in range(len(self._executionsFake)):
            if i < len(self._mediasFake):
                self._executionsFake[i]._media = self._mediasFake[i]
            else :
                self._executionsFake[i]._media = self._mediasFake[len(self._mediasFake)-1]
            if i < len(self._filterchainsFake):
                self._executionsFake[i]._filterchain = self._filterchainsFake[i]
            else :
                self._executionsFake[i]._filterchain = self._filterchainsFake[len(self._filterchainsFake)-1]
        self._logger.info("OK")

        self._logger.info("Generation of fake values done...")



    #####################################################
    #
    # Inner descriptive classes
    #
    class DetectionTask(object):
        def __init__(self, name):
            self._name = name
            self._media = None
            self._filterchain = None

    class Media(object):
        def __init__(self, name):
            self._name = name
            self._params = []

    class Filterchain(object):
        def __init__(self, name):
            self._name = name
            self._params = []
            self._filters = []

    class Filter(object):
        def __init__(self, name):
            self._name = name
            self._params = []

    class Param(object):
        def __init__(self, name, value):
            self._name = name
            self._value = value
    #
    #
    #####################################################
