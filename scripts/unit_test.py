#!/usr/bin/env python

import VisionServerAdapter
import sys
import time

def main():

    visionServer = VisionServerAdapter.VisionServerAdapter()
    logger = visionServer.getLogger()

    logger.info("Starting Vision Server Unit Testing Sequence")

    logger.info("Getting media list")
    mediaList = (str(visionServer.getMediaList()).split(':'))[1].split(';')

    for i in range(0,len(mediaList)):
        mediaList[i] = mediaList[i].strip()

    logger.info("Discovered " + str(len(mediaList)-1) + " medias on the server...")
    time.sleep(1)
    for i in range(0,len(mediaList)):
        print mediaList[i]
    time.sleep(2)

    for i in range(0,5):

        logger.info("Starting media " + mediaList[0] + " sequence : " + str(i))
        executionName = visionServer.startMedia(nodeName=mediaList[0],filterchainName="None",mediaName="/home/mojo/robosubFinal.mp4").response

        print executionName
        time.sleep(2)

        logger.info("Stopping media " + mediaList[0])
        visionServer.stopMedia(nodeName=executionName,filterchainName="None",mediaName=mediaList[0])

if __name__ == "__main__":
    sys.exit(main())




