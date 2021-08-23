'''
Module for controlling ZWO ASI CMOS cameras, designed to mimic andor.py 
such that the two are easily interchangable for the scidar.

Check andor.py for docstrings etc.

Ollie Farley, CfAI 2019
'''
import zwoasi as asi
import numpy
import time

ASI_LIB = "/usr/local/lib/libASICamera2.so"

class defaultParams:
    '''Default parameters for ZWO cameras'''

    asiLib = ASI_LIB
    hasShutter = 0
    operatingTemp = None

    def barf(self):
        raise NotImplementedError('This function not yet implemented')

class ZWO():

    def __init__(self, params=defaultParams):
        self.params = params
        asi.init(self.params.asiLib)

        self.nCameras = asi.get_num_cameras()
        if self.nCameras == 0:
            raise Exception('No cameras found')
        
        self.cameraModels = asi.list_cameras()

        self.cameras = []
        self.cameraInfo = []
        for i in range(self.nCameras):
            c = asi.Camera(i)
            self.cameras.append(c)
            self.cameraInfo.append(c.get_camera_property())

        if self.params.operatingTemp!=None:
            self.cooler_on(self.params.operatingTemp)
        
        if self.params.hasShutter!=0:
            self.shutter_open()

    def shutter_open(self):
        raise NotImplementedError('This function not yet implemented')

    def shutter_close(self):
        raise NotImplementedError('This function not yet implemented')

    def cooler_on(self, temp):
        for c in self.cameras:
            try:
                c.set_control_value(asi.ASI_COOLER_ON, 1)
                c.set_control_value(asi.ASI_TARGET_TEMP, temp)
            except Exception as e:
                self.handle_error(e)
        
    def cooler_off(self):
        for c in self.cameras:
            try:
                c.set_control_value(asi.ASI_COOLER_ON, 0)
            except Exception as e:
                self.handle_error(e)
    
    def set_temp(self, temp):
        for c in cameras:
            try:
                c.set_control_value(asi.ASI_TARGET_TEMP, temp)
            except Exception as e:
                self.handle_error(e)

    def temp(self, internal=0):
        if self.nCameras==1:
            return self.cameras[0].get_control_value(asi.ASI_TEMPERATURE)[0]/10.
        else:
            return [c.get_control_value(asi.ASI_TEMPERATURE)[0]/10. for c in self.cameras]

    def shutdown(self, safe_temp=0.):
        # Note we ignore safe_temp for now
        for c in self.cameras:
            try:
                print('Shutting down cameras')
                c.close()
            except Exception as e:
                self.handle_error(e)

    def setup_common(self, roiList):
        # Note that we can't bin separately in x and y as for andor cameras, so we use 
        # only the x value
        try:
            if roiList != None:
                if len(roiList) == self.nCameras:
                    for iroi in range(len(roiList)):
                        xBin,yBin,xLo,xHi,yLo,yHi=roiList[iroi]


                        if (xHi - xLo + 1) % xBin == 0:
                            self.xDim = (xHi - xLo + 1)/xBin                      # x-dimension
                        else:
                            raise 'ROI x-dimension must be a multiple of x binning value'
                        if (yHi - yLo + 1) % yBin == 0:
                            self.yDim = (yHi - yLo + 1)/yBin                      # y-dimension
                        else:
                            raise 'ROI y-dimension must be a multiple of y binning value'
                        try:
                            self.cameras[iroi].set_roi(start_x=xLo, start_y=yLo, width=self.xDim, height=self.yDim, bins=xBin)
                        except ValueError as e:
                            print(e)
                else:
                    xBin,yBin,xLo,xHi,yLo,yHi=roiList[0]

                    if (xHi - xLo + 1) % xBin == 0:
                        self.xDim = (xHi - xLo + 1)/xBin                      # x-dimension
                    else:
                        raise 'ROI x-dimension must be a multiple of x binning value'
                    if (yHi - yLo + 1) % yBin == 0:
                        self.yDim = (yHi - yLo + 1)/yBin                      # y-dimension
                    else:
                        raise 'ROI y-dimension must be a multiple of y binning value'
                    for c in self.cameras:
                        try:
                            c.set_roi(start_x=xLo, start_y=yLo, width=self.xDim-1, height=self.yDim-1, bins=xBin)
                        except ValueError as e:
                            print(e)
            else:
                self.xDim, self.yDim = self.cameras[0].get_roi()[-2:]
        except Exception as e:
            self.handle_error(e)
    
    def setup_image(self):
        raise NotImplementedError('This function not yet implemented')

    def snap_image(self):
        raise NotImplementedError('This function not yet implemented')

    def setup_sequence(self, exp_time, EMgain=0, roiList=None, nframes=10):
        try:
            self.nframes = nframes
            self.setup_common(roiList)
            for i,c in enumerate(self.cameras):
                exp_time_us = int(exp_time * 1e6) # integer microseconds used in zwo cameras
                c.set_control_value(asi.ASI_EXPOSURE, exp_time_us)
                if type(EMgain) is list:
                    c.set_control_value(asi.ASI_GAIN, EMgain[i])
                else:
                    c.set_control_value(asi.ASI_GAIN, EMgain)
                c.sequenceData = numpy.empty((self.nframes, self.yDim, self.xDim))
                c.sequenceDataSize=self.nframes*self.yDim*self.xDim
            return exp_time, -1, -1 # accumulation time and kinetic time we dont get from the sdk so just return -1
        except Exception as e:
            self.handle_error(e)
    
    def externalTrigger(self):
        raise NotImplementedError('This function not yet implemented')

    def snap_sequence(self, timeout=10., timestamp=0):
        try:
            # NOTE THIS WILL NOT WORK FOR MORE THAN ONE CAMERA!!!
            startTime = time.time()
            timeout = int(timeout * 1e3) # timeout in ms not seconds
            c = self.cameras[0]
            c.start_video_capture()
            for i in range(self.nframes):
                c.sequenceData[i] = c.capture_video_frame(timeout=timeout)
            dropped_frames = c.get_dropped_frames()
            c.stop_video_capture()
            if timestamp > 0:
                return c.sequenceData, startTime
            else:
                return c.sequenceData
        except Exception as e:
            self.handle_error(e)

    def handle_error(self,e):
        print('ASI camera error!')
        print(e)
        print('Shutting down the camera(s) now')
        self.shutdown()
