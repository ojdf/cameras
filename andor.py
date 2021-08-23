'''Andor CCD control module. 
- Supports iXon and Luca. 
- Supports multiple cameras (only tested with Lucas).
- Currently assumes all cameras are the same model.
- Uses the 'ctypes' module to wrap the Andor C SDK.
- See the Andor SDK Manual for more info on how to use cameras.

Tim Butterley, Richard Wilson, James Osborn, Durham CfAI, 2004-2010
'''


import ctypes
import os
import sys
import time

import numpy
#import serial

#from AndorError import *




def getAndorErrorCodes(headerFile='/usr/local/include/atmcdLXd.h'):
    '''Function to extract error codes from the Andor SDK header file.'''
    f=open(headerFile)
    dat=f.readlines()
    f.close()
    pattern='#define DRV_'
    errorCodes={}
    for l in dat:
        if l[:len(pattern)]==pattern:
            x=l.split()
            errorCodes[int(x[2])]=x[1]
    return errorCodes


def dummyAndorErrorCodes(first=20000,last=21000):
    '''Function to generate a dummy error dictionary - just in case getAndorErrorCodes() is broken.'''
    errorCodes={}
    for i in range(first,last+1):
        errorCodes[i]='ERROR CODE '+str(i)
    return errorCodes


# Set up error codes
try:
    AndorError=getAndorErrorCodes()
except:
    #print 'Failed to get error codes from Andor header file.'
    AndorError=dummyAndorErrorCodes()

ignoredErrors=[20002,20034,20035,20036,20037]



class defaultParams:
    '''Class storing parameters for Andor cameras'''

    andorLib="/usr/local/lib/libandor.so"
    andorConfigDir="/usr/local/etc/andor"
   
    hasShutter=1   # 0 for no shutter
    operatingTemp=None

    # Parameters for iXon only (leave defaults for Luca)
    ixonADChannel=None
    ixonPreAmpGain=None
    ixonBaselineOffset=None
    ixonHSSpeed=None
    ixonVSSpeed=None
    ixonVSAmplitude=None

    # External trigger parameters
    triggerPort=None
    triggerSignal='A'
    triggerResponse='B'
    
    # Multiple camera parameters
    reverseOrder=0
    master=None
    
    def barf(self):
        '''Print parameter names and values'''
        ls=dir(self)
        ls.remove('__doc__')
        ls.remove('__module__')
        ls.remove('barf')
        for x in ls:
            print x,'=',eval('self.'+x)


class Luca(defaultParams):
    hasShutter=0
    operatingTemp=-20


class DV860(defaultParams):   #Original ESO SLODAR
    ixonHSSpeed=[0,0]
    ixonVSSpeed=2
    operatingTemp=-60


class DV887(defaultParams):   #GEMINI SLODAR
    ixonADChannel=0
    ixonPreAmpGain=0
    ixonHSSpeed=[0,0]
    ixonVSSpeed=2
    operatingTemp=-60


class DV885(defaultParams):   #Durham SLODAR
    ixonADChannel=0
    ixonPreAmpGain=1
    ixonHSSpeed=[0,0]
    ixonVSSpeed=2  #was 4 but 4 doesn't exist!
    operatingTemp=-60




class CameraObject:
    def __init__(self):
        self.handle=ctypes.c_long()
        self.serialNumber=None
        self.model=None
        self.currentTemp=None
        self.imageData=None
        self.sequenceData=None
        self.imageDataSize=None
        self.sequenceDataSize=None
        self.xPix=None
        self.yPix=None
        self.actualExpTime=None
        self.actualAccTime=None
        self.actualKinTime=None
        self.lastError=None

    def barf(self):
        '''Print parameter names and values'''
        ls=dir(self)
        ls.remove('__init__')
        ls.remove('__doc__')
        ls.remove('__module__')
        ls.remove('barf')
        ls.remove('imageData')
        ls.remove('sequenceData')
        ls.remove('lastError')
        for x in ls:
            print x,'=',eval('self.'+x)



class Andor:

    def __init__(self,params=defaultParams):  #change defaults for iXon only
        '''Initialise all CCDs and open their shutters'''

        self.params=params
        self.libc = ctypes.cdll.LoadLibrary(self.params.andorLib)

        self.nCameras=0 #set this before first "self.call()"
        self.shutting_down=0   #flag used by temp()

        # Find out how many cameras are visible
        nCam = ctypes.c_int()
        self.call(None,'GetAvailableCameras',ctypes.byref(nCam))
        self.nCameras=nCam.value

        # Get handles for all visible cameras
        self.cameras=[]
        for i in range(self.nCameras):
            self.cameras.append(CameraObject())
            self.call(None,'GetCameraHandle',ctypes.c_int(i),ctypes.byref(self.cameras[i].handle))

        # Initialise all cameras
        self.call(self.cameras,'Initialize',self.params.andorConfigDir)
        for cam in self.cameras:
            if cam.lastError != 20002:
                print 'andor: Camera(s) failed to initialise, shutting down.'
                self.call(self.cameras,'ShutDown')        
                sys.exit()

        # Get camera properties
        ser=ctypes.c_int()
        x=ctypes.c_int()
        y=ctypes.c_int()
        ManyChars = ctypes.c_char * 256
        m=ManyChars()
        for cam in self.cameras:
            self.call(cam,'GetCameraSerialNumber',ctypes.byref(ser))
            self.call(cam,'GetDetector',ctypes.byref(x),ctypes.byref(y))
            self.call(cam,'GetHeadModel',ctypes.byref(m))
            cam.serialNumber=ser.value
            cam.xPix=x.value
            cam.yPix=y.value
            cam.model=m.value
            print cam.serialNumber,cam.xPix,cam.yPix,cam.model

        # Sort cameras by serial number (so same one always comes first)
        self.cameras.sort(key=lambda x: x.serialNumber, reverse=self.params.reverseOrder)
        
        # Check camera order
        serList=[]
        for cam in self.cameras:
            serList.append(cam.serialNumber)
        print serList

        if self.params.operatingTemp!=None:
            self.cooler_on(self.params.operatingTemp)

        if self.params.hasShutter>0:
            self.shutter_open()   # open shutter

        if self.params.ixonADChannel!=None:   # set AD Channel (if necessary) - IXON ONLY
            self.call(self.cameras,'SetADChannel',ctypes.c_int(self.params.ixonADChannel))

        if self.params.ixonPreAmpGain!=None:   # set pre-amp gain (if necessary) - IXON ONLY
            self.call(self.cameras,'SetPreAmpGain',ctypes.c_int(self.params.ixonPreAmpGain))

        if self.params.ixonVSAmplitude!=None:   # set VS amplitude (if necessary) - IXON ONLY
            self.call(self.cameras,'SetVSAmplitude',ctypes.c_int(self.params.ixonVSAmplitude))

        if self.params.ixonBaselineOffset!=None:   # set baseline offset i.e. bias (if necessary) - IXON ONLY
            self.call(self.cameras,'SetPreAmpGain',ctypes.c_int(self.params.ixonBaselineOffset))


    def call(self,cameras,func,*params):
        '''Call a libandor function for specified camera(s).'''
        if type(cameras) != type([]):
            if cameras==None:
                cams=[]  #empty list
            elif type(cameras)==type(CameraObject()):
                cams=[cameras]  #list of one object
            else:
                raise 'andor.Andor.call error: cameras must be a None a camera object or a list of camera objects'
        else:
            cams=cameras

        if len(cams)<1 or self.nCameras<2:
            err=eval('self.libc.'+func)(*params)
            if err not in ignoredErrors:   # != 20002:
                print 'Andor error: '+func+', ', AndorError[err]
                #raise 'Andor error: '+`func`+',', AndorError[err]
                #raise 'arse'
            if len(cams)>0:
                cams[0].lastError=err
            return err
        else:
            errs=[]
            for cam in cams:
                err = self.libc.SetCurrentCamera(cam.handle)
                if err not in ignoredErrors:   # != 20002:
                    raise 'Andor error: SetCurrentCamera,', AndorError[err]
                err=eval('self.libc.'+func)(*params)
                cam.lastError=err
                if err not in ignoredErrors:   # != 20002:
                    print 'Andor error: '+func+', ', AndorError[err]
                    #raise 'Andor error: '+`func`+',', AndorError[err]	 
                errs.append(err)
            return errs


    def shutter_open(self):
        '''Open the shutter.'''
        self.call(self.cameras,'SetShutter',ctypes.c_int(1),ctypes.c_int(1),ctypes.c_int(50),ctypes.c_int(50))


    def shutter_close(self):
        '''Close the shutter.'''
        self.call(self.cameras,'SetShutter',ctypes.c_int(1),ctypes.c_int(2),ctypes.c_int(50),ctypes.c_int(50))


    def cooler_on(self, temp):
        '''Switch on the Peltier cooler.'''
        self.call(self.cameras,'SetTemperature',ctypes.c_int(temp))
        self.call(self.cameras,'CoolerON')


    def cooler_off(self):
        '''Switch off the Peltier cooler.'''
        self.call(self.cameras,'CoolerOFF')
 

    def set_temp(self, temp):
        '''Set the target CCD temperature.'''
        self.call(self.cameras,'SetTemperature',ctypes.c_int(temp))


    def temp(self,internal=0):
        '''Get the current CCD temperature.'''
        if self.shutting_down and internal<1:  # shutdown operation
            return self.current_temp
        else:  # normal operation
            t=ctypes.c_int()
            for cam in self.cameras:
                self.call(cam,'GetTemperature',ctypes.byref(t))
                cam.currentTemp=t.value
            if self.nCameras==1:
                return self.cameras[0].currentTemp
            else:
                return [cam.currentTemp for cam in self.cameras]   #return integer for single camera


    def cooler_wait(self, target_temp):
        '''Check the CCD temperature in a loop, waiting for it to cool to a target temperature - DEFUNCT?'''
        temp = self.temp()
        if type(temp)==type([]):
            temp=max(temp)
        while temp > target_temp:
            temp = self.temp()
            print temp
            if type(temp)==type([]):
                temp=max(temp)
            time.sleep(1)


    def shutdown2(self):
        '''Shutdown the CCD(s) immediately. Assumes shutter has been closed and camera allowed to warm up first.'''
        err = self.call(self.cameras,'ShutDown')
        return err


    def shutdown(self,safe_temp=-30):
        '''Shutdown the CCD, closing the shutter and warming up to specified temperature first.'''
        self.shutting_down=1
        self.current_temp=self.temp(internal=1)
        if self.nCameras>1:
            t=min(self.current_temp)
        else:
            t=self.current_temp
	if 'Luc' not in self.cameras[0].model:
       	    self.shutter_close()
        self.call(self.cameras,'CoolerOFF')
        self.shutting_down=1
        while t < safe_temp:
	    #print t, safe_temp
            self.current_temp=self.temp(internal=1)
            if self.nCameras>1:
                t=min(self.current_temp)
            else:
                t=self.current_temp
            time.sleep(2)
	#print 'bob'
        err = self.call(self.cameras,'ShutDown')  
	#print 'bob2'      
        return err

    
    def setupCommon(self, roiList):
        '''Setup camera options common to both setup_image and setup_sequence.'''
        ## Tidy this up
        #if type(roiList)=='list':
        #print 'roi',roiList
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

                    self.call(self.cameras[iroi],'SetReadMode',ctypes.c_int(4))
                    self.call(self.cameras[iroi],'SetImage',ctypes.c_int(xBin), ctypes.c_int(yBin), ctypes.c_int(xLo), ctypes.c_int(xHi), ctypes.c_int(yLo), ctypes.c_int(yHi))
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

                self.call(self.cameras,'SetReadMode',ctypes.c_int(4))
                self.call(self.cameras,'SetImage',ctypes.c_int(xBin), ctypes.c_int(yBin), ctypes.c_int(xLo), ctypes.c_int(xHi), ctypes.c_int(yLo), ctypes.c_int(yHi))
        else:
            xBin=1
            yBin=1
            xLo=1
            xHi=self.cameras[0].xPix
            yLo=1
            yHi=self.cameras[0].yPix


            if (xHi - xLo + 1) % xBin == 0:
                self.xDim = (xHi - xLo + 1)/xBin                      # x-dimension
            else:
                raise 'ROI x-dimension must be a multiple of x binning value'
            if (yHi - yLo + 1) % yBin == 0:
                self.yDim = (yHi - yLo + 1)/yBin                      # y-dimension
            else:
                raise 'ROI y-dimension must be a multiple of y binning value'

            self.call(self.cameras,'SetReadMode',ctypes.c_int(4))
            self.call(self.cameras,'SetImage',ctypes.c_int(xBin), ctypes.c_int(yBin), ctypes.c_int(xLo), ctypes.c_int(xHi), ctypes.c_int(yLo), ctypes.c_int(yHi))

        if self.params.ixonVSSpeed!=None:
            self.call(self.cameras,'SetVSSpeed',ctypes.c_int(self.params.ixonVSSpeed))
        if self.params.ixonHSSpeed!=None:
            self.call(self.cameras,'SetHSSpeed',ctypes.c_int(self.params.ixonHSSpeed[0]), ctypes.c_int(self.params.ixonHSSpeed[1]))


    def setup_image(self, exp_time, EMgain=0, roiList=None):                     # exp_time=FLOAT, EMgain=INT (0-255), ROI=[xbin,ybin,xlo,xhi,ylo,yhi]
        '''Setup the camera for single image acqisition (mode 1).'''
        self.setupCommon(roiList)
        self.call(self.cameras,'SetAcquisitionMode',ctypes.c_int(1))
        self.call(self.cameras,'SetExposureTime',ctypes.c_float(exp_time))
        self.call(self.cameras,'SetEMCCDGain',ctypes.c_int(EMgain))

        self.call(self.cameras,'SetTriggerMode',ctypes.c_int(0)) #0: no trigger, 6: ext trigger
       
        for i in range(self.nCameras):
            self.cameras[i].imageData=numpy.zeros((self.yDim,self.xDim),numpy.uint16)
            self.cameras[i].imageDataSize=self.yDim*self.xDim


    def snap_image(self,sifFile=None):
        '''Grab a single image, with parameters as defined by the call to setup_image.'''
        self.call(self.cameras,'StartAcquisition')
        
        stat = ctypes.c_int()
        for cam in self.cameras:  #poll each CCD in turn to make sure they've all finished acquiring
            self.call(cam,'GetStatus',ctypes.byref(stat))
            while stat.value == 20072:
                self.call(cam,'GetStatus',ctypes.byref(stat))
 
        for i in range(self.nCameras):
            self.call(self.cameras[i],'GetAcquiredData16',ctypes.c_long(self.cameras[i].imageData.ctypes.data),ctypes.c_long(self.cameras[i].imageDataSize))
            if sifFile != None:
                print 'Saving SIF file:',sifFile
                self.call(self.cameras[i],'SaveAsSif',ctypes.c_char_p(sifFile+str(self.cameras[i].handle.value)+'.sif'))

        if self.nCameras==1:
            return self.cameras[0].imageData
        else:
            return [cam.imageData for cam in self.cameras]


    def setup_sequence(self, exp_time, EMgain=0, roiList=None, nframes=10):         # exp_time=FLOAT, EMgain=INT (0-2 55), ROI=[xbin,ybin,xlo,xhi,ylo,yhi]
        '''Setup the camera for image sequence acquisition (mode 4). N.B. Make sure DMA memory size is sufficient for iXon.'''

        self.nframes=nframes
        self.setupCommon(roiList)
        self.call(self.cameras,'SetAcquisitionMode',ctypes.c_int(3))
        self.call(self.cameras,'SetExposureTime',ctypes.c_float(exp_time))
        if type(EMgain)==list:
            for iCam in range(self.nCameras):
                EMgain[iCam] = int(EMgain[iCam])
                self.call(self.cameras[iCam],'SetEMCCDGain',ctypes.c_int(EMgain[iCam]))
        else:
            EMgain = int(EMgain)
            self.call(self.cameras,'SetEMCCDGain',ctypes.c_int(EMgain))
        if self.params.triggerPort==None:
            self.call(self.cameras,'SetTriggerMode',ctypes.c_int(0)) #0: no trigger, 6: ext trigger
        else:
            self.call(self.cameras,'SetTriggerMode',ctypes.c_int(6)) #0: no trigger, 6: ext trigger

        if self.params.master != None:
            for cam in self.cameras:
                if cam.serialNumber == self.params.master:
                    self.call(cam,'SetTriggerMode',ctypes.c_int(0)) #0: no trigger, 6: ext trigger
                    #print 'Master: ',cam.serialNumber
                else:
                    self.call(cam,'SetTriggerMode',ctypes.c_int(6)) #0: no trigger, 6: ext trigger
                    #print 'Slave: ',cam.serialNumber

        self.call(self.cameras,'SetNumberAccumulations',ctypes.c_int(1))
        self.call(self.cameras,'SetAccumulationCycleTime',ctypes.c_float(0.))
        self.call(self.cameras,'SetNumberKinetics',ctypes.c_int(nframes))
        self.call(self.cameras,'SetKineticCycleTime',ctypes.c_float(0))

        for i in range(self.nCameras):
            self.cameras[i].sequenceData=numpy.empty((self.nframes,self.yDim,self.xDim),numpy.uint16)
            self.cameras[i].sequenceDataSize=self.nframes*self.yDim*self.xDim

        expT=ctypes.c_float()
        accT=ctypes.c_float()
        kinT=ctypes.c_float()
        for cam in self.cameras:
            self.call(self.cameras,'GetAcquisitionTimings',ctypes.byref(expT),ctypes.byref(accT),ctypes.byref(kinT))
            cam.actualExpTime=expT.value
            cam.actualAccTime=accT.value
            cam.actualKinTime=kinT.value

        return expT.value,accT.value,kinT.value


    def externalTrigger(self):
        '''Send external trigger signal via serial port.'''
        trig_port=serial.Serial(self.params.triggerPort,9600,parity='N',bytesize=8,stopbits=1,rtscts=0,xonxoff=0,timeout=1)
        trig_port.write(self.params.triggerSignal)
        if self.params.triggerResponse != None:
            resp=trig_port.readline().count(self.params.triggerResponse)
            if resp < 1:
                print 'No response from trigger box!'
        trig_port.close()


    def snap_sequence(self,timeout=10.,timestamp=0):
        '''Grab a sequence of images, with parameters as defined by the call to setup_sequence.'''
        startTime=time.time()
        
        #self.call(self.cameras,'StartAcquisition')
        if self.params.master != None:
            masterCam=None
            for cam in self.cameras:
                if cam.serialNumber == self.params.master:
                    masterCam=cam
                else:
                    self.call(cam,'StartAcquisition')
                    #print 'Slave: ',cam.serialNumber
            time.sleep(0.03)
            self.call(masterCam,'StartAcquisition')
            #print 'Master: ',cam.serialNumber
        else:
            self.call(self.cameras,'StartAcquisition')


        if self.params.triggerPort!=None:  #send serial trigger signal
            time.sleep(0.1)
            startTime=time.time()
            self.externalTrigger()

        t0=time.time()
        waiting=1
        while waiting>0:
            if time.time()-t0 > timeout:  #check acquisition hasn't timed out, restart if it has
                print 'snap_sequence(): TIMED OUT'
                self.call(self.cameras,'AbortAcquisition')
                print 'snap_sequence(): Attempting to restart acquisition'
                startTime=time.time()
 
                #self.call(self.cameras,'StartAcquisition')
                if self.params.master != None:
                    masterCam=None
                    for cam in self.cameras:
                        if cam.serialNumber == self.params.master:
                            masterCam=cam
                        else:
                            self.call(cam,'StartAcquisition')
                            #print 'Slave: ',cam.serialNumber
                    time.sleep(0.03)
                    self.call(masterCam,'StartAcquisition')
                    #print 'Master: ',cam.serialNumber
                else:
                    self.call(self.cameras,'StartAcquisition')
                if self.triggerPort!=None:  #send serial trigger signal
                    time.sleep(0.5)
                    startTime=time.time()
                    self.externalTrigger()

                t0=time.time()

            stat = ctypes.c_int()  #check whether data acquisition has finished
            waiting=0
            for cam in self.cameras:
                #print cam
                self.call(cam,'GetStatus',ctypes.byref(stat))
                if stat.value == 20072:  #still acquiring
                    waiting+=1

        #t1=time.time()  #end time (apparently not used for anything!)
        for i in range(self.nCameras):
            self.call(self.cameras[i],'GetAcquiredData16',ctypes.c_long(self.cameras[i].sequenceData.ctypes.data),ctypes.c_long(self.cameras[i].sequenceDataSize))

        if self.nCameras==1:
            if timestamp>0:
                return self.cameras[0].sequenceData, startTime
            else:
                return self.cameras[0].sequenceData
        else:
            if timestamp>0:
                return [cam.sequenceData for cam in self.cameras], startTime
            else:
                return [cam.sequenceData for cam in self.cameras]


    def test(self):
        '''Ask camera for current acquisition timings.'''
        for cam in self.cameras:
            print 'Camera ',cam.handle.value
            x=ctypes.c_float()
            y=ctypes.c_float()
            z=ctypes.c_float()
            self.call(cam,'GetAcquisitionTimings',ctypes.byref(x),ctypes.byref(y),ctypes.byref(z))
            print '  exposure',x.value
            print '  accumulate',y.value
            print '  kinetic',z.value
            print '  Frame rate =',1./z.value,'Hz'





if __name__=='__main__':
    import numpyFITS as FITS

    print 'Testing andor.py (assuming Luca)'
    par=Luca()
    print 'Initialising'
    ccd=Andor(par)
    print [cam.serialNumber for cam in ccd.cameras]

    print 'Setting up'
    for cam in ccd.cameras:
        cam.barf()
        print

    ccd.setup_image(0.02) #,roiList=[1,1,1,128,1,128])
    print 'Snapping'
    im=ccd.snap_image('test')
    if type(im)==type([]):
        print [x.shape for x in im]
    else:
        print im.shape

    print 'Cooling'
    ccd.cooler_on(-20)
    ccd.cooler_wait(-15)
    
    print 'Getting sequence'
    ccd.setup_sequence(0.02,200,nframes=200) #,roiList=[1,1,1,128,1,128])
    dat=ccd.snap_sequence()
    if type(dat)==type([]):
        for i in range(len(dat)):
            FITS.Write(dat[i],'data'+str(i+1)+'.fits')
    else:
        FITS.Write(dat,'data.fits')
    

    for cam in ccd.cameras:
        cam.barf()
        print
          
    print 'Shutting down'
    #ccd.shutdown2()
    ccd.shutdown(safe_temp=0)
    print 'OK'





