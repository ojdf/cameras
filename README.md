# cameras

Files for interfacing with cameras in python. `andor.py` is the original, with other files defined to be drop-in replacements for using different cameras with CfAI instruments (e.g. SCIDAR).

## Requirements

### ZWO
First you need to install the sdk from here under the "Developers” section here: [https://astronomy-imaging-camera.com/software-drivers](https://astronomy-imaging-camera.com/software-drivers)

Then get the python bindings from here: [https://github.com/stevemarple/python-zwoasi](https://github.com/stevemarple/python-zwoasi)

Once both of those are installed, you should be able to `import zwoasi` (it may show an error about not finding the SDK, ignore it).

You then need to set the `ASI_LIB` attribute in `zwo.py` to point to the correct 
path (from the Andor SDK install), which should be a `.so`, `.dylib` or `.dll` depending on platform.

If it is working, you should be able to get frames from a camera with something like the following:
```
import zwo
Z = zwo.ZWO()
Z.setup_sequence(exposure_time, EMgain=gain, nframes=N)
frames = Z.snap_sequence()
```

Note that some 

### Andor
Some sort of Andor SDK? Idk. Definitely works with an Andor Luca on some ancient version of Ubuntu.
