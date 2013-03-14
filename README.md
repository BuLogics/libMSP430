# About
libMSP430 is a friendly fork-and-fix of the canonocal TI 'MSP430DLLv3' for linux, 
which is previously only available as a zip file with a hard to find patch for 
Debian/Ubuntu.

When trying to use mspdebug using the 'tilib' option on my persoan Linux box, I 
could not find a working zip of 'MSP430DLLv3' to build, so I collected the 
patches and posted them in this repo to share.


# Branches
  master : our patched branch of MSP430DLLv3 

# instructions
 0. check out this repository
 1. run './ubuntu_install_prereqes.sh'
 2. run ' mspdebug tilib -d /dev/ttyACM0' (assuming your ti device is on ACM0)
    and verify you have an error loading 'libmsp430.so' on your machine
 3. run 'make' If you are lucky, this should *just work* 
 4. run 'make install' to install the driver, so mspdebug can find the lib
 5. Plug in a MSP-FET430UIF device into your machine
 6. run ' mspdebug tilib -d /dev/ttyACM0' (assuming your ti device is on ACM0) 
    it should work now.

                                                                
#See Also:                
 - Unnamed TI employee post/patch:
 : http://e2e.ti.com/support/microcontrollers/msp430/f/166/p/214038/756939.aspx
 - This mspdebug linux install info: 
 http://e2e.ti.com/support/microcontrollers/msp430/f/166/p/214038/756939.aspx
 - the device this is driver is building for:
 https://github.com/BuLogics/libMSP430
