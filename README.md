
#Motivepy is a Python API for communicating with OptiTrack cameras.

##Why we wrote Motivepy
The main goal of the API is to substitute the Motive GUI
whenever it is convenient.

For this we wrapped most of OptiTrack's C++ Motive SDK with Cython.
We tried to structure Motivepy as pythonic as possible and also
added some stability to the raw functionality of the original SDK.
In addition we developed some advanced features like a 3D viewer
script for tracked points.

##How to use Motivepy
First note that motivepy is compatible with 32 and 64bit. In the following
we only describe the 32bit case, but 64bit is analogous

###Install Motive
Go to https://www.optitrack.com/downloads/ download and install Motive 1.9
(Or older. Newer versions should also work, but have not been tested).

###Install Python
To use a Python API you need to install Python on your machine.
We recommend to install Python 2.7 and make sure it runs in the windows console
(i.e. open the Command Window, type *python* and hit enter. If python starts, then it's installed!
If it is installed but not working, try and add the following adresses (or depending where you installed stuff) to the PATH variable::
  ```
  C:\Python27\; C:\Python27\Scripts\
  ```

###Install Cython
Open a command window and write::
  ```
  pip install cython 0.22.1
  ```

It is important to use Cython version 0.22.1, as later versions need a different code
for the pointcloudgroup functions to work (Feel free to try it out and comment,
it should actually be easier then what we did with 0.22.1).

###Download Motivepy
A convenient way to download motivepy would be to clone the repository using *git* (you may need to install git as well: https://git-scm.com/downloads)
and download it from the command window into the folder of your choice using the command::

  ```
  git clone https://github.com/NicolasKuske/motivepy.git
  ```

###Install Motivepy
Before you can install Motivepy you need to add the following adresses (or depending where you installed stuff) to the PATH variable::
  ```
  C:\Program Files (x86)\OptiTrack\Motive\lib; C:\Program Files (x86)\OptiTrack\Motive
  ```
Then navigate into the motivepy directory via the command window (*cd motivepy*) and run the install option on the setup.py script::

  ```
  cd motivepy
  python setup.py install
  ```

(In case you are still missing dll dependencies, download and open Dependency Walker. There run the dll or program
giving the error)

###And Now

Now go to http://nicolaskuske.github.io/motivepy/ and start with the tutorials.

Njoy!







