HACMS Demo Application
=========

License
-------
Copyright (c) 2013, The Trustees of the University of Pennsylvania.
Developed with the sponsorship of the Defense Advanced Research Projects
Agency (DARPA).

Permission is hereby granted, free of charge, to any person obtaining a
copy of this data, including any software or models in source or binary
form, as well as any drawings, specifications, and documentation
(collectively "the Data"), to deal in the Data without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Data, and to
permit persons to whom the Data is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Data.

THE DATA IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS, SPONSORS, DEVELOPERS, CONTRIBUTORS, OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE DATA OR THE USE OR OTHER DEALINGS IN THE DATA.

Authors
-------
[Peter Gebhard](http://www.seas.upenn.edu/~pgeb), [Nicola Bezzo](http://www.seas.upenn.edu/~nicbezzo)

[PRECISE Center](http://precise.seas.upenn.edu)

[Computer and Information Science Department](http://www.cis.upenn.edu/)

[University of Pennsylania](http://www.seas.upenn.edu/)

Installation steps
------------------
Install the following required third-party libraries:
- [ROS](http://wiki.ros.org/) (rospy, PyQt, and paramiko, modules required by this software, are bundled as part of ROS)
    - [rospy](http://wiki.ros.org/rospy)
    - [PyQt](http://www.riverbankcomputing.com/software/pyqt/intro)
    - [paramiko](https://github.com/paramiko/paramiko)
- [pyqtgraph](http://www.pyqtgraph.org/)
    - [SciPy](http://www.scipy.org/)
    - [NumPy](http://www.numpy.org/)
    - [gfortran](http://gcc.gnu.org/wiki/GFortran) (for installing SciPy and NumPy)

ROS setup tips:
- Make sure your ~/.bashrc file contains the correct ROS\_IP and ROS\_MASTER\_URI values.

SSH connection tips:
- Find the LandShark black_box host key (use 'ssh-keyscan -t rsa <landshark\_ip>')
- Copy the host key to the local ~/.ssh/known_hosts file
- Ensure the 'hacms.cfg' file exists and contains the correct values (IP address, etc.)
    - See /hacms/hacms.cfg_sample.txt for an example.


Credits
-------
Icons from [famfamfam.com](http://www.famfamfam.com/lab/icons/silk/)
