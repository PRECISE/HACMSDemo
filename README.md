HACMSDemo
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
Peter Gebhard, Nicola Bezzo
PRECISE Center
Computer and Information Science Department
University of Pennsylania

Installation steps
------------------
Install the following required third-party libraries:
- ROS (ROSPy and paramiko, two python modules required by this software, are bundled as part of the ROS package)
- PyQt (PyQt 4.10.2, Qt4 version)
- pyqtgraph
- mapnik (package name: python-mapnik)

ROS setup tips:
- Ensure that the ~/.bashrc file contains the correct ROS\_IP and ROS\_MASTER\_URI values.

SSH connection tips:
- Get the LandShark black_box host key (use 'ssh-keyscan -t rsa <landshark\_ip>')
- Copy the host key to the local ~/.ssh/known_hosts file
- Ensure the 'hacms.cfg' file exists and contains the correct values (IP address, etc.)

************
TODO: Copy instructions here which Nico wrote for the SRI Git commit.
************

Credits
-------
Icons from famfamfam.com (http://www.famfamfam.com/lab/icons/silk/)
